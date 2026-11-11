//! The twin's turning, simulated in the half section: side 1 from the bar, then side 2 on what
//! side 1 cut off (turned end for end), each checked as the lathe's simulation checks it
//! ([`crate::lathe::sim`]); side 2's comparison with the part is the finished part's.

use schemars::JsonSchema;
use serde::Serialize;

use super::job::CtxJob;
use super::plan::CtxPlan;
use crate::lathe::sim::{LatheSimReport, simulate_from};
use crate::region::{Area, Ring};
use crate::{CamError, Result};

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct CtxSimReport {
    pub side1: LatheSimReport,
    /// Its `left` and `gouges` are the finished part's.
    pub side2: LatheSimReport,
    /// Where a live tool cut into the part.
    pub live_gouges: Vec<LiveGouge>,
}

/// A live tool's point inside the finished part: the side, the operation, the tool and where
/// (the side's work frame).
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct LiveGouge {
    pub side: u8,
    pub operation: String,
    pub tool: u32,
    pub at: [f64; 3],
}

/// `a` turned end for end about work Z `-l / 2`: z → −l − z.
fn turned(a: &Area, l: f64) -> Result<Area> {
    let rings: Vec<Ring> = a.rings().map(|r| r.iter().map(|p| [-l - p[0], p[1]]).collect()).collect();
    Area::from_rings(&rings)
}

/// Simulate both sides' turning; check the live tools against the part `body`; the half sections
/// left on side 1 (the bar and the part) and side 2.
pub fn simulate(job: &CtxJob, plan: &CtxPlan, body: &solidcraft_kernel::Body) -> Result<(CtxSimReport, Area, Area)> {
    let job1 = job.side_job(1, 0.0)?;
    let (r1, left1) = simulate_from(&job1, &plan.side1, None)?;
    let parting = plan.parting().ok_or_else(|| CamError::Invalid("side 1 doesn't part off".into()))?;
    let w = job1.tool(parting.tool).map_or(0.0, |t| t.blade_width());
    let cut = parting.moves.iter().map(|m| m.to()[1]).fold(f64::INFINITY, f64::min) + w;
    let su = &plan.side1.setup;
    let piece = left1.intersection(&Area::rect(cut, -1.0, su.front + 1.0, su.radius + 1.0)?)?;
    let l = plan.side1.profile.length;
    let job2 = job.side_job(2, plan.grip)?;
    let (r2, left2) = simulate_from(&job2, &plan.side2, Some(turned(&piece, l)?))?;
    let mut live_gouges = Vec::new();
    for (side, ops) in [(1u8, &plan.live1), (2u8, &plan.live2)] {
        if ops.is_empty() {
            continue;
        }
        let b = super::live::side_body(body, job.part.front, side)?;
        for op in ops {
            live_gouges.extend(live_check(job, &b, op, side)?);
        }
    }
    Ok((CtxSimReport { side1: r1, side2: r2, live_gouges }, left1, left2))
}

/// Tool positions along the live moves this far apart at most, mm.
const LIVE_STEP: f64 = 1.0;
/// The tool's surface is checked this far inside its true shape: contact with the part's own
/// faces is no gouge, and the exact classification is not to be trusted closer than a few
/// hundredths to a face's seam (a point 0.02 inside a drilled hole's wall at its seam reads as
/// in the material), mm.
const LIVE_TOL: f64 = 0.05;
/// Most points one operation's check classifies.
const LIVE_MAX_POINTS: usize = 6000;

/// Points on a cutter of radius `r` whose end centre is `c`, axis `a` (unit, from the end into
/// the tool), `len` of it: its end's rim and centre, and a ring up its side.
fn cutter_points(c: [f64; 3], a: [f64; 3], r: f64, len: f64, out: &mut Vec<[f64; 3]>) {
    // two directions square to the axis
    let u = if a[2].abs() < 0.9 { [a[1], -a[0], 0.0] } else { [1.0, 0.0, 0.0] };
    let ul = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt().max(1e-12);
    let u = [u[0] / ul, u[1] / ul, u[2] / ul];
    let v = [a[1] * u[2] - a[2] * u[1], a[2] * u[0] - a[0] * u[2], a[0] * u[1] - a[1] * u[0]];
    let rr = (r - LIVE_TOL).max(0.0);
    let at = |h: f64, ang: f64, rad: f64| {
        let (cs, sn) = (ang.cos() * rad, ang.sin() * rad);
        [c[0] + a[0] * h + u[0] * cs + v[0] * sn, c[1] + a[1] * h + u[1] * cs + v[1] * sn, c[2] + a[2] * h + u[2] * cs + v[2] * sn]
    };
    out.push(at(LIVE_TOL, 0.0, 0.0));
    for k in 0..8 {
        let ang = k as f64 * std::f64::consts::FRAC_PI_4;
        out.push(at(LIVE_TOL, ang, rr));
        out.push(at(len.min(r), ang + 0.3, rr));
    }
}

/// Classify points along a live operation's tool positions against the part `b` (the side's
/// frame): those inside it are gouges.
fn live_check(job: &CtxJob, b: &solidcraft_kernel::Body, op: &super::live::LiveOp, side: u8) -> Result<Vec<LiveGouge>> {
    use super::live::{LiveKind, LiveMove};
    let Some(t) = job.live(op.tool) else { return Ok(Vec::new()) };
    let r = t.diameter / 2.0;
    let mut pts: Vec<[f64; 3]> = Vec::new();
    match op.kind {
        LiveKind::FaceMill => {
            let mut at: Option<[f64; 3]> = None;
            for m in &op.moves {
                let to = m.to();
                let from = at.unwrap_or(to);
                let len = ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2) + (to[2] - from[2]).powi(2)).sqrt();
                let n = (len / LIVE_STEP).ceil().max(1.0) as usize;
                for k in 1..=n {
                    let s = k as f64 / n as f64;
                    let c = [from[0] + (to[0] - from[0]) * s, from[1] + (to[1] - from[1]) * s, from[2] + (to[2] - from[2]) * s];
                    cutter_points(c, [0.0, 0.0, 1.0], r, t.flute_length, &mut pts);
                }
                if matches!(m, LiveMove::Rapid(_)) {
                    // where polar interpolation comes on: in the air
                }
                at = Some(to);
            }
        }
        LiveKind::AxialDrill => {
            for h in &op.holes {
                let (cx, cy) = (h.r * h.c.to_radians().cos(), h.r * h.c.to_radians().sin());
                let n = ((h.z + 1.0 - h.to) / LIVE_STEP).ceil().max(1.0) as usize;
                for k in 0..=n {
                    let z = h.to + (h.z + 1.0 - h.to) * k as f64 / n as f64;
                    cutter_points([cx, cy, z], [0.0, 0.0, 1.0], r, 0.0, &mut pts);
                }
            }
        }
        LiveKind::RadialDrill => {
            for h in &op.holes {
                let d = [h.c.to_radians().cos(), h.c.to_radians().sin(), 0.0];
                let n = ((h.r + 1.0 - h.to) / LIVE_STEP).ceil().max(1.0) as usize;
                for k in 0..=n {
                    let x = h.to + (h.r + 1.0 - h.to) * k as f64 / n as f64;
                    cutter_points([d[0] * x, d[1] * x, h.z], d, r, 0.0, &mut pts);
                }
            }
        }
        LiveKind::Keyway => {
            let d = [op.c.to_radians().cos(), op.c.to_radians().sin(), 0.0];
            let mut at: Option<[f64; 3]> = None;
            for m in &op.moves {
                let to = m.to();
                let from = at.unwrap_or(to);
                let len = (to[0] - from[0]).hypot(to[2] - from[2]);
                let n = (len / LIVE_STEP).ceil().max(1.0) as usize;
                for k in 1..=n {
                    let s = k as f64 / n as f64;
                    let (x, z) = (from[0] + (to[0] - from[0]) * s, from[2] + (to[2] - from[2]) * s);
                    cutter_points([d[0] * x, d[1] * x, z], d, r, t.flute_length, &mut pts);
                }
                at = Some(to);
            }
        }
    }
    // evenly thinned to the budget
    let keep = pts.len().div_ceil(LIVE_MAX_POINTS).max(1);
    let mut out = Vec::new();
    for p in pts.iter().step_by(keep) {
        let q = solidcore_record::Vec3::new(p[0], p[1], p[2]);
        if b.classify_point(q)? == solidcraft_kernel::PointClass::Inside && out.len() < 50 {
            out.push(LiveGouge { side, operation: op.name.clone(), tool: op.tool, at: *p });
        }
    }
    Ok(out)
}
