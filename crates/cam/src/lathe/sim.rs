//! Turning, simulated in the bar's half section (z along the axis, r out from it, work
//! coordinates): each move sweeps the tool's insert (a rhombus of its insert angle at its holder's
//! approach, its corner rounded to the nose radius) and takes what it covers from the bar. What
//! is left is compared with the part: material cut from it (gouges), material left on it, rapids
//! through material and the tools coming at the jaws.

use schemars::JsonSchema;
use serde::Serialize;

use super::job::{LatheJob, LatheTool};
use super::plan::{LathePlan, TipMove};
use crate::region::{Area, Ring};
use crate::{CamError, Result};

/// Length of the insert's edges in the simulation, mm.
const EDGE: f64 = 6.0;
/// Pieces smaller than this are ignored (a nose radius's fillet in a square inside corner is
/// r²(1 − π/4): 0.0086 mm² at R0.2), mm².
const AREA_TOL: f64 = 0.01;

/// Pieces thinner than this on average along the axis are the paths' and the outline's round-off,
/// mm.
const MIN_THICKNESS: f64 = 0.002;

/// A piece of material, in work coordinates: its Z span, its diameter span and its area (mm²).
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct Piece {
    pub z: [f64; 2],
    pub diameter: [f64; 2],
    pub area: f64,
}

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct RapidHit {
    pub operation: String,
    /// The tip's X (diameter) and Z where the rapid ended.
    pub at: [f64; 2],
    pub area: f64,
}

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct LatheSimReport {
    /// Half-section area turned away, mm² (times 2π r̄ for a volume).
    pub removed_area: f64,
    /// Material cut from the part, largest first.
    pub gouges: Vec<Piece>,
    /// Material left on the part, largest first (the plan's warnings name what its tools can't
    /// reach).
    pub left: Vec<Piece>,
    pub rapid_hits: Vec<RapidHit>,
    /// Moves reaching behind the jaws' face.
    pub jaw_hits: usize,
}

/// A unit vector `deg` degrees round from +z toward +r.
fn dir(deg: f64) -> [f64; 2] {
    let a = deg.to_radians();
    [a.cos(), a.sin()]
}

/// The insert of `t` with its nose's centre at `c` ([z, r]): its outline's points.
pub fn insert(t: &LatheTool, c: [f64; 2]) -> Vec<[f64; 2]> {
    let (lead, trail) = (t.lead_angle(), t.trail_angle());
    let half = (t.insert_angle_deg / 2.0).to_radians();
    let rn = t.nose_radius;
    let bis = dir((lead + trail) / 2.0);
    // the sharp corner the nose rounds, and where the round meets the edges
    let corner = [c[0] - bis[0] * rn / half.sin(), c[1] - bis[1] * rn / half.sin()];
    let (ul, ut) = (dir(lead), dir(trail));
    let a = [corner[0] + ul[0] * EDGE, corner[1] + ul[1] * EDGE];
    let b = [corner[0] + ut[0] * EDGE, corner[1] + ut[1] * EDGE];
    let far = [a[0] + ut[0] * EDGE, a[1] + ut[1] * EDGE];
    let mut pts = vec![a, far, b];
    // the nose: the round between the edges' tangent points, the side facing away from the insert
    let t_lead = [corner[0] + ul[0] * rn / half.tan(), corner[1] + ul[1] * rn / half.tan()];
    let a0 = (t_lead[1] - c[1]).atan2(t_lead[0] - c[0]);
    let sweep = std::f64::consts::PI - 2.0 * half;
    for k in 0..=24 {
        // from the leading edge's tangent point round the outside to the trailing edge's
        let ang = a0 + sweep * k as f64 / 24.0;
        pts.push([c[0] + rn * ang.cos(), c[1] + rn * ang.sin()]);
    }
    pts
}

/// The convex hull of `pts`, anticlockwise.
fn hull(mut pts: Vec<[f64; 2]>) -> Ring {
    pts.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    let mut lower: Vec<[f64; 2]> = Vec::new();
    for p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], *p) <= 0.0 {
            lower.pop();
        }
        lower.push(*p);
    }
    let mut upper: Vec<[f64; 2]> = Vec::new();
    for p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], *p) <= 0.0 {
            upper.pop();
        }
        upper.push(*p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// The nose centre of the tip at `p` ([x radius, z]) as [z, r].
fn centre(t: &LatheTool, p: [f64; 2]) -> [f64; 2] {
    [p[1] + t.nose_radius, p[0] + t.nose_radius]
}

fn pieces(a: &Area, min_area: f64, min_thickness: f64) -> Vec<Piece> {
    let mut out: Vec<Piece> = a
        .shapes
        .iter()
        .filter_map(|sh| {
            let one = Area { shapes: vec![sh.clone()] };
            let area = one.area();
            let (lo, hi) = one.bounds()?;
            // and thicker on average than the paths' round-off along it
            let thick = area / (hi[0] - lo[0]).max(1e-9) > min_thickness;
            (area > min_area && thick).then_some(Piece { z: [lo[0], hi[0]], diameter: [2.0 * lo[1], 2.0 * hi[1]], area })
        })
        .collect();
    out.sort_by(|a, b| b.area.total_cmp(&a.area));
    out.truncate(50);
    out
}

/// The part's half section in work coordinates, extended back over the parting allowance.
pub fn part_area(plan: &LathePlan) -> Result<Area> {
    let p = &plan.profile;
    let l = p.length;
    let back = plan.setup.turned_to;
    let first = p.od.first().copied().unwrap_or(0.0);
    let mut ring: Ring = vec![[back, 0.0], [back, first]];
    let pts: Vec<[f64; 2]> = p.od.iter().enumerate().map(|(i, r)| [i as f64 * p.dz - l, *r]).collect();
    ring.extend(super::plan::simplify(&pts, 0.0005));
    ring.push([0.0, 0.0]);
    Area::union_of(&[ring])
}

/// Simulate `plan`.
pub fn simulate(job: &LatheJob, plan: &LathePlan) -> Result<(LatheSimReport, Area)> {
    let su = &plan.setup;
    let mut left = Area::rect(su.jaws, 0.0, su.front, su.radius)?;
    let start = left.area();
    let mut rapid_hits = Vec::new();
    let mut jaw_hits = 0usize;
    for op in &plan.ops {
        let t = job.tool(op.tool).ok_or_else(|| CamError::Invalid(format!("simulation: no tool T{}", op.tool)))?;
        let mut batch: Vec<Ring> = Vec::new();
        let mut at: Option<[f64; 2]> = None;
        for m in &op.moves {
            let to = m.to();
            let from = at.unwrap_or(to);
            let mut pts = insert(t, centre(t, from));
            pts.extend(insert(t, centre(t, to)));
            let swept = hull(pts);
            if swept.iter().any(|q| q[0] < su.jaws) {
                jaw_hits += 1;
            }
            match m {
                TipMove::Feed(_) => batch.push(swept),
                TipMove::Rapid(_) => {
                    if !batch.is_empty() {
                        left = left.difference(&Area::union_of(&batch)?)?;
                        batch.clear();
                    }
                    let hit = left.intersection(&Area::union_of(&[swept])?)?.area();
                    if hit > 1e-3 && rapid_hits.len() < 50 {
                        rapid_hits.push(RapidHit { operation: op.name.clone(), at: [2.0 * to[0], to[1]], area: hit });
                    }
                }
            }
            at = Some(to);
        }
        if !batch.is_empty() {
            left = left.difference(&Area::union_of(&batch)?)?;
        }
    }
    let part = part_area(plan)?;
    let window = Area::rect(su.turned_to, 0.0, su.front + 1.0, su.radius + 1.0)?;
    let extra = left.intersection(&window)?.difference(&part)?;
    let cut = part.difference(&left)?;
    // the paths are chords within PATH_TOL of the nose's path, the part's outline within the
    // mesh's tolerance of it: a cut thinner than both on average along it is no gouge
    let gouges = pieces(&cut, 1e-3, super::plan::PATH_TOL + super::profile::MESH_TOL);
    let report = LatheSimReport { removed_area: start - left.area(), gouges, left: pieces(&extra, AREA_TOL, MIN_THICKNESS), rapid_hits, jaw_hits };
    Ok((report, left))
}

/// A drawing of the half section: the bar as left (grey), the part's outline (blue), the
/// feeds (orange) and rapids (dashed), the jaws' face (red); `scale` pixels per mm.
pub fn svg(plan: &LathePlan, left: &Area, scale: f64) -> Result<String> {
    use std::fmt::Write as _;
    let su = &plan.setup;
    let (z0, z1) = (su.jaws - 2.0, su.front + 6.0);
    let (r0, r1) = (-1.0, su.radius + 6.0);
    let w = (z1 - z0) * scale;
    let h = (r1 - r0) * scale;
    let px = |p: [f64; 2]| ((p[0] - z0) * scale, (r1 - p[1]) * scale);
    let path = |ring: &Ring| {
        let mut s = String::new();
        for (i, q) in ring.iter().enumerate() {
            let (x, y) = px(*q);
            s.push_str(&format!("{}{x:.2},{y:.2} ", if i == 0 { "M" } else { "L" }));
        }
        s.push('Z');
        s
    };
    let mut o = String::new();
    let e = |e: std::fmt::Error| CamError::Invalid(e.to_string());
    writeln!(o, r#"<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0}" height="{h:.0}" viewBox="0 0 {w:.1} {h:.1}">"#).map_err(e)?;
    writeln!(o, r#"<rect width="100%" height="100%" fill="white"/>"#).map_err(e)?;
    let d: String = left.rings().map(path).collect::<Vec<_>>().join(" ");
    writeln!(o, r##"<path d="{d}" fill="#c8ccd2" fill-rule="evenodd" stroke="#7a808a" stroke-width="0.5"/>"##).map_err(e)?;
    let part = part_area(plan)?;
    let d: String = part.rings().map(path).collect::<Vec<_>>().join(" ");
    writeln!(o, r##"<path d="{d}" fill="none" stroke="#2f6fd6" stroke-width="1"/>"##).map_err(e)?;
    let colours = ["#d97706", "#c2410c", "#16a34a"];
    for (i, op) in plan.ops.iter().enumerate() {
        let colour = colours.get(i % colours.len()).copied().unwrap_or("#d97706");
        let mut at: Option<(f64, f64)> = None;
        for m in &op.moves {
            let p = m.to();
            let q = px([p[1], p[0]]);
            if let Some((x, y)) = at {
                let dash = if matches!(m, TipMove::Rapid(_)) { r#" stroke-dasharray="3,3" opacity="0.5""# } else { "" };
                writeln!(o, r#"<line x1="{x:.2}" y1="{y:.2}" x2="{:.2}" y2="{:.2}" stroke="{colour}" stroke-width="0.6"{dash}/>"#, q.0, q.1)
                    .map_err(e)?;
            }
            at = Some(q);
        }
    }
    let (jx, _) = px([su.jaws, 0.0]);
    writeln!(o, r##"<line x1="{jx:.2}" y1="0" x2="{jx:.2}" y2="{h:.1}" stroke="#dc2626" stroke-width="1.5"/>"##).map_err(e)?;
    let (ax0, ay) = px([z0, 0.0]);
    let (ax1, _) = px([z1, 0.0]);
    writeln!(o, r##"<line x1="{ax0:.2}" y1="{ay:.2}" x2="{ax1:.2}" y2="{ay:.2}" stroke="#555" stroke-dasharray="8,3,2,3" stroke-width="0.6"/>"##)
        .map_err(e)?;
    writeln!(o, "</svg>").map_err(e)?;
    Ok(o)
}
