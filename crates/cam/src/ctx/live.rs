//! The live tools' work on a turn-mill part, read from its exact faces in each side's work frame
//! (Z0 the face that side finishes, the part at negative Z):
//!
//! - **axial holes** off the axis (upright cylinders, as the router reads them): drilled with a
//!   live axial drill of their size at their C angle, from the side they open to (side 1 when
//!   both);
//! - **radial holes** (cylinders square to the axis, their axis through it): drilled with a live
//!   radial drill at their C angle and Z; a through hole from both sides, each past the centre;
//! - **keyways**: a flat floor square to a radius with the slot's width along the axis, as wide as
//!   a live radial end mill: milled at a fixed C, in levels down to the floor;
//! - **face milling**: material standing above an upward floor (from the side's face) within the
//!   turned envelope, cleared level by level with a live axial end mill in polar interpolation:
//!   rings round the area the cutter's centre may go, kept off the part's material above the level.
//!
//! In polar interpolation C turns at feed / radius: each move's feed is held under C's speed for
//! polar work (`machine.polar_c_rpm`) at its nearest point to the centre, and a move that would pass within [`R_MIN`] of it is cut
//! there: the tool lifts, goes round the centre in the air, and comes back down.

use solidcore_record::Vec3;
use solidcraft_kernel::{Body, FaceSurface, face_surface};

use super::job::{CtxJob, LiveTool, Mount};
use crate::job::ToolKind;
use crate::lathe::job::Front;
use crate::lathe::profile::Profile;
use crate::part::PartFeatures;
use crate::region::{Area, Ring};
use crate::{CamError, Result};

/// Closest a polar move comes to the centre, mm.
pub const R_MIN: f64 = 1.0;
/// Radius the tool goes round the centre at, in the air, mm.
const R_ROUND: f64 = 3.0;
/// Above the face, mm: where live tools move between cuts.
const CLEAR: f64 = 2.0;
/// Chord tolerance of the milled areas, mm.
const AREA_TOL: f64 = 0.002;
/// A tool fits a slot or hole within this of its width, mm.
const FIT: f64 = 0.02;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LiveKind {
    AxialDrill,
    RadialDrill,
    Keyway,
    FaceMill,
}

/// A hole to drill: axial, `r` the centre's radius, `z` the hole's top and `to` its bottom (Z);
/// radial, `r` the surface's radius where it starts, `z` its Z and `to` the bottom (a radius).
#[derive(Clone, Copy, Debug)]
pub struct DrillAt {
    pub c: f64,
    pub r: f64,
    pub z: f64,
    pub to: f64,
}

/// A live tool's move: in polar interpolation `[x, y, z]` on the face (radius); a keyway's
/// `[x (radius), 0, z]` at its C.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LiveMove {
    Rapid([f64; 3]),
    Feed([f64; 3], f64),
}

impl LiveMove {
    pub fn to(&self) -> [f64; 3] {
        match self {
            LiveMove::Rapid(p) | LiveMove::Feed(p, _) => *p,
        }
    }
}

#[derive(Clone, Debug)]
pub struct LiveOp {
    pub kind: LiveKind,
    pub name: String,
    pub tool: u32,
    pub rpm: f64,
    pub feed: f64,
    pub plunge: f64,
    pub peck: Option<f64>,
    pub holes: Vec<DrillAt>,
    /// A keyway's C.
    pub c: f64,
    pub moves: Vec<LiveMove>,
}

/// The part in side `side`'s work frame: its face at Z0, the part toward −Z.
pub fn side_body(body: &Body, front: Front, side: u8) -> Result<Body> {
    let b = body.exact_bounds()?;
    let (lo, hi) = (b.min.z, b.max.z);
    let up = matches!((front, side), (Front::PlusZ, 1) | (Front::MinusZ, 2));
    let zero = Vec3::new(0.0, 0.0, 0.0);
    let moved = if up {
        solidcraft_kernel::transform(body, Vec3::new(0.0, 0.0, -hi), zero, Vec3::new(0.0, 0.0, 1.0), 0.0)?
    } else {
        // end for end: half a turn about Y
        solidcraft_kernel::transform(body, Vec3::new(0.0, 0.0, lo), zero, Vec3::new(0.0, 1.0, 0.0), std::f64::consts::PI)?
    };
    Ok(moved)
}

/// The envelope's outer radius at work Z `z` (the side's profile).
fn od_at(p: &Profile, z: f64) -> f64 {
    p.od_at(z + p.length)
}

/// The largest outer radius of the envelope over work Z `a..b`.
fn od_over(p: &Profile, a: f64, b: f64) -> f64 {
    let (i, j) = (p.index(a.min(b) + p.length), p.index(a.max(b) + p.length));
    (i..=j).filter_map(|k| p.od.get(k)).copied().fold(0.0, f64::max)
}

/// The narrowest the envelope's bore is over work Z `a..b` (0: solid on the axis somewhere there).
fn id_over(p: &Profile, a: f64, b: f64) -> f64 {
    let (i, j) = (p.index(a.min(b) + p.length), p.index(a.max(b) + p.length));
    (i..=j).filter_map(|k| p.id.get(k)).copied().fold(f64::INFINITY, f64::min).max(0.0)
}

fn deg(a: f64) -> f64 {
    let d = a.to_degrees();
    let d = if d < 0.0 { d + 360.0 } else { d };
    if d >= 360.0 - 1e-9 { 0.0 } else { d }
}

/// The live tool of `kind` and `mount` whose diameter is `d` (within [`FIT`]).
fn fitting(job: &CtxJob, kind: ToolKind, mount: Mount, d: f64) -> Option<&LiveTool> {
    job.live_tools.iter().filter(|t| t.kind == kind && t.mount == mount && (t.diameter - d).abs() <= FIT).min_by_key(|t| t.number)
}

fn rpm(job: &CtxJob, t: &LiveTool) -> f64 {
    t.rpm.min(job.machine.live_max_rpm)
}

/// Axial holes off the axis that open to this side's face: `taken` are centres (in this side's
/// frame) another side drills.
pub fn axial_holes(job: &CtxJob, f: &PartFeatures, taken: &[[f64; 2]]) -> (Vec<LiveOp>, Vec<[f64; 2]>, Vec<String>) {
    let mut warnings = Vec::new();
    let mut ops: Vec<LiveOp> = Vec::new();
    let mut done = Vec::new();
    for h in &f.holes {
        let r = h.center[0].hypot(h.center[1]);
        if r < 1e-3 || taken.iter().any(|c| (c[0] - h.center[0]).hypot(c[1] - h.center[1]) < 1e-3) {
            continue;
        }
        // open to this face: no floor of the part above its top over the drill's circle
        let disk = match Area::circle(h.center[0], h.center[1], h.diameter / 2.0, AREA_TOL) {
            Ok(a) => a,
            Err(_) => continue,
        };
        let covered = f.floors.iter().any(|fl| fl.z > h.z_top + 1e-3 && fl.region.intersection(&disk).is_ok_and(|a| a.area() > 1e-4));
        if covered {
            continue;
        }
        let Some(t) = fitting(job, ToolKind::Drill, Mount::Axial, h.diameter) else {
            warnings.push(format!(
                "the Ø{:.2} hole at X{:.3} Y{:.3} is not drilled: no live axial drill of that size",
                h.diameter, h.center[0], h.center[1]
            ));
            continue;
        };
        let tip = t.diameter / 2.0 / (t.tip_angle_deg.unwrap_or(118.0).to_radians() / 2.0).tan();
        let to = if h.through { h.z_bottom - tip - 0.5 } else { h.z_bottom };
        if !h.through {
            warnings.push(format!(
                "the blind Ø{:.2} hole at X{:.3} Y{:.3} is left with T{}'s point at its floor",
                h.diameter, h.center[0], h.center[1], t.number
            ));
        }
        if h.z_top - to > t.flute_length {
            warnings.push(format!(
                "the Ø{:.2} hole at X{:.3} Y{:.3} is deeper than T{}'s flutes: drilled to their length",
                h.diameter, h.center[0], h.center[1], t.number
            ));
        }
        let to = to.max(h.z_top - t.flute_length);
        let at = DrillAt { c: deg(h.center[1].atan2(h.center[0])), r, z: h.z_top, to };
        done.push(h.center);
        match ops.iter_mut().find(|o| o.tool == t.number) {
            Some(o) => o.holes.push(at),
            None => ops.push(LiveOp {
                kind: LiveKind::AxialDrill,
                name: format!("axial holes Ø{}", crate::post::num(t.diameter, 3)),
                tool: t.number,
                rpm: rpm(job, t),
                feed: t.feed,
                plunge: t.plunge,
                peck: t.peck,
                holes: vec![at],
                c: 0.0,
                moves: Vec::new(),
            }),
        }
    }
    for o in &mut ops {
        o.holes.sort_by(|a, b| a.c.total_cmp(&b.c));
    }
    (ops, done, warnings)
}

/// Points along a face's edges (its corners and between), in the body's frame.
fn face_points(body: &Body, f: usize) -> Vec<[f64; 3]> {
    use solidcore_record::brep::Loop;
    use solidcraft_geom::brep::eval::CurveEval;
    let rec = body.record();
    let mut out = Vec::new();
    for lp in rec.faces.get(f).map(|x| x.loops.as_slice()).unwrap_or_default() {
        match lp {
            Loop::Edges { edges } => {
                for oe in edges {
                    let Some(e) = rec.edges.get(oe.edge as usize) else { continue };
                    out.extend(e.vertices.iter().filter_map(|v| rec.vertices.get(*v as usize)).map(|v| v.point));
                    if let Ok(c) = CurveEval::new(e.parameters()) {
                        let [t0, t1] = e.interval;
                        out.extend((1..16).filter_map(|i| c.point(t0 + (t1 - t0) * i as f64 / 16.0)).map(|p| [p.x, p.y, p.z]));
                    }
                }
            }
            Loop::Vertex { vertex } => out.extend(rec.vertices.get(*vertex as usize).map(|v| v.point)),
        }
    }
    out
}

/// Radial holes and keyways, read from the faces of `body` (side 1's frame) with `p` its
/// envelope.
pub fn radial(job: &CtxJob, body: &Body, p: &Profile) -> (Vec<LiveOp>, Vec<String>) {
    let mut warnings = Vec::new();
    let mut ops: Vec<LiveOp> = Vec::new();
    // radial cylinders: (direction angle, z, radius, t-range along the direction)
    let mut holes: Vec<(f64, f64, f64, [f64; 2], [f64; 2])> = Vec::new();
    let mut floors: Vec<(usize, [f64; 2], f64, [f64; 2], [f64; 2])> = Vec::new(); // face, n, d, t, z
    let mut ends: Vec<([f64; 2], f64, f64)> = Vec::new(); // axis dir, radius, z of a cylinder along a radius off the axis
    for f in 0..body.face_count() {
        match face_surface(body, f) {
            Some(FaceSurface::Cylinder { origin, axis, radius }) => {
                let a = axis.vec();
                if a.z.abs() > 1e-6 {
                    continue;
                }
                let n = [a.x, a.y];
                let off = origin.x * -n[1] + origin.y * n[0];
                let pts = face_points(body, f);
                // how far round its axis the face goes: a hole all the way, a slot's end half
                let (u, v) = ([0.0, 0.0, 1.0], [-n[1], n[0], 0.0]);
                let mut angs: Vec<f64> = pts
                    .iter()
                    .map(|q| {
                        let d = [q[0] - origin.x, q[1] - origin.y, q[2] - origin.z];
                        (d[0] * v[0] + d[1] * v[1] + d[2] * v[2]).atan2(d[0] * u[0] + d[1] * u[1] + d[2] * u[2])
                    })
                    .collect();
                angs.sort_by(f64::total_cmp);
                let gap = angs
                    .windows(2)
                    .map(|w| w[1] - w[0])
                    .fold(0.0, f64::max)
                    .max(angs.first().zip(angs.last()).map_or(std::f64::consts::TAU, |(a, b)| a + std::f64::consts::TAU - b));
                let partial = gap > 0.6;
                if off.abs() < 1e-6 && !partial {
                    let ts: Vec<f64> = pts.iter().map(|q| q[0] * n[0] + q[1] * n[1]).collect();
                    let t = [ts.iter().copied().fold(f64::INFINITY, f64::min), ts.iter().copied().fold(f64::NEG_INFINITY, f64::max)];
                    let zs = [origin.z, origin.z];
                    // one direction for both halves of a split cylinder
                    let (ang, t) = if n[0] < -1e-9 || (n[0].abs() <= 1e-9 && n[1] < 0.0) {
                        ((-n[1]).atan2(-n[0]), [-t[1], -t[0]])
                    } else {
                        (n[1].atan2(n[0]), t)
                    };
                    holes.push((ang, origin.z, radius, t, zs));
                } else {
                    ends.push((n, radius, origin.z));
                }
            }
            Some(FaceSurface::Plane { normal, point }) => {
                let nv = normal.vec();
                if nv.z.abs() > 1e-6 {
                    continue;
                }
                let n = [nv.x, nv.y];
                let d = point.x * n[0] + point.y * n[1];
                if d <= 1e-6 {
                    continue;
                }
                let pts = face_points(body, f);
                let ts: Vec<f64> = pts.iter().map(|q| -q[0] * n[1] + q[1] * n[0]).collect();
                let zs: Vec<f64> = pts.iter().map(|q| q[2]).collect();
                let t = [ts.iter().copied().fold(f64::INFINITY, f64::min), ts.iter().copied().fold(f64::NEG_INFINITY, f64::max)];
                let z = [zs.iter().copied().fold(f64::INFINITY, f64::min), zs.iter().copied().fold(f64::NEG_INFINITY, f64::max)];
                floors.push((f, n, d, t, z));
            }
            _ => {}
        }
    }
    // radial holes: one per direction, z and radius (a split cylinder's faces joined)
    let mut groups: Vec<(f64, f64, f64, [f64; 2])> = Vec::new();
    for (ang, z, r, t, _) in holes {
        match groups.iter_mut().find(|g| (g.0 - ang).abs() < 1e-6 && (g.1 - z).abs() < 1e-6 && (g.2 - r).abs() < 1e-6) {
            Some(g) => g.3 = [g.3[0].min(t[0]), g.3[1].max(t[1])],
            None => groups.push((ang, z, r, t)),
        }
    }
    let x_min = job.machine.x_min;
    for (ang, z, r, t) in groups {
        let d = 2.0 * r;
        let Some(tool) = fitting(job, ToolKind::Drill, Mount::Radial, d) else {
            warnings.push(format!("the Ø{d:.2} cross hole at Z{z:.3} is not drilled: no live radial drill of that size"));
            continue;
        };
        let tip = r / (tool.tip_angle_deg.unwrap_or(118.0).to_radians() / 2.0).tan();
        let surface = od_at(p, z);
        let through = t[0] < -r && t[1] > r;
        // from each open side: to just past the centre (as far as X goes) for a through hole
        let mut sides: Vec<(f64, f64)> = Vec::new();
        if through {
            let to = (-tip - 0.3).max(x_min);
            if to > -tip + 1e-9 {
                warnings.push(format!(
                    "the Ø{d:.2} cross hole at Z{z:.3}: X goes to {x_min} only: the drill's full width doesn't reach the centre from either side ({:.2} short)",
                    to + tip
                ));
            }
            sides.push((ang, to));
            sides.push((ang + std::f64::consts::PI, to));
        } else if t[1] > r {
            sides.push((ang, t[0].max(0.0)));
        } else {
            sides.push((ang + std::f64::consts::PI, (-t[1]).max(0.0)));
        }
        let holes: Vec<DrillAt> = sides.into_iter().map(|(a, to)| DrillAt { c: deg(a), r: surface, z, to }).collect();
        if surface - holes.iter().map(|h| h.to).fold(f64::INFINITY, f64::min) > tool.flute_length {
            warnings.push(format!("the Ø{d:.2} cross hole at Z{z:.3} is deeper than T{}'s flutes", tool.number));
        }
        ops.push(LiveOp {
            kind: LiveKind::RadialDrill,
            name: format!("cross hole Ø{} at Z{}", crate::post::num(d, 3), crate::post::num(z, 3)),
            tool: tool.number,
            rpm: rpm(job, tool),
            feed: tool.feed,
            plunge: tool.plunge,
            peck: tool.peck,
            holes,
            c: 0.0,
            moves: Vec::new(),
        });
    }
    // keyways: a floor square to a radius, as wide as a radial end mill, centred on its C, along Z
    for (face, n, dist, t, z) in floors {
        let w = t[1] - t[0];
        let centre = (t[0] + t[1]) / 2.0;
        let Some(tool) = fitting(job, ToolKind::Flat, Mount::Radial, w) else { continue };
        if centre.abs() > 1e-3 {
            warnings.push(format!("face {face}: a slot {w:.2} wide off its centre line: not milled (a radial tool cuts on a line through the axis)"));
            continue;
        }
        let rt = tool.diameter / 2.0;
        // its ends: half round (the cutter's) or square (left round)
        let round = ends.iter().filter(|e| (e.0[0] - n[0]).abs() + (e.0[1] - n[1]).abs() < 1e-6 && (e.1 - w / 2.0).abs() < 1e-3).count() >= 2;
        if !round {
            warnings.push(format!("the keyway at Z{:.3} to {:.3} has square ends: they are left round (R{})", z[0], z[1], crate::post::num(rt, 3)));
        }
        let (za, zb) = (z[0] + rt, z[1] - rt);
        if zb < za - 1e-9 {
            continue;
        }
        let top = od_over(p, z[0], z[1]);
        let step = tool.stepdown.unwrap_or(tool.diameter).min(tool.flute_length);
        let n_lv = ((top - dist) / step - 1e-6).ceil().max(1.0) as usize;
        if top - dist > tool.flute_length {
            warnings.push(format!("the keyway at Z{:.3}: deeper than T{}'s flutes", z[0], tool.number));
        }
        let mut mv = vec![LiveMove::Rapid([top + CLEAR, 0.0, zb])];
        let mut at_b = true;
        for k in 1..=n_lv {
            let x = top - (top - dist) * k as f64 / n_lv as f64;
            let (here, there) = if at_b { (zb, za) } else { (za, zb) };
            mv.push(LiveMove::Feed([x, 0.0, here], tool.plunge));
            mv.push(LiveMove::Feed([x, 0.0, there], tool.feed));
            at_b = !at_b;
        }
        let last = mv.last().map(LiveMove::to).unwrap_or([top, 0.0, zb]);
        mv.push(LiveMove::Rapid([top + CLEAR, 0.0, last[2]]));
        ops.push(LiveOp {
            kind: LiveKind::Keyway,
            name: format!("keyway {} wide at Z{} to {}", crate::post::num(w, 3), crate::post::num(z[0], 3), crate::post::num(z[1], 3)),
            tool: tool.number,
            rpm: rpm(job, tool),
            feed: tool.feed,
            plunge: tool.plunge,
            peck: None,
            holes: Vec::new(),
            c: deg(n[1].atan2(n[0])),
            moves: mv,
        });
    }
    (ops, warnings)
}

/// The closest point of segment `a`–`b` to the origin: (its parameter, its distance).
fn closest(a: [f64; 2], b: [f64; 2]) -> (f64, f64) {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 { (-(a[0] * dx + a[1] * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
    (t, (a[0] + t * dx).hypot(a[1] + t * dy))
}

/// The feed for a polar move from `a` to `b`: `f`, held so C turns no faster than `c_max`
/// (degrees a second) at its nearest point to the centre (a move along Z alone turns no C).
fn polar_feed(a: [f64; 2], b: [f64; 2], f: f64, c_max: f64) -> f64 {
    if (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9 {
        return f;
    }
    let (_, r) = closest(a, b);
    f.min(c_max.to_radians() * r.max(1e-3) * 60.0)
}

/// Where segment `a`–`b` enters and leaves the circle of radius `r` about the origin (parameters).
fn through_circle(a: [f64; 2], b: [f64; 2], r: f64) -> Option<(f64, f64)> {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let qa = dx * dx + dy * dy;
    let qb = 2.0 * (a[0] * dx + a[1] * dy);
    let qc = a[0] * a[0] + a[1] * a[1] - r * r;
    let disc = qb * qb - 4.0 * qa * qc;
    if qa <= 0.0 || disc <= 0.0 {
        return None;
    }
    let s = disc.sqrt();
    Some(((-qb - s) / (2.0 * qa), (-qb + s) / (2.0 * qa)))
}

/// The fastest a polar move goes, mm/min (all of them are feeds: a rapid near the centre would
/// turn C too fast).
const POLAR_RAPID: f64 = 3000.0;

/// Polar moves from a position: each a feed C can follow, none through the centre's circle.
struct Polar<'a> {
    out: &'a mut Vec<LiveMove>,
    at: [f64; 3],
    clear: f64,
    c_max: f64,
}

impl Polar<'_> {
    fn push(&mut self, to: [f64; 3], f: f64) {
        let f = polar_feed([self.at[0], self.at[1]], [to[0], to[1]], f, self.c_max);
        self.out.push(LiveMove::Feed(to, f));
        self.at = to;
    }

    /// Straight to `to` at feed `f`; where the line passes within [`R_MIN`] of the centre, up to
    /// the clearance there, round the centre at [`R_ROUND`] and back down.
    fn line(&mut self, to: [f64; 3], f: f64) {
        let (a, b) = ([self.at[0], self.at[1]], [to[0], to[1]]);
        let (_, near) = closest(a, b);
        // cutting, as close as R_MIN; in the air, round at R_ROUND straight away
        let cutting = self.at[2] < self.clear - 1e-9 || to[2] < self.clear - 1e-9;
        let keep = if cutting { R_MIN } else { R_ROUND };
        let crossing = if near < keep - 1e-9 { through_circle(a, b, keep) } else { None };
        let Some((t0, t1)) = crossing else {
            self.push(to, f);
            return;
        };
        let lerp = |t: f64| {
            let t = t.clamp(0.0, 1.0);
            [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, self.at[2] + (to[2] - self.at[2]) * t]
        };
        let (p0, p1) = (lerp(t0), lerp(t1));
        self.push(p0, f);
        let z = self.at[2];
        if z < self.clear - 1e-9 {
            self.push([p0[0], p0[1], self.clear], POLAR_RAPID);
        }
        // round the centre, the short way
        let (a0, a1) = (p0[1].atan2(p0[0]), p1[1].atan2(p1[0]));
        let mut sweep = a1 - a0;
        while sweep > std::f64::consts::PI {
            sweep -= std::f64::consts::TAU;
        }
        while sweep < -std::f64::consts::PI {
            sweep += std::f64::consts::TAU;
        }
        let steps = ((sweep.abs() / 0.2).ceil() as usize).max(1);
        for k in 1..=steps {
            let ang = a0 + sweep * k as f64 / steps as f64;
            self.push([R_ROUND * ang.cos(), R_ROUND * ang.sin(), self.clear], POLAR_RAPID);
        }
        self.push([p1[0], p1[1], self.clear], POLAR_RAPID);
        if z < self.clear - 1e-9 {
            self.push([p1[0], p1[1], p1[2].min(z.max(p1[2]))], f);
        }
        self.push(to, f);
    }

    /// A path of feeds at height `z` (closed rings and open lines alike): over its start at the
    /// clearance, down at `plunge`, along it at `feed`, back up.
    fn path(&mut self, pts: &[[f64; 2]], z: f64, feed: f64, plunge: f64) {
        let Some(&first) = pts.first() else { return };
        if self.at[2] < self.clear - 1e-9 {
            self.push([self.at[0], self.at[1], self.clear], POLAR_RAPID);
        }
        self.line([first[0], first[1], self.clear], POLAR_RAPID);
        self.push([first[0], first[1], z], plunge);
        for &b in pts.iter().skip(1) {
            self.line([b[0], b[1], z], feed);
        }
        self.push([self.at[0], self.at[1], self.clear], POLAR_RAPID);
    }
}

/// A closed ring as a path starting at its point farthest from the centre (most likely in the
/// air), round and back to it.
fn ring_path(r: &Ring) -> Vec<[f64; 2]> {
    let start = r.iter().enumerate().max_by(|a, b| a.1[0].hypot(a.1[1]).total_cmp(&b.1[0].hypot(b.1[1]))).map_or(0, |(i, _)| i);
    let mut v: Vec<[f64; 2]> = r.iter().cycle().skip(start).take(r.len()).copied().collect();
    if let Some(&f) = v.first() {
        v.push(f);
    }
    v
}

/// Face milling on one side: the material over each upward floor within the envelope, `f` read
/// in the side's frame and `p` its envelope.
pub fn face_milling(job: &CtxJob, f: &PartFeatures, p: &Profile, side: u8) -> Result<(Vec<LiveOp>, Vec<String>)> {
    let mut warnings = Vec::new();
    let mut ops = Vec::new();
    let tools: Vec<&LiveTool> = job.live_tools.iter().filter(|t| t.kind == ToolKind::Flat && t.mount == Mount::Axial).collect();
    let floors: Vec<&crate::part::Floor> = f.floors.iter().filter(|fl| !fl.top && !fl.in_hole).collect();
    let mut work: Vec<(f64, Area, f64)> = Vec::new(); // floor z, region, from z
    for fl in floors {
        // the envelope stands over it from where it first reaches the floor's region
        let mut from = None;
        let mut z = 0.0;
        let step = 0.05;
        while z > fl.z + 1e-9 {
            let disk = Area::circle(0.0, 0.0, od_at(p, z - step / 2.0).max(1e-3), AREA_TOL)?;
            if fl.region.intersection(&disk)?.area() > 1e-4 {
                from = Some(z);
                break;
            }
            z -= step;
        }
        if let Some(from) = from {
            work.push((fl.z, fl.region.clone(), from));
        }
    }
    if work.is_empty() {
        return Ok((ops, warnings));
    }
    // the tool: the largest axial end mill whose radius fits the narrowest gap (tried largest
    // first: the first leaving no material it should cut)
    let Some(tool) = tools.iter().max_by(|a, b| a.diameter.total_cmp(&b.diameter)).copied() else {
        warnings.push(format!("side {side}: faces to mill but no live axial end mill"));
        return Ok((ops, warnings));
    };
    let rt = tool.diameter / 2.0;
    let step = tool.stepdown.unwrap_or(tool.diameter).min(tool.flute_length);
    let over = tool.stepover.unwrap_or(0.4) * tool.diameter;
    let c_max = job.machine.polar_c_rpm * 6.0;
    let mut mv: Vec<LiveMove> = Vec::new();
    // in from the bar's radius plus the clearance, on X
    let start = [job.stock.diameter / 2.0 + CLEAR + rt, 0.0, CLEAR];
    // the first move: where polar interpolation comes on (off the centre, on X)
    mv.push(LiveMove::Rapid(start));
    let mut polar = Polar { out: &mut mv, at: start, clear: CLEAR, c_max };
    for (zf, region, from) in &work {
        let n = ((from - zf) / step - 1e-6).ceil().max(1.0) as usize;
        if from - zf > tool.flute_length {
            warnings.push(format!("side {side}: the floor at Z{zf:.3} is {:.2} deep: deeper than T{}'s flutes", from - zf, tool.number));
        }
        let mut prev = *from;
        for k in 1..=n {
            let z = from - (from - zf) * k as f64 / n as f64;
            let od = od_over(p, z, prev);
            let prev_z = prev;
            prev = z;
            let disk = Area::circle(0.0, 0.0, od.max(1e-3), AREA_TOL)?;
            let remove = region.intersection(&disk)?;
            if remove.area() < 1e-4 {
                continue;
            }
            // the part's material above this level: every floor above it, the top face's too
            let mut keep = Area::empty();
            for fl in f.floors.iter().filter(|fl| fl.z > z + 1e-6) {
                keep = keep.union(&fl.region)?;
            }
            let allowed = Area::circle(0.0, 0.0, od + rt + 5.0, AREA_TOL)?.difference(&keep.offset(rt - 0.01, AREA_TOL)?)?;
            // the open bore beside it is air the cutter may pass over: a pocket round the bore is cut
            // as one, its rings not folding back at the centre (where C would reverse at once)
            let bore = id_over(p, z, prev_z);
            let opens = bore > 1e-3 && remove.intersection(&Area::circle(0.0, 0.0, bore + 0.05, AREA_TOL)?)?.area() > 1e-4;
            let zone = if opens { remove.union(&Area::circle(0.0, 0.0, bore, AREA_TOL)?)? } else { remove.clone() };
            let centre = allowed.intersection(&zone.offset(rt - 0.01, AREA_TOL)?)?;
            if centre.is_empty() {
                warnings.push(format!("side {side}: Z{z:.3}: T{} doesn't fit where it should cut", tool.number));
                continue;
            }
            // what the cutter leaves of it: reported
            let swept = centre.offset(rt, AREA_TOL)?;
            let missed = remove.difference(&swept)?;
            if missed.area() > 0.01 {
                warnings.push(format!("side {side}: Z{z:.3}: {:.3} mm² over the floor that T{} can't reach is left", missed.area(), tool.number));
            }
            // rings from the outside in
            let mut a = centre;
            let mut guard = 0;
            while !a.is_empty() && guard < 500 {
                guard += 1;
                for r in a.rings() {
                    polar.path(&ring_path(r), z, tool.feed, tool.plunge);
                }
                a = a.offset(-over, AREA_TOL)?;
            }
        }
    }
    // back out to where it came in
    polar.line(start, POLAR_RAPID);
    if mv.len() > 1 {
        ops.push(LiveOp {
            kind: LiveKind::FaceMill,
            name: format!("face milling side {side}"),
            tool: tool.number,
            rpm: rpm(job, tool),
            feed: tool.feed,
            plunge: tool.plunge,
            peck: None,
            holes: Vec::new(),
            c: 0.0,
            moves: mv,
        });
    }
    Ok((ops, warnings))
}

/// All the live tools' work on `side`.
pub fn plan_side(job: &CtxJob, body: &Body, p: &Profile, side: u8, taken: &[[f64; 2]]) -> Result<(Vec<LiveOp>, Vec<[f64; 2]>, Vec<String>)> {
    let b = side_body(body, job.part.front, side)?;
    let f = crate::part::read_part(&b).map_err(|e| CamError::Invalid(format!("side {side}: {e}")))?;
    let (mut ops, done, mut warnings) = axial_holes(job, &f, taken);
    if side == 1 {
        let (r, w) = radial(job, &b, p);
        ops.extend(r);
        warnings.extend(w);
    }
    let (m, w) = face_milling(job, &f, p, side)?;
    // milling first: holes drill through milled faces cleanly
    let mut all = m;
    all.append(&mut ops);
    warnings.extend(w);
    Ok((all, done, warnings))
}

/// Roughly how long `op` takes, s: its feeds at their rates, each hole drilled twice over (the
/// pecks' returns) and indexed in a second, and 5 s for C, milling mode and the live tool.
pub fn estimate(op: &LiveOp) -> f64 {
    let mut t = 5.0;
    let mut at: Option<[f64; 3]> = None;
    for m in &op.moves {
        let to = m.to();
        let from = at.unwrap_or(to);
        let len = ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2) + (to[2] - from[2]).powi(2)).sqrt();
        let f = match m {
            LiveMove::Rapid(_) => 3000.0,
            LiveMove::Feed(_, f) => f.max(1.0),
        };
        t += len / f * 60.0;
        at = Some(to);
    }
    for h in &op.holes {
        let depth = match op.kind {
            LiveKind::RadialDrill => h.r - h.to,
            _ => h.z - h.to,
        };
        t += 1.0 + 2.0 * depth.abs() / op.plunge.max(1.0) * 60.0;
    }
    t
}
