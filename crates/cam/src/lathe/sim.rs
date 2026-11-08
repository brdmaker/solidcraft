//! Turning, simulated in the bar's half section (z along the axis, r out from it, work
//! coordinates): each move sweeps the tool's insert (a rhombus of its insert angle at its holder's
//! approach, its corner rounded to the nose radius) and takes what it covers from the bar. What
//! is left is compared with the part: material cut from it (gouges), material left on it, rapids
//! through material and the tools coming at the jaws.

use schemars::JsonSchema;
use serde::Serialize;

use super::job::{LatheJob, LatheTool, LatheToolKind};
use super::plan::{LatheOpKind, LathePlan, TipMove};
use crate::region::{Area, Ring};
use crate::{CamError, Result};

/// Length of the insert's edges in the simulation, mm.
const EDGE: f64 = 6.0;
/// Pieces smaller than this are ignored (a nose radius's fillet in a square inside corner is
/// r²(1 − π/4): 0.0086 mm² at R0.2), mm².
const AREA_TOL: f64 = 0.01;

/// Pieces thinner than this on average along the axis are round-off: the paths' tolerance, and
/// the profile's step along the axis (a square shoulder is held to the nearest sample, so a
/// strip up to a step thick may stand against it), mm.
const MIN_THICKNESS: f64 = super::plan::PATH_TOL + super::profile::STEP;

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
    /// Moves reaching behind the jaws' face (a tool's or its holder's).
    pub jaw_hits: usize,
    /// Where a tool's holder runs into the bar (what is left of it after the cuts of the same
    /// moves): a crash.
    pub holder_hits: Vec<HolderHit>,
}

/// A holder running into the bar, in work coordinates: the operation, the tool, the material it
/// meets (its Z span, its diameter span) and how much of it (mm² of the half section).
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct HolderHit {
    pub operation: String,
    pub tool: u32,
    pub z: [f64; 2],
    pub diameter: [f64; 2],
    pub area: f64,
}

/// A unit vector `deg` degrees round from +z toward +r.
fn dir(deg: f64) -> [f64; 2] {
    let a = deg.to_radians();
    [a.cos(), a.sin()]
}

/// The insert of `t` with its nose's centre at `c` ([z, r]): its outline's points.
pub fn insert(t: &LatheTool, c: [f64; 2]) -> Vec<[f64; 2]> {
    insert_sized(t, c, EDGE)
}

/// The insert with edges `edge` long.
fn insert_sized(t: &LatheTool, c: [f64; 2], edge: f64) -> Vec<[f64; 2]> {
    let (lead, trail) = (t.lead_angle(), t.trail_angle());
    let half = (t.insert_angle_deg / 2.0).to_radians();
    let rn = t.nose_radius;
    let bis = dir((lead + trail) / 2.0);
    // the sharp corner the nose rounds, and where the round meets the edges
    let corner = [c[0] - bis[0] * rn / half.sin(), c[1] - bis[1] * rn / half.sin()];
    let (ul, ut) = (dir(lead), dir(trail));
    let a = [corner[0] + ul[0] * edge, corner[1] + ul[1] * edge];
    let b = [corner[0] + ut[0] * edge, corner[1] + ut[1] * edge];
    let far = [a[0] + ut[0] * edge, a[1] + ut[1] * edge];
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

/// The tool `t`'s outline with its tip (where it is touched off) at `p` ([x radius, z]), as
/// [z, r] points: a turning insert with its nose's centre above and behind the tip; a boring bar's
/// mirrored, its nose's centre below; a grooving blade standing up from its corner toward the
/// chuck; a threading tool's V standing on its point; a drill's cone and body along the axis.
pub fn shape(t: &LatheTool, p: [f64; 2]) -> Vec<[f64; 2]> {
    let rn = t.nose_radius;
    match t.kind {
        LatheToolKind::Turning => insert(t, [p[1] + rn, p[0] + rn]),
        LatheToolKind::Boring => {
            let c = [p[1] + rn, p[0] - rn];
            insert_sized(t, c, EDGE / 2.0).into_iter().map(|q| [q[0], 2.0 * c[1] - q[1]]).collect()
        }
        LatheToolKind::Grooving => {
            let (w, h) = (t.blade_width(), t.reach() + 5.0);
            vec![[p[1], p[0]], [p[1] + w, p[0]], [p[1] + w, p[0] + h], [p[1], p[0] + h]]
        }
        LatheToolKind::Threading => {
            let h = super::plan::THREAD_V;
            let s = h * (t.point_angle().to_radians() / 2.0).tan();
            vec![[p[1], p[0]], [p[1] + s, p[0] + h], [p[1] + s, p[0] + h + 10.0], [p[1] - s, p[0] + h + 10.0], [p[1] - s, p[0] + h]]
        }
        LatheToolKind::Drill => {
            let r = t.diameter.unwrap_or(0.0) / 2.0;
            let cone = r / (t.point_angle().to_radians() / 2.0).tan();
            let body = 60.0;
            vec![[p[1], 0.0], [p[1] + cone, r], [p[1] + cone + body, r], [p[1] + cone + body, 0.0]]
        }
    }
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
    let mut area = Area::union_of(&[ring])?;
    // less the bore (through the parting allowance where it goes through)
    let mut bores: Vec<Ring> = Vec::new();
    let mut i = 0;
    let n = p.id.len();
    while i < n {
        if p.id.get(i).copied().unwrap_or(0.0) <= 0.0 {
            i += 1;
            continue;
        }
        let a = i;
        while i < n && p.id.get(i).copied().unwrap_or(0.0) > 0.0 {
            i += 1;
        }
        let b = i - 1;
        let z = |k: usize| k as f64 * p.dz - l;
        let mut pts: Vec<[f64; 2]> = (a..=b).map(|k| [z(k), p.id.get(k).copied().unwrap_or(0.0)]).collect();
        let za = if a == 0 { back - 1.0 } else { z(a) };
        if let Some(f) = pts.first().copied() {
            pts.insert(0, [za, f[1]]);
        }
        let mut ring: Ring = vec![[za, -1.0]];
        ring.extend(super::plan::simplify(&pts, 0.0005));
        let zb = if b + 1 >= n { 1.0 } else { z(b) };
        if let Some(last) = ring.last().copied() {
            ring.push([zb, last[1]]);
        }
        ring.push([zb, -1.0]);
        bores.push(ring);
    }
    if !bores.is_empty() {
        area = area.difference(&Area::union_of(&bores)?)?;
    }
    // threads cut to their minor diameter
    for t in &plan.threads {
        let (a, b, f) = (t.z[0], t.z[1], t.flank);
        let top = t.major + 1.0;
        let flank_top = f * (top - t.minor) / (t.major - t.minor).max(1e-9);
        let groove: Ring = vec![[a, t.minor], [b, t.minor], [b + flank_top, top], [a - flank_top, top]];
        area = area.difference(&Area::union_of(&[groove])?)?;
    }
    Ok(area)
}

/// Where the holder of `t` is with the tool's touch-off point at `p` ([x radius, z]), as [z, r]
/// points of the half section: what it covers round the axis.
pub fn holder_shape(t: &LatheTool, p: [f64; 2]) -> Option<Vec<[f64; 2]>> {
    let h = t.holder.as_ref()?;
    let [r0, r1] = h.radii(p[0]);
    let (z0, z1) = (p[1] + h.z[0], p[1] + h.z[1]);
    Some(vec![[z0, r0], [z1, r0], [z1, r1], [z0, r1]])
}

/// Holders meeting less material than this, or a layer thinner on average than
/// [`HOLDER_THICKNESS`], are only touching it (a holder's face passing level with a face of the
/// bar), mm².
const HOLDER_TOL: f64 = 1e-3;
const HOLDER_THICKNESS: f64 = 0.005;

/// Simulate `plan`.
pub fn simulate(job: &LatheJob, plan: &LathePlan) -> Result<(LatheSimReport, Area)> {
    let su = &plan.setup;
    let mut left = Area::rect(su.jaws, 0.0, su.front, su.radius)?;
    let start = left.area();
    let mut rapid_hits = Vec::new();
    let mut jaw_hits = 0usize;
    let mut holder_hits: Vec<HolderHit> = Vec::new();
    for op in &plan.ops {
        let t = job.tool(op.tool).ok_or_else(|| CamError::Invalid(format!("simulation: no tool T{}", op.tool)))?;
        let mut batch: Vec<Ring> = Vec::new();
        let mut held: Vec<Ring> = Vec::new();
        let mut at: Option<[f64; 2]> = None;
        // the holder's sweeps against what the cuts before and with them leave
        let mut check = |left: &Area, held: &mut Vec<Ring>| -> Result<()> {
            if held.is_empty() {
                return Ok(());
            }
            let met = left.intersection(&Area::union_of(held)?)?;
            held.clear();
            for p in pieces(&met, HOLDER_TOL, 0.0) {
                let across = (p.z[1] - p.z[0]).max((p.diameter[1] - p.diameter[0]) / 2.0);
                if p.area / across.max(1e-9) > HOLDER_THICKNESS && holder_hits.len() < 50 {
                    holder_hits.push(HolderHit { operation: op.name.clone(), tool: op.tool, z: p.z, diameter: p.diameter, area: p.area });
                }
            }
            Ok(())
        };
        for m in &op.moves {
            let to = m.to();
            let from = at.unwrap_or(to);
            let mut pts = shape(t, from);
            pts.extend(shape(t, to));
            let swept = hull(pts);
            // a drill on the axis passes inside the jaws
            if t.kind != LatheToolKind::Drill && swept.iter().any(|q| q[0] < su.jaws) {
                jaw_hits += 1;
            }
            if let (Some(mut a), Some(b)) = (holder_shape(t, from), holder_shape(t, to)) {
                a.extend(b);
                let h = hull(a);
                if h.iter().any(|q| q[0] < su.jaws) {
                    jaw_hits += 1;
                }
                held.push(h);
            }
            match m {
                TipMove::Feed(_) | TipMove::Thread { .. } => batch.push(swept),
                TipMove::Rapid(_) => {
                    if !batch.is_empty() {
                        left = left.difference(&Area::union_of(&batch)?)?;
                        batch.clear();
                    }
                    check(&left, &mut held)?;
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
        check(&left, &mut held)?;
    }
    let mut part = part_area(plan)?;
    // the parting cut: the kerf behind the part is no gouge, nor what stays in the chuck behind it
    let mut from = su.turned_to;
    for op in plan.ops.iter().filter(|o| o.kind == LatheOpKind::Parting) {
        let w = job.tool(op.tool).map_or(0.0, LatheTool::blade_width);
        let z = op.moves.iter().map(|m| m.to()[1]).fold(f64::INFINITY, f64::min);
        if z.is_finite() && w > 0.0 {
            part = part.difference(&Area::rect(z, -1.0, z + w, su.radius + 1.0)?)?;
            from = from.max(z + w);
        }
    }
    if from > su.turned_to {
        part = part.difference(&Area::rect(su.turned_to - 1.0, -1.0, from, su.radius + 1.0)?)?;
    }
    let window = Area::rect(from, 0.0, su.front + 1.0, su.radius + 1.0)?;
    let extra = left.intersection(&window)?.difference(&part)?;
    let cut = part.difference(&left)?;
    // the paths are chords within PATH_TOL of the nose's path, the part's outline within the
    // mesh's tolerance of it: a cut thinner than both on average along it is no gouge
    let gouges = pieces(&cut, 1e-3, super::plan::PATH_TOL + super::profile::MESH_TOL);
    let report = LatheSimReport {
        removed_area: start - left.area(),
        gouges,
        left: pieces(&extra, AREA_TOL, MIN_THICKNESS),
        rapid_hits,
        jaw_hits,
        holder_hits,
    };
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

/// The colour each kind of operation's feeds are drawn in.
pub fn colour(kind: LatheOpKind) -> &'static str {
    match kind {
        LatheOpKind::Facing => "#7c3aed",
        LatheOpKind::Drilling => "#0891b2",
        LatheOpKind::Roughing => "#d97706",
        LatheOpKind::Boring => "#db2777",
        LatheOpKind::Finishing => "#16a34a",
        LatheOpKind::Grooving => "#c2410c",
        LatheOpKind::Threading => "#4f46e5",
        LatheOpKind::Parting => "#b91c1c",
    }
}

/// The picture [`svg`] draws, as a PNG `width` pixels wide: the bar as left (grey), the part's
/// outline (blue), each operation's feeds in its [`colour`], rapids faint, the jaws' face (red)
/// and the axis.
pub fn png(plan: &LathePlan, left: &Area, width: u32) -> Result<Vec<u8>> {
    use tiny_skia::{FillRule, Paint, PathBuilder, Pixmap, Stroke, StrokeDash, Transform};
    let su = &plan.setup;
    let (z0, z1) = (su.jaws - 2.0, su.front + 6.0);
    let (r0, r1) = (-1.0, su.radius + 6.0);
    if !(z1 > z0 && r1 > r0 && z0.is_finite() && r1.is_finite()) {
        return Err(CamError::Invalid("the bar has no size".into()));
    }
    let width = width.clamp(64, 4096);
    let scale = f64::from(width) / (z1 - z0);
    let height = (((r1 - r0) * scale).round() as u32).clamp(16, 4096);
    let mut pm = Pixmap::new(width, height).ok_or_else(|| CamError::Invalid("picture size".into()))?;
    pm.fill(crate::preview::rgb("#ffffff", 1.0));
    // (z, r) in mm → pixels, r up
    let tf = Transform::from_row(scale as f32, 0.0, 0.0, -(scale as f32), (-z0 * scale) as f32, (r1 * scale) as f32);
    let paint = |hex: &str, a: f32| {
        let mut p = Paint::default();
        p.set_color(crate::preview::rgb(hex, a));
        p.anti_alias = true;
        p
    };
    let line = |w: f64| Stroke { width: (w / scale) as f32, ..Stroke::default() };
    let rings = |a: &Area| {
        let mut pb = PathBuilder::new();
        for ring in a.rings() {
            for (i, q) in ring.iter().enumerate() {
                if i == 0 {
                    pb.move_to(q[0] as f32, q[1] as f32);
                } else {
                    pb.line_to(q[0] as f32, q[1] as f32);
                }
            }
            pb.close();
        }
        pb.finish()
    };
    if let Some(p) = rings(left) {
        pm.fill_path(&p, &paint("#c8ccd2", 1.0), FillRule::EvenOdd, tf, None);
        pm.stroke_path(&p, &paint("#7a808a", 1.0), &line(0.6), tf, None);
    }
    if let Some(p) = rings(&part_area(plan)?) {
        pm.stroke_path(&p, &paint("#2f6fd6", 1.0), &line(1.2), tf, None);
    }
    for op in &plan.ops {
        let mut feeds = PathBuilder::new();
        let mut rapids = PathBuilder::new();
        let mut at: Option<[f64; 2]> = None;
        for m in &op.moves {
            let q = m.to();
            if let Some(a) = at {
                let pb = if matches!(m, TipMove::Rapid(_)) { &mut rapids } else { &mut feeds };
                pb.move_to(a[1] as f32, a[0] as f32);
                pb.line_to(q[1] as f32, q[0] as f32);
            }
            at = Some(q);
        }
        let c = colour(op.kind);
        if let Some(p) = rapids.finish() {
            let mut s = line(0.6);
            s.dash = StrokeDash::new(vec![(3.0 / scale) as f32, (3.0 / scale) as f32], 0.0);
            pm.stroke_path(&p, &paint(c, 0.35), &s, tf, None);
        }
        if let Some(p) = feeds.finish() {
            pm.stroke_path(&p, &paint(c, 0.9), &line(0.8), tf, None);
        }
    }
    let mut jaws = PathBuilder::new();
    jaws.move_to(su.jaws as f32, r0 as f32);
    jaws.line_to(su.jaws as f32, r1 as f32);
    if let Some(p) = jaws.finish() {
        pm.stroke_path(&p, &paint("#dc2626", 1.0), &line(2.0), tf, None);
    }
    let mut axis = PathBuilder::new();
    axis.move_to(z0 as f32, 0.0);
    axis.line_to(z1 as f32, 0.0);
    if let Some(p) = axis.finish() {
        let mut s = line(0.8);
        s.dash = StrokeDash::new(vec![(8.0 / scale) as f32, (3.0 / scale) as f32, (2.0 / scale) as f32, (3.0 / scale) as f32], 0.0);
        pm.stroke_path(&p, &paint("#555555", 1.0), &s, tf, None);
    }
    crate::preview::encode(&pm)
}
