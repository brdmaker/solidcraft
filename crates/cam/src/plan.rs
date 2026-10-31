//! Toolpaths for one part, from its machining features, then for the whole nested sheet.
//!
//! Each part is planned in its own coordinates (its Z as modelled), then placed: turned and
//! moved as the nesting says, its bottom on the sheet's bottom, work Z0 the sheet's top.
//!
//! The material to keep at height z is every floor at or above z, the chamfer bands (their own
//! cutter finishes them) and the hole stacks (their own operations cut them). The pocket to clear
//! at z is the part's outline less that. Roughing clears pockets level by level with offset
//! rings, leaving `stock_to_leave`; finishing cuts floors at their exact height and walls with the
//! finish tool; the profile cuts the part free last, over tabs.

use schemars::JsonSchema;
use serde::Serialize;

use crate::job::{Job, Tool};
use crate::nest::Placement;
use crate::part::{HoleStack, PartFeatures};
use crate::region::{Area, Ring, signed_area};
use crate::toolpath::{Heights, Move, OpKind, Operation, PathBuilder, nearest_vertex, with_tabs};
use crate::{CamError, Result};

/// Chord tolerance of toolpath curves made from offsets (mm).
pub const PATH_TOL: f64 = 0.005;

/// Heights closer than this are the same level (mm).
const Z_EPS: f64 = 1e-6;

/// Most offset rings one area may give (a guard against runaway loops).
const MAX_RINGS: usize = 5_000;

/// Most levels one operation may cut.
const MAX_LEVELS: usize = 2_000;

/// The toolpaths of one part, in its own coordinates.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct PartPlan {
    pub ops: Vec<Operation>,
    /// What the plan could not do, each with the reason (a feature with no tool that fits, a
    /// pocket narrower than the cutter...).
    pub warnings: Vec<String>,
}

/// What to cut and where the material starts.
struct Ctx<'a> {
    f: &'a PartFeatures,
    job: &'a Job,
    label: &'a str,
    /// The sheet's top in the part's coordinates.
    sheet_top: f64,
    /// Where through-cuts end.
    through: f64,
    h: Heights,
    warnings: Vec<String>,
}

impl Ctx<'_> {
    fn warn(&mut self, w: String) {
        if !self.warnings.contains(&w) {
            self.warnings.push(w);
        }
    }

    fn op(&self, kind: OpKind, what: &str, tool: &Tool, moves: Vec<Move>) -> Operation {
        Operation { kind, name: format!("{}: {what}", self.label), tool: tool.number, rpm: tool.rpm, moves }
    }

    /// The material kept at height `z`: floors at or above it, chamfer bands that reach it and
    /// the hole stacks' circles.
    fn keep(&self, z: f64) -> Result<Area> {
        let mut k = Area::empty();
        for fl in self.f.floors.iter().filter(|fl| fl.z >= z - Z_EPS) {
            k = k.union(&fl.region)?;
        }
        for c in self.f.chamfers.iter().filter(|c| c.z_top >= z - Z_EPS) {
            k = k.union(&c.region)?;
        }
        for h in &self.f.holes {
            k = k.union(&Area::circle(h.center[0], h.center[1], hole_radius(h), PATH_TOL)?)?;
        }
        Ok(k)
    }

    /// The pocket to clear down to `z`: inside the outline, outside what is kept just above
    /// `z` (a floor at exactly `z` is the pocket's bottom, not material beside it).
    fn pocket(&self, z: f64) -> Result<Area> {
        self.f.outline.difference(&self.keep(z + Z_EPS * 10.0)?)
    }
}

fn hole_radius(h: &HoleStack) -> f64 {
    h.steps.iter().map(|s| s.r_top.max(s.r_bottom)).fold(0.0, f64::max)
}

/// Levels from `from` down to each target in turn, no deeper than `step` apart, ending exactly
/// on every target.
fn levels(from: f64, targets: &[f64], step: f64) -> Vec<f64> {
    let mut out = Vec::new();
    let mut z = from;
    let mut t: Vec<f64> = targets.iter().copied().filter(|t| *t < from - Z_EPS).collect();
    t.sort_by(|a, b| b.total_cmp(a));
    t.dedup_by(|a, b| (*a - *b).abs() < Z_EPS);
    let step = step.max(0.01);
    for target in t {
        let n = ((z - target) / step - 1e-9).ceil().max(1.0) as usize;
        let d = (z - target) / n as f64;
        for i in 1..=n {
            if out.len() >= MAX_LEVELS {
                return out;
            }
            out.push(if i == n { target } else { z - d * i as f64 });
        }
        z = target;
    }
    out
}

#[cfg(test)]
pub(crate) fn levels_for_tests(from: f64, targets: &[f64], step: f64) -> Vec<f64> {
    levels(from, targets, step)
}

/// Offset rings filling `area` (the region the tool's centre may reach), `stepover` apart,
/// innermost first, each with how far in it is.
fn fill_rings(area: &Area, stepover: f64) -> Result<Vec<Ring>> {
    let mut rings = Vec::new();
    let mut a = area.clone();
    let mut depth = 0usize;
    let mut layers: Vec<Vec<Ring>> = Vec::new();
    while !a.is_empty() && rings.len() < MAX_RINGS && depth < MAX_RINGS {
        let layer: Vec<Ring> = a.rings().cloned().collect();
        rings.extend(layer.iter().cloned());
        layers.push(layer);
        a = a.offset(-stepover, PATH_TOL)?;
        depth += 1;
    }
    Ok(layers.into_iter().rev().flatten().collect())
}

/// A ring run with the finished wall on its left: pockets and islands both clockwise about the
/// cleared side. Outer rings of an area run counter-clockwise, holes clockwise; reverse both.
fn climb_inside(r: &Ring) -> Ring {
    r.iter().rev().copied().collect()
}

/// Is the straight move from `a` to `b` inside `area` all the way?
fn inside_all_the_way(area: &Area, a: [f64; 2], b: [f64; 2]) -> bool {
    let n = ((a[0] - b[0]).hypot(a[1] - b[1]) / 0.5).ceil().clamp(1.0, 400.0) as usize;
    (0..=n).all(|i| {
        let t = i as f64 / n as f64;
        area.contains([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t])
    })
}

/// Clear `rings` (innermost first) at height `z`, the material's top there being `from`.
/// Moves between rings at depth when the way lies inside `allowed`, else lifts and re-enters.
fn clear_rings(b: &mut PathBuilder, rings: &[Ring], allowed: &Area, z: f64, from: f64, tool: &Tool, ramp: f64) {
    let mut down = false;
    for r in rings {
        let r = climb_inside(r);
        let Some(_) = r.first() else { continue };
        let start = nearest_vertex(&r, [b.at[0], b.at[1]]);
        let Some(&p) = r.get(start) else { continue };
        let reach = (p[0] - b.at[0]).hypot(p[1] - b.at[1]);
        if down && reach <= tool.diameter * 2.0 && inside_all_the_way(allowed, [b.at[0], b.at[1]], p) {
            b.line([p[0], p[1], z], tool.feed);
            b.ring(&r, start, z, tool.feed);
        } else {
            b.travel_to(p, from);
            b.line([p[0], p[1], from], tool.plunge);
            b.ramp_ring(&r, start, z, ramp, tool.feed, tool.plunge);
            down = true;
        }
    }
}

/// Plan one part.
pub fn plan_part(f: &PartFeatures, job: &Job, label: &str) -> Result<PartPlan> {
    let st = &job.strategy;
    let height = f.top_z - f.bottom_z;
    let thickness = job.stock.thickness;
    if height > thickness + 1e-6 {
        return Err(CamError::Invalid(format!("{label}: the part is {height:.3} mm tall; the sheet is {thickness:.3} mm thick")));
    }
    let sheet_top = f.bottom_z + thickness;
    let mut cx = Ctx {
        f,
        job,
        label,
        sheet_top,
        through: f.bottom_z - job.stock.spoilboard_allowance,
        h: Heights { clearance: sheet_top + st.clearance, retract: st.retract },
        warnings: f.unsupported.iter().map(|n| format!("{label}: face {} ({}) is not machined: {}", n.face, n.surface, n.reason)).collect(),
    };
    let mut ops = Vec::new();
    let ramp = st.ramp_angle_deg.to_radians();
    let rough = job.rough_tool();
    let finish = job.finish_tool();
    if rough.is_none() || finish.is_none() {
        cx.warn(format!("{label}: the job has no flat end mill: no facing, pockets, holes without a drill, or profile"));
    }

    // Facing: the sheet above a part thinner than it.
    if let Some(t) = rough
        && sheet_top > f.top_z + Z_EPS
    {
        let rt = t.diameter / 2.0;
        let area = f.outline.offset(rt, PATH_TOL)?;
        let rings = fill_rings(&area, t.stepover())?;
        let mut b = PathBuilder::new(cx.h, [0.0, 0.0]);
        let mut from = sheet_top;
        for z in levels(sheet_top, &[f.top_z], t.stepdown()) {
            clear_rings(&mut b, &rings, &area, z, from, t, ramp);
            b.up();
            from = z;
        }
        ops.push(cx.op(OpKind::Facing, "facing", t, b.moves));
    }

    // Pocket floors (not the top, not a hole's seat) and through cutouts.
    let pocket_floors: Vec<(f64, Area)> = f.floors.iter().filter(|fl| !fl.top && !fl.in_hole).map(|fl| (fl.z, fl.region.clone())).collect();
    let mut targets: Vec<f64> = pocket_floors.iter().map(|(z, _)| *z).collect();
    if !f.cutouts.is_empty() {
        targets.push(cx.through);
    }

    // Roughing, leaving stock on walls and floors.
    if let Some(t) = rough
        && !targets.is_empty()
    {
        let rt = t.diameter / 2.0;
        let leave = st.stock_to_leave;
        let rough_targets: Vec<f64> = targets.iter().map(|z| if *z > cx.through + Z_EPS { z + leave } else { *z }).collect();
        let mut b = PathBuilder::new(cx.h, [0.0, 0.0]);
        let mut from = f.top_z;
        for z in levels(f.top_z, &rough_targets, t.stepdown()) {
            let pocket = cx.pocket(z)?;
            let allowed = pocket.offset(-(rt + leave), PATH_TOL)?;
            if allowed.is_empty() && !pocket.is_empty() {
                cx.warn(format!("{label}: a pocket at z {z:.3} is narrower than T{} (Ø{}) can rough", t.number, t.diameter));
            }
            let rings = fill_rings(&allowed, t.stepover())?;
            clear_rings(&mut b, &rings, &allowed, z, from, t, ramp);
            b.up();
            from = z;
        }
        if !b.moves.is_empty() {
            ops.push(cx.op(OpKind::Roughing, "roughing", t, b.moves));
        }
    }

    // Floors at their exact height, then the walls level by level.
    if let Some(t) = finish
        && !targets.is_empty()
    {
        let rt = t.diameter / 2.0;
        let leave = st.stock_to_leave;
        let mut fb = PathBuilder::new(cx.h, [0.0, 0.0]);
        for (z, region) in &pocket_floors {
            let pocket = cx.pocket(*z)?;
            let allowed = pocket.offset(-rt, PATH_TOL)?.intersection(&region.offset(rt, PATH_TOL)?)?;
            let rings = fill_rings(&allowed, t.stepover())?;
            clear_rings(&mut fb, &rings, &pocket.offset(-rt, PATH_TOL)?, *z, z + leave, t, ramp);
            fb.up();
        }
        if !fb.moves.is_empty() {
            ops.push(cx.op(OpKind::FloorFinish, "floors", t, fb.moves));
        }
        let mut wb = PathBuilder::new(cx.h, [0.0, 0.0]);
        let mut from = f.top_z;
        for z in levels(f.top_z, &targets, t.stepdown()) {
            let pocket = cx.pocket(z)?;
            let allowed = pocket.offset(-rt, PATH_TOL)?;
            if allowed.is_empty() && !pocket.is_empty() {
                cx.warn(format!("{label}: a pocket at z {z:.3} is narrower than T{} (Ø{}) can finish", t.number, t.diameter));
            }
            let walls: Vec<Ring> = allowed.rings().cloned().collect();
            for r in &walls {
                let r = climb_inside(r);
                let start = nearest_vertex(&r, [wb.at[0], wb.at[1]]);
                let Some(&p) = r.get(start) else { continue };
                wb.travel_to(p, from);
                wb.line([p[0], p[1], z], t.plunge);
                wb.ring(&r, start, z, t.feed);
                wb.up();
            }
            from = z;
        }
        if !wb.moves.is_empty() {
            ops.push(cx.op(OpKind::WallFinish, "walls", t, wb.moves));
        }
    }

    holes(&mut cx, &mut ops)?;
    chamfers(&mut cx, &mut ops)?;
    profile(&mut cx, &mut ops, ramp)?;
    Ok(PartPlan { ops, warnings: cx.warnings })
}

/// Drill or mill every hole stack's round steps; countersink its cones.
fn holes(cx: &mut Ctx, ops: &mut Vec<Operation>) -> Result<()> {
    let job = cx.job;
    let label = cx.label;
    let mut drills: Vec<(u32, Vec<Move>)> = Vec::new();
    let mut milled: Option<PathBuilder> = None;
    let mut cones: Option<PathBuilder> = None;
    let holes = cx.f.holes.clone();
    for h in &holes {
        let c = h.center;
        let cyl: Vec<_> = h.steps.iter().filter(|s| s.kind == "cylinder").collect();
        let mut top = h.z_top;
        let last = cyl.len().saturating_sub(1);
        for (i, s) in cyl.iter().enumerate() {
            let bottom = if i == last && h.through { cx.through } else { s.z_bottom };
            let r = s.r_top;
            // A through hole of a drill's size, under nothing wider than itself that would need
            // a flat seat: drill it.
            if i == last
                && h.through
                && let Some(d) = job.drill(2.0 * r)
            {
                let tip = r / (d.tip_angle().to_radians() / 2.0).tan();
                let depth = h.z_top - (bottom - tip);
                if depth > d.flute_length {
                    cx.warn(format!(
                        "{label}: hole Ø{:.3} at ({:.3}, {:.3}) is {depth:.3} deep; drill T{} reaches {}",
                        2.0 * r,
                        c[0],
                        c[1],
                        d.number,
                        d.flute_length
                    ));
                }
                // Drilling runs before any hole is milled, so the material's top over the hole
                // is still the stack's top: retract above that, never into an uncut counterbore.
                let clear = cx.h.clearance;
                let mv = Move::Drill { at: c, back: clear, r: h.z_top + cx.h.retract, bottom: bottom - tip, peck: d.peck, feed: d.feed };
                let over = Move::Rapid { to: [c[0], c[1], clear] };
                match drills.iter_mut().find(|(n, _)| *n == d.number) {
                    Some((_, m)) => m.extend([over, mv]),
                    None => drills.push((d.number, vec![over, mv])),
                }
                top = s.z_bottom;
                continue;
            }
            let Some(t) = job.finish_tool() else {
                cx.warn(format!("{label}: hole Ø{:.3} at ({:.3}, {:.3}): no drill of its size and no end mill", 2.0 * r, c[0], c[1]));
                top = s.z_bottom;
                continue;
            };
            let rt = t.diameter / 2.0;
            let outer = r - rt;
            if outer < 0.05 * rt {
                cx.warn(format!(
                    "{label}: hole Ø{:.3} at ({:.3}, {:.3}): T{} (Ø{}) is too wide and no drill matches",
                    2.0 * r,
                    c[0],
                    c[1],
                    t.number,
                    t.diameter
                ));
                top = s.z_bottom;
                continue;
            }
            if top - bottom > t.flute_length {
                cx.warn(format!("{label}: hole Ø{:.3} at ({:.3}, {:.3}) is deeper than T{}'s flutes", 2.0 * r, c[0], c[1], t.number));
            }
            // Rings from the middle out: the innermost a helix entry, each ring wider by the
            // stepover, the last on the wall.
            let mut radii = vec![outer];
            while let Some(&r_in) = radii.last() {
                let next = r_in - t.stepover();
                if next <= 0.05 * rt || radii.len() > 10_000 {
                    break;
                }
                radii.push(next);
            }
            radii.reverse();
            let b = milled.get_or_insert_with(|| PathBuilder::new(cx.h, c));
            let first = radii.first().copied().unwrap_or(outer);
            b.travel_to([c[0] + first, c[1]], top);
            let mut from = top;
            for z in levels(top, &[bottom], t.stepdown()) {
                b.line([c[0] + first, c[1], from], t.plunge);
                b.helix(c, first, z, (t.stepdown() / 2.0).min(t.diameter * 0.25), t.feed);
                for r in radii.iter().skip(1) {
                    b.circle(c, *r, z, t.feed);
                }
                b.line([c[0] + first, c[1], z], t.feed);
                from = z;
            }
            b.up();
            top = s.z_bottom;
        }
        // Countersinks and the chamfer round a round recess: cones of the stack.
        for s in h.steps.iter().filter(|s| s.kind == "cone") {
            let Some(t) = job.chamfer_tool() else {
                cx.warn(format!("{label}: countersink Ø{:.3} at ({:.3}, {:.3}): no chamfer cutter in the job", 2.0 * s.r_top, c[0], c[1]));
                continue;
            };
            let half = (t.tip_angle() / 2.0).to_radians();
            let cone_half = ((s.r_top - s.r_bottom) / (s.z_top - s.z_bottom).max(1e-9)).atan();
            if (cone_half - half).abs() > 0.5f64.to_radians() {
                cx.warn(format!(
                    "{label}: countersink at ({:.3}, {:.3}) is {:.1}°; T{} is {:.1}°",
                    c[0],
                    c[1],
                    2.0 * cone_half.to_degrees(),
                    t.number,
                    t.tip_angle()
                ));
                continue;
            }
            // Tip below the cone's foot by `chamfer_tip_below`, axis in from the foot by the
            // same times tan(half): the flank lies on the cone. Past the centre: plunge there.
            let below = cx.job.strategy.chamfer_tip_below.min(s.r_bottom / half.tan());
            let r = (s.r_bottom - below * half.tan()).max(0.0);
            let z_tip = s.z_bottom - below;
            // the cutter's cone must span from its tip up to the countersink's top
            if t.diameter / 2.0 < (s.z_top - z_tip) * half.tan() - 1e-6 {
                cx.warn(format!("{label}: countersink Ø{:.3} at ({:.3}, {:.3}) is wider than T{}", 2.0 * s.r_top, c[0], c[1], t.number));
                continue;
            }
            let b = cones.get_or_insert_with(|| PathBuilder::new(cx.h, c));
            b.travel_to(c, s.z_top);
            b.line([c[0], c[1], z_tip], t.plunge);
            if r > 1e-6 {
                b.circle(c, r, z_tip, t.feed);
                b.line([c[0], c[1], z_tip], t.feed);
            }
            b.up();
        }
    }
    for (n, moves) in drills {
        if let Some(t) = job.tool(n) {
            ops.push(cx.op(OpKind::Drilling, "drilling", t, moves));
        }
    }
    if let (Some(b), Some(t)) = (milled, job.finish_tool()) {
        ops.push(cx.op(OpKind::HoleMilling, "holes", t, b.moves));
    }
    if let (Some(b), Some(t)) = (cones, job.chamfer_tool()) {
        ops.push(cx.op(OpKind::Chamfer, "countersinks", t, b.moves));
    }
    Ok(())
}

/// Chamfer bands along openings (before the profile) and round the outline (after it, over
/// the profile's slot).
fn chamfers(cx: &mut Ctx, ops: &mut Vec<Operation>) -> Result<()> {
    if cx.f.chamfers.is_empty() {
        return Ok(());
    }
    let Some(t) = cx.job.chamfer_tool() else {
        cx.warn(format!("{}: {} chamfer band(s): no chamfer cutter in the job", cx.label, cx.f.chamfers.len()));
        return Ok(());
    };
    let half = (t.tip_angle() / 2.0).to_radians();
    let mut inner = PathBuilder::new(cx.h, [0.0, 0.0]);
    let mut outer = PathBuilder::new(cx.h, [0.0, 0.0]);
    let bands = cx.f.chamfers.clone();
    for band in &bands {
        if (band.angle_deg - (90.0 - t.tip_angle() / 2.0)).abs() > 0.5 {
            cx.warn(format!(
                "{}: a {:.1}° chamfer needs a {:.1}° cutter; T{} is {:.1}°",
                cx.label,
                band.angle_deg,
                180.0 - 2.0 * band.angle_deg,
                t.number,
                t.tip_angle()
            ));
            continue;
        }
        let below = cx.job.strategy.chamfer_tip_below;
        let z_tip = band.z_bottom - below;
        let reach = (band.z_top - z_tip) * half.tan() + below * half.tan();
        if reach > t.diameter / 2.0 + 1e-6 {
            cx.warn(format!("{}: a {:.3} mm chamfer is wider than T{} reaches", cx.label, band.width, t.number));
            continue;
        }
        let shift = below * half.tan();
        let outline = band.on == "outline";
        // The foot of the band: its outer ring round the outline, its holes round an opening.
        let mut path_rings = Vec::new();
        for shape in &band.region.shapes {
            if outline {
                if let Some(o) = shape.first() {
                    path_rings.extend(
                        Area::from_rings(std::slice::from_ref(o))?.offset(shift, PATH_TOL)?.shapes.into_iter().filter_map(|s| s.into_iter().next()),
                    );
                }
            } else {
                for hole in shape.iter().skip(1) {
                    let filled: Ring = hole.iter().rev().copied().collect();
                    let a = Area::from_rings(&[filled])?.offset(-shift, PATH_TOL)?;
                    path_rings.extend(a.shapes.into_iter().filter_map(|s| s.into_iter().next()).map(|r| climb_inside(&r)));
                }
            }
        }
        let b = if outline { &mut outer } else { &mut inner };
        for r in path_rings {
            let start = nearest_vertex(&r, [b.at[0], b.at[1]]);
            let Some(&p) = r.get(start) else { continue };
            b.travel_to(p, band.z_top);
            b.line([p[0], p[1], z_tip], t.plunge);
            b.ring(&r, start, z_tip, t.feed);
            b.up();
        }
    }
    if !inner.moves.is_empty() {
        ops.push(cx.op(OpKind::Chamfer, "chamfers", t, inner.moves));
    }
    if !outer.moves.is_empty() {
        ops.push(cx.op(OpKind::EdgeChamfer, "edge chamfer", t, outer.moves));
    }
    Ok(())
}

/// Cut the part free round its outline, climbing (part on the left), over tabs.
fn profile(cx: &mut Ctx, ops: &mut Vec<Operation>, ramp: f64) -> Result<()> {
    let Some(t) = cx.job.profile_tool() else { return Ok(()) };
    let rt = t.diameter / 2.0;
    let depth = cx.sheet_top - cx.through;
    if depth > t.flute_length + 1e-6 {
        cx.warn(format!("{}: the profile is {depth:.3} deep; T{}'s flutes are {}", cx.label, t.number, t.flute_length));
    }
    let tabs = &cx.job.strategy.tabs;
    let tab_top = cx.f.bottom_z + tabs.height;
    let mut b = PathBuilder::new(cx.h, [0.0, 0.0]);
    let path = cx.f.outline.offset(rt, PATH_TOL)?;
    for shape in &path.shapes {
        let Some(r) = shape.first() else { continue };
        if signed_area(r) <= 0.0 {
            continue;
        }
        let pts = with_tabs(r, tabs.count as usize, tabs.width + t.diameter);
        let Some(&(p0, _)) = pts.first() else { continue };
        b.travel_to(p0, cx.sheet_top);
        b.line([p0[0], p0[1], cx.sheet_top], t.plunge);
        let mut from = cx.sheet_top;
        let tab_len = tabs.width + t.diameter;
        for z in levels(cx.sheet_top, &[cx.through], t.stepdown()) {
            let tabbed = tabs.count > 0 && z < tab_top - Z_EPS;
            // Ramp down over the first stretch, which is clear of tabs (they start half a
            // spacing in), steeper when that stretch is short.
            let clear = if tabbed { (crate::region::ring_length(r) / tabs.count as f64 - tab_len) / 2.0 } else { f64::INFINITY };
            let run = ((from - z) / ramp.tan()).min(clear.max(1e-3) * 0.9);
            let mut gone = 0.0;
            let mut prev = p0;
            for &(p, tab) in pts.iter().skip(1) {
                gone += (p[0] - prev[0]).hypot(p[1] - prev[1]);
                let floor = if tabbed && tab { tab_top } else { z };
                let ramped = from - (from - z) * (gone / run.max(1e-9)).min(1.0);
                let zz = ramped.max(floor);
                if gone <= run || (zz - b.at[2]).abs() <= Z_EPS {
                    b.line([p[0], p[1], zz], t.feed);
                } else {
                    // onto or off a tab: level to its edge, then straight up or down
                    b.line([p[0], p[1], b.at[2]], t.feed);
                    b.line([p[0], p[1], zz], t.plunge);
                }
                prev = p;
            }
            if b.at[2] > z + Z_EPS && !tabbed {
                b.line([p0[0], p0[1], z], t.plunge);
            }
            from = z;
        }
        b.up();
    }
    if !b.moves.is_empty() {
        ops.push(cx.op(OpKind::Profile, "profile", t, b.moves));
    }
    Ok(())
}

/// A move turned `q` quarter turns about the origin, moved by `d`, and lowered by `dz`.
fn place_move(m: &Move, q: u8, d: [f64; 2], dz: f64) -> Move {
    let t = |p: [f64; 2]| {
        let p = match q % 4 {
            1 => [-p[1], p[0]],
            2 => [-p[0], -p[1]],
            3 => [p[1], -p[0]],
            _ => p,
        };
        [p[0] + d[0], p[1] + d[1]]
    };
    let t3 = |p: [f64; 3]| {
        let xy = t([p[0], p[1]]);
        [xy[0], xy[1], p[2] - dz]
    };
    match m {
        Move::Rapid { to } => Move::Rapid { to: t3(*to) },
        Move::Line { to, feed } => Move::Line { to: t3(*to), feed: *feed },
        Move::Arc { to, center, ccw, feed } => Move::Arc { to: t3(*to), center: t(*center), ccw: *ccw, feed: *feed },
        Move::Drill { at, back, r, bottom, peck, feed } => {
            Move::Drill { at: t(*at), back: back - dz, r: r - dz, bottom: bottom - dz, peck: *peck, feed: *feed }
        }
    }
}

/// The whole sheet's operations, in sheet coordinates: every placed copy's, in the order they
/// must run (material off before parts are cut free), grouped by tool within each step.
pub fn plan_sheet(job: &Job, parts: &[(&PartFeatures, &PartPlan)], placements: &[Placement]) -> Vec<Operation> {
    let mut all: Vec<(u8, u32, usize, Operation)> = Vec::new();
    for (i, pl) in placements.iter().enumerate() {
        let Some((f, plan)) = parts.get(pl.part) else { continue };
        // part Z → sheet Z: the part's bottom on the sheet's bottom, Z0 on the sheet's top
        let dz = f.bottom_z + job.stock.thickness;
        for op in &plan.ops {
            let moves = op.moves.iter().map(|m| place_move(m, pl.quarter_turns, pl.offset, dz)).collect();
            let name = op.name.replacen(&pl.label, &format!("{} #{}", pl.label, pl.copy + 1), 1);
            all.push((op.kind.rank(), op.tool, i, Operation { kind: op.kind, name, tool: op.tool, rpm: op.rpm, moves }));
        }
    }
    all.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
    all.into_iter().map(|(.., op)| op).collect()
}
