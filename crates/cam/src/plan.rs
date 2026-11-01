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

use crate::adaptive::AdaptiveParams;
use crate::job::{CountersinkMethod, CutOutMethod, Job, LevelOrder, Pattern, Strategy, Tool};
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
    /// How far roughing and wall finishing cut through: `through`, or the skin's top when the
    /// parts are cut out through a skin.
    cut_to: f64,
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

/// One pocket region at one level, for roughing: `root` is the region at the top level it lies
/// in, so `area` order can cut each root all the way down.
struct RoughTask {
    z: f64,
    from: f64,
    root: usize,
    area: Area,
}

/// A point inside `area` (not on its edge).
fn interior_point(area: &Area) -> Option<[f64; 2]> {
    let mut a = area.clone();
    let mut last = None;
    for _ in 0..8 {
        if let Some(p) = a.rings().next().and_then(|r| r.first()).copied() {
            last = Some(p);
        }
        let (lo, hi) = a.bounds()?;
        let shrink = ((hi[0] - lo[0]).min(hi[1] - lo[1]) * 0.05).max(1e-3);
        let next = a.offset(-shrink, 0.01).ok()?;
        if next.is_empty() {
            break;
        }
        a = next;
    }
    a.rings().next().and_then(|r| r.first()).copied().or(last).filter(|p| area.contains(*p))
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

/// A pocket corner the finishing tool can't reach into: its tip, the lowest height it was
/// found at and how far the material left reaches into it.
struct Corner {
    at: [f64; 2],
    z_low: f64,
    depth: f64,
}

/// Material a tool of radius `rt` leaves at height `z`: where `pocket` is not covered by the
/// tool anywhere its centre may go (`allowed`). Each piece of it reaching more than 0.05 mm past
/// what the tool covers (more than the chords of a curved wall leave), beside it, is a corner
/// tighter than the tool: its tip is the point farthest in. Corners found at several heights
/// are one.
fn unreached_corners(pocket: &Area, allowed: &Area, rt: f64, z: f64, corners: &mut Vec<Corner>) -> Result<()> {
    if allowed.is_empty() {
        return Ok(());
    }
    let covered = allowed.offset(rt, PATH_TOL)?;
    let edges: Vec<([f64; 2], [f64; 2])> = covered.rings().flat_map(crate::region::ring_edges).collect();
    let dist = |p: [f64; 2]| {
        edges
            .iter()
            .map(|(a, b)| {
                let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                let l2 = dx * dx + dy * dy;
                let t = if l2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
                (a[0] + t * dx - p[0]).hypot(a[1] + t * dy - p[1])
            })
            .fold(f64::INFINITY, f64::min)
    };
    // past 0.05 mm: the hairlines along walls, where two regions' chords disagree, drop out
    // and each corner is a piece of its own
    for shape in &pocket.difference(&covered.offset(0.05, PATH_TOL)?)?.shapes {
        let d: Vec<([f64; 2], f64)> = shape.iter().flatten().map(|p| (*p, dist(*p))).collect();
        let Some(&(tip, depth)) = d.iter().max_by(|a, b| a.1.total_cmp(&b.1)) else { continue };
        let nearest = d.iter().map(|(_, x)| *x).fold(f64::INFINITY, f64::min);
        // a corner's leftover lies beside what the tool covers and is a wedge: a corner of
        // angle a leaves r²(cot(a/2) − (π − a)/2) over a depth of r(1/sin(a/2) − 1), 1.25
        // depth² at 90° and still 0.19 depth² at 20°; slivers away from every wall the tool
        // finishes are far thinner
        let area = Area { shapes: vec![shape.clone()] }.area();
        if nearest > 0.05 + 0.02 || area < 0.02 * depth * depth {
            continue;
        }
        match corners.iter_mut().find(|c| (c.at[0] - tip[0]).hypot(c.at[1] - tip[1]) < 0.01) {
            Some(c) => {
                c.z_low = c.z_low.min(z);
                c.depth = c.depth.max(depth);
            }
            None => corners.push(Corner { at: tip, z_low: z, depth }),
        }
    }
    Ok(())
}

/// Adaptive clearing's settings for roughing with `t`.
fn adaptive_params(st: &Strategy, t: &Tool, ramp: f64) -> AdaptiveParams {
    let rt = t.diameter / 2.0;
    AdaptiveParams {
        step: t.stepover(),
        max_engagement: st.roughing.max_engagement_deg.map(f64::to_radians).unwrap_or_else(|| {
            (AdaptiveParams::straight_engagement(t.stepover().min(rt * 0.9), rt) * 1.3).clamp(std::f64::consts::FRAC_PI_2, std::f64::consts::PI)
        }),
        both_ways: st.roughing.both_ways,
        ramp,
        retract: st.retract,
        rapid: st.rapid_feed,
        max_thinning: if st.feeds.chip_thinning { st.feeds.max_thinning } else { 1.0 },
        slotting: st.feeds.slotting,
        link: st.feeds.link.max(t.feed).min(st.rapid_feed.max(t.feed)),
    }
}

/// Rough `area` (where the cutter's centre may go) at `z`, the material's top there being
/// `from`, in the strategy's pattern; returns adaptive clearing's notes.
fn rough_area(b: &mut PathBuilder, area: &Area, z: f64, from: f64, t: &Tool, st: &Strategy, params: &AdaptiveParams) -> Result<Vec<String>> {
    match st.roughing.pattern {
        Pattern::Adaptive => crate::adaptive::clear_level(b, area, z, from, t, params),
        Pattern::Offset => {
            let rings = fill_rings(area, t.stepover())?;
            clear_rings(b, &rings, area, z, from, t, params.ramp);
            Ok(Vec::new())
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
        cut_to: match st.cut_out.method {
            CutOutMethod::Skin => f.bottom_z + st.cut_out.skin,
            CutOutMethod::Profile => f.bottom_z - job.stock.spoilboard_allowance,
        },
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
        targets.push(cx.cut_to);
    }

    // Roughing, leaving stock on walls and floors: each pocket region at each level, in the
    // order the strategy asks, adaptive or offset.
    if let Some(t) = rough
        && !targets.is_empty()
    {
        let rt = t.diameter / 2.0;
        let leave = st.stock_to_leave;
        // floors get their stock to leave; a cut through (to the skin or below the sheet) none
        let rough_targets: Vec<f64> = targets.iter().map(|z| if *z > cx.cut_to + Z_EPS { z + st.floor_stock_to_leave } else { *z }).collect();
        let mut tasks: Vec<RoughTask> = Vec::new();
        let mut roots: Vec<Area> = Vec::new();
        let mut from = f.top_z;
        for z in levels(f.top_z, &rough_targets, t.stepdown()) {
            let pocket = cx.pocket(z)?;
            let mut allowed = pocket.offset(-(rt + leave), PATH_TOL)?;
            // a region too narrow to leave the stock on its walls (a slot not much wider than
            // the cutter) is roughed leaving none, rather than left for the wall passes
            for sh in pocket.offset(-(rt + 0.02), PATH_TOL)?.shapes {
                let tight = Area { shapes: vec![sh] };
                if tight.intersection(&allowed)?.is_empty() {
                    if let Some((lo, hi)) = tight.bounds() {
                        cx.warn(format!(
                            "{label}: adaptive: the pocket at ({:.1}, {:.1}) on z {z:.3} is too narrow for T{} (Ø{}) to leave {leave} mm on its walls: it is roughed leaving none, close to slotting",
                            (lo[0] + hi[0]) / 2.0,
                            (lo[1] + hi[1]) / 2.0,
                            t.number,
                            t.diameter
                        ));
                    }
                    allowed = allowed.union(&tight)?;
                }
            }
            if allowed.is_empty() && !pocket.is_empty() {
                cx.warn(format!("{label}: a pocket at z {z:.3} is narrower than T{} (Ø{}) can rough", t.number, t.diameter));
            }
            for shape in allowed.shapes {
                let area = Area { shapes: vec![shape] };
                let inside = interior_point(&area);
                let root = match roots.iter().position(|r| inside.is_some_and(|p| r.contains(p))) {
                    Some(i) => i,
                    None => {
                        roots.push(area.clone());
                        roots.len() - 1
                    }
                };
                tasks.push(RoughTask { z, from, root, area });
            }
            from = z;
        }
        // depth: level by level (as made); area: each root region all the way down first
        if st.roughing.order == LevelOrder::Area {
            tasks.sort_by(|a, b| a.root.cmp(&b.root).then(b.z.total_cmp(&a.z)));
        }
        let params = adaptive_params(st, t, ramp);
        // one operation per level and region, so a sheet can be cut level by level
        for task in &tasks {
            let mut b = PathBuilder::new(cx.h, [0.0, 0.0]);
            for note in rough_area(&mut b, &task.area, task.z, task.from, t, st, &params)? {
                cx.warn(format!("{label}: {note}"));
            }
            b.up();
            if !b.moves.is_empty() {
                ops.push(cx.op(OpKind::Roughing, "roughing", t, b.moves));
            }
        }
    }

    // Floors at their exact height, then the walls level by level.
    if let Some(t) = finish
        && !targets.is_empty()
    {
        let rt = t.diameter / 2.0;
        let leave = st.floor_stock_to_leave;
        for (z, region) in &pocket_floors {
            let mut fb = PathBuilder::new(cx.h, [0.0, 0.0]);
            let pocket = cx.pocket(*z)?;
            let allowed = pocket.offset(-rt, PATH_TOL)?.intersection(&region.offset(rt, PATH_TOL)?)?;
            let rings = fill_rings(&allowed, t.stepover())?;
            clear_rings(&mut fb, &rings, &pocket.offset(-rt, PATH_TOL)?, *z, z + leave, t, ramp);
            fb.up();
            if !fb.moves.is_empty() {
                ops.push(cx.op(OpKind::FloorFinish, "floors", t, fb.moves));
            }
        }
        let mut from = f.top_z;
        let mut corners: Vec<Corner> = Vec::new();
        for z in levels(f.top_z, &targets, t.stepdown()) {
            let pocket = cx.pocket(z)?;
            let allowed = pocket.offset(-rt, PATH_TOL)?;
            if allowed.is_empty() && !pocket.is_empty() {
                cx.warn(format!("{label}: a pocket at z {z:.3} is narrower than T{} (Ø{}) can finish", t.number, t.diameter));
            }
            unreached_corners(&pocket, &allowed, rt, z, &mut corners)?;
            // the wall passes, each leaving less, the last none
            let mut wb = PathBuilder::new(cx.h, [0.0, 0.0]);
            for left in st.wall_passes.iter().copied().chain([0.0]) {
                let path = if left > 0.0 { pocket.offset(-(rt + left), PATH_TOL)? } else { allowed.clone() };
                for r in path.rings() {
                    let r = climb_inside(r);
                    let start = nearest_vertex(&r, [wb.at[0], wb.at[1]]);
                    let Some(&p) = r.get(start) else { continue };
                    wb.travel_to(p, from);
                    wb.line([p[0], p[1], z], t.plunge);
                    wb.ring(&r, start, z, t.feed);
                    wb.up();
                }
            }
            if !wb.moves.is_empty() {
                ops.push(cx.op(OpKind::WallFinish, "walls", t, wb.moves));
            }
            from = z;
        }
        for c in &corners {
            cx.warn(format!(
                "{label}: T{} (Ø{}) can't reach into the pocket corner at ({:.2}, {:.2}) down to z {:.3}: it leaves material up to {:.2} mm into the corner (rounded R{} where the design is sharper); a smaller finishing tool reaches further",
                t.number, t.diameter, c.at[0], c.at[1], c.z_low, c.depth, rt
            ));
        }
    }

    holes(&mut cx, &mut ops)?;
    chamfers(&mut cx, &mut ops)?;
    match st.cut_out.method {
        CutOutMethod::Skin => contour_and_separate(&mut cx, &mut ops, ramp)?,
        CutOutMethod::Profile => profile(&mut cx, &mut ops, ramp)?,
    }
    Ok(PartPlan { ops, warnings: cx.warnings })
}

/// Drill or mill every hole stack's round steps; countersink its cones.
fn holes(cx: &mut Ctx, ops: &mut Vec<Operation>) -> Result<()> {
    let job = cx.job;
    let label = cx.label;
    let mut drills: Vec<(u32, Vec<Move>)> = Vec::new();
    // milled steps by tool, blind or through (a through hole runs near the end: it leaks)
    let mut milled: Vec<(u32, bool, PathBuilder)> = Vec::new();
    let mut cones: Option<PathBuilder> = None;
    // end-milled countersinks by tool, on the part's top face or on a floor below it
    let mut sinks: Vec<(u32, bool, PathBuilder)> = Vec::new();
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
            let Some(t) = job.hole_tool(2.0 * r) else {
                let why = match job.finish_tool() {
                    Some(f) => format!("T{} (Ø{}) is too wide, no smaller end mill fits and no drill matches", f.number, f.diameter),
                    None => "no drill of its size and no end mill".to_string(),
                };
                cx.warn(format!("{label}: hole Ø{:.3} at ({:.3}, {:.3}): {why}", 2.0 * r, c[0], c[1]));
                top = s.z_bottom;
                continue;
            };
            let rt = t.diameter / 2.0;
            let outer = r - rt;
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
            let through = i == last && h.through;
            let k = match milled.iter().position(|(n, th, _)| *n == t.number && *th == through) {
                Some(k) => k,
                None => {
                    milled.push((t.number, through, PathBuilder::new(cx.h, c)));
                    milled.len() - 1
                }
            };
            let Some((_, _, b)) = milled.get_mut(k) else { continue };
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
            if job.strategy.countersinks == CountersinkMethod::EndMill {
                end_mill_cone(cx, h, s, &mut sinks);
                continue;
            }
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
    for (n, through, b) in milled {
        if let Some(t) = job.tool(n) {
            let (kind, what) = if through { (OpKind::ThroughHole, "through holes") } else { (OpKind::HoleMilling, "holes") };
            ops.push(cx.op(kind, what, t, b.moves));
        }
    }
    for (n, top, b) in sinks {
        if let Some(t) = job.tool(n) {
            ops.push(cx.op(if top { OpKind::Countersink } else { OpKind::Chamfer }, "countersinks", t, b.moves));
        }
    }
    if let (Some(b), Some(t)) = (cones, job.chamfer_tool()) {
        ops.push(cx.op(OpKind::Chamfer, "countersinks", t, b.moves));
    }
    Ok(())
}

/// Bore the cone `s` of the hole stack `h` with the end mill that fits the hole under it: rough
/// it in the tool's stepdown, each step rings from the cone inwards a stepover apart until one
/// lies within the hole (what is left inside, the hole's bore takes from the top down later);
/// then finish it in steps of `countersink_stepdown`, one ring each with the tool's edge on the
/// cone at the step's height, never past it. The cone widens upwards, so each step leaves a
/// stair as high as the step. On the part's top face the
/// countersink is cut first of all; on a floor below, after the walls.
fn end_mill_cone(cx: &mut Ctx, h: &HoleStack, s: &crate::part::HoleStep, sinks: &mut Vec<(u32, bool, PathBuilder)>) {
    let c = h.center;
    // the hole under it: the widest cylinder starting at or below the cone's foot
    let r_hole = h
        .steps
        .iter()
        .filter(|q| q.kind == "cylinder" && q.z_top <= s.z_bottom + Z_EPS)
        .max_by(|a, b| a.z_top.total_cmp(&b.z_top))
        .map_or(s.r_bottom, |q| q.r_top);
    let Some(t) = cx.job.hole_tool(2.0 * r_hole) else {
        cx.warn(format!(
            "{}: countersink Ø{:.3} at ({:.3}, {:.3}): no end mill fits the Ø{:.3} hole under it",
            cx.label,
            2.0 * s.r_top,
            c[0],
            c[1],
            2.0 * r_hole
        ));
        return;
    };
    let rt = t.diameter / 2.0;
    let height = s.z_top - s.z_bottom;
    if height <= Z_EPS {
        return;
    }
    let slope = (s.r_top - s.r_bottom) / height;
    let rings = |z: f64| -> Vec<f64> {
        // the tool's edge on the cone at z, then inwards until a ring lies within the hole
        let mut out = vec![s.r_bottom + (z - s.z_bottom) * slope - rt];
        while let Some(&r) = out.last() {
            if r <= r_hole + 1e-9 || r - t.stepover() <= 0.05 * rt || out.len() > 10_000 {
                break;
            }
            out.push(r - t.stepover());
        }
        out.reverse();
        out
    };
    let top = s.z_top >= cx.f.top_z - Z_EPS;
    let k = match sinks.iter().position(|(n, tp, _)| *n == t.number && *tp == top) {
        Some(k) => k,
        None => {
            sinks.push((t.number, top, PathBuilder::new(cx.h, c)));
            sinks.len() - 1
        }
    };
    let Some((_, _, b)) = sinks.get_mut(k) else { return };
    let step = cx.job.strategy.countersink_stepdown;
    // rough: steps the tool's stepdown deep, each cleared from the middle out to the cone at its
    // depth (the cone widens upwards: nothing above is touched)
    let mut from = s.z_top;
    let mut first = true;
    for z in levels(s.z_top, &[s.z_bottom], t.stepdown()) {
        let radii = rings(z);
        let Some(&inner) = radii.first() else { continue };
        if first {
            b.travel_to([c[0] + inner, c[1]], s.z_top);
            b.line([c[0] + inner, c[1], s.z_top], t.plunge);
            first = false;
        } else {
            b.line([c[0] + inner, c[1], from], t.feed);
        }
        b.helix(c, inner, z, (t.stepdown() / 2.0).min(t.diameter * 0.25), t.plunge.max(t.feed * 0.5));
        for r in radii.iter().skip(1) {
            b.circle(c, *r, z, t.feed);
        }
        from = z;
    }
    // finish from the top down, `countersink_stepdown` at a time: one ring each, the tool's edge
    // on the cone. The stair a rough step leaves is at most a stepdown across the slope, inside
    // the cutter's width, so the ring takes all of it
    let cone = |z: f64| s.r_bottom + (z - s.z_bottom) * slope - rt;
    let mut finishing = levels(s.z_top, &[s.z_bottom], step).into_iter().peekable();
    if let Some(&z1) = finishing.peek() {
        // up through the cleared middle, across to the first ring above the material
        b.line([b.at[0], b.at[1], s.z_top], t.feed);
        b.line([c[0] + cone(z1), c[1], s.z_top], t.feed);
    }
    for z in finishing {
        b.helix(c, cone(z), z, step, t.feed);
    }
    b.up();
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

/// The `skin` cut-out of one part with the profile tool. Roughing cleared round it leaving the
/// stock to leave; the wall passes (each leaving less: 0.3, then 0.1 by default) run round the
/// outline level by level down to the skin's top. The separation, last of all, runs the final
/// pass (leaving nothing) the same way, then cuts the skin through `skin_stepdown` at a time,
/// round the outline and inside each through cutout, down past the sheet's bottom: each level
/// an operation of its own, so the whole sheet is cut level by level. Climbing: anticlockwise
/// round the outside, clockwise inside a cutout.
fn contour_and_separate(cx: &mut Ctx, ops: &mut Vec<Operation>, ramp: f64) -> Result<()> {
    let Some(t) = cx.job.profile_tool() else { return Ok(()) };
    let rt = t.diameter / 2.0;
    let depth = cx.sheet_top - cx.through;
    if depth > t.flute_length + 1e-6 {
        cx.warn(format!("{}: the contour is {depth:.3} deep; T{}'s flutes are {}", cx.label, t.number, t.flute_length));
    }
    let outside: Vec<Ring> = cx.f.outline.offset(rt, PATH_TOL)?.shapes.iter().filter_map(|sh| sh.first().cloned()).collect();
    let mut inside: Vec<Ring> = Vec::new();
    for c in &cx.f.cutouts {
        let hole = Area::union_of(&[c.ring.iter().rev().copied().collect()])?;
        let path = hole.offset(-rt, PATH_TOL)?;
        if path.is_empty() {
            cx.warn(format!(
                "{}: the cutout at ({:.3}, {:.3}) is narrower than T{} (Ø{}): its skin is not cut through",
                cx.label,
                (c.bounds[0][0] + c.bounds[1][0]) / 2.0,
                (c.bounds[0][1] + c.bounds[1][1]) / 2.0,
                t.number,
                t.diameter
            ));
        }
        inside.extend(path.shapes.iter().filter_map(|sh| sh.first().map(climb_inside)));
    }
    let skin_top = cx.cut_to;
    // the wall passes round the outline, level by level down to the skin, each leaving less
    let mut from = cx.sheet_top;
    for z in levels(cx.sheet_top, &[skin_top], t.stepdown()) {
        let mut b = PathBuilder::new(cx.h, [0.0, 0.0]);
        for left in &cx.job.strategy.wall_passes {
            for r in cx.f.outline.offset(rt + left, PATH_TOL)?.shapes.iter().filter_map(|sh| sh.first()) {
                let start = nearest_vertex(r, [b.at[0], b.at[1]]);
                let Some(&p) = r.get(start) else { continue };
                b.travel_to(p, from);
                b.line([p[0], p[1], from], t.plunge);
                b.ramp_ring(r, start, z, ramp, t.feed, t.plunge);
                b.up();
            }
        }
        if !b.moves.is_empty() {
            ops.push(cx.op(OpKind::Profile, "contour", t, b.moves));
        }
        from = z;
    }
    // the separation: the last pass, leaving nothing, round the outline down to the skin level
    // by level, then through the skin round it and inside each cutout
    let mut from = cx.sheet_top;
    for z in levels(cx.sheet_top, &[skin_top], t.stepdown()) {
        let mut b = PathBuilder::new(cx.h, [0.0, 0.0]);
        for r in &outside {
            let start = nearest_vertex(r, [b.at[0], b.at[1]]);
            let Some(&p) = r.get(start) else { continue };
            b.travel_to(p, from);
            b.line([p[0], p[1], from], t.plunge);
            b.ramp_ring(r, start, z, ramp, t.feed, t.plunge);
            b.up();
        }
        if !b.moves.is_empty() {
            ops.push(cx.op(OpKind::Separation, "separation", t, b.moves));
        }
        from = z;
    }
    let step = cx.job.strategy.cut_out.skin_stepdown;
    // through to the spoilboard allowance, or down to a thin skin left to hold the parts
    let last = if cx.job.strategy.cut_out.final_skin > 0.0 { cx.f.bottom_z + cx.job.strategy.cut_out.final_skin } else { cx.through };
    for z in levels(skin_top, &[last], step) {
        let mut b = PathBuilder::new(cx.h, [0.0, 0.0]);
        for r in outside.iter().chain(&inside) {
            let start = nearest_vertex(r, [b.at[0], b.at[1]]);
            let Some(&p) = r.get(start) else { continue };
            b.travel_to(p, from);
            b.line([p[0], p[1], from], t.plunge);
            b.ramp_ring(r, start, z, ramp, t.feed, t.plunge);
            b.up();
        }
        if !b.moves.is_empty() {
            ops.push(cx.op(OpKind::Separation, "separation", t, b.moves));
        }
        from = z;
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

/// The whole sheet's operations, in work coordinates (`placements` are from the sheet's
/// front-left corner), and what could not be done.
///
/// Every placed copy's operations, and with the `skin` cut-out the roughing of a band round
/// the parts (the nest's spacing wide, so the gaps between parts are cleared whole) down to the
/// skin. They run kind by kind (material off before parts are cut free, the skin last), each
/// kind tool by tool and, over the whole sheet, level by level from the top down, so the sheet
/// is thinned evenly and stays held; by `area` roughing order, each part's roughing as planned.
pub fn plan_sheet(job: &Job, parts: &[(&PartFeatures, &PartPlan)], placements: &[Placement]) -> Result<(Vec<Operation>, Vec<String>)> {
    let mut warnings = Vec::new();
    let mut all: Vec<(usize, Operation)> = Vec::new();
    let mut outlines = Area::empty();
    for (i, pl) in placements.iter().enumerate() {
        let Some((f, plan)) = parts.get(pl.part) else { continue };
        outlines = outlines.union(&f.outline.placed(pl.quarter_turns, pl.offset[0], pl.offset[1]))?;
        // part Z → sheet Z: the part's bottom on the sheet's bottom, Z0 on the sheet's top
        let dz = f.bottom_z + job.stock.thickness;
        for op in &plan.ops {
            let moves = op.moves.iter().map(|m| place_move(m, pl.quarter_turns, pl.offset, dz)).collect();
            let name = op.name.replacen(&pl.label, &format!("{} #{}", pl.label, pl.copy + 1), 1);
            all.push((i, Operation { kind: op.kind, name, tool: op.tool, rpm: op.rpm, moves }));
        }
    }
    let st = &job.strategy;
    if st.cut_out.method == CutOutMethod::Skin
        && !outlines.is_empty()
        && let Some(t) = job.rough_tool()
    {
        let rt = t.diameter / 2.0;
        let leave = st.stock_to_leave;
        let spacing = job.nest.spacing;
        if spacing < t.diameter + 2.0 * leave + 0.5 {
            warnings.push(format!(
                "nest.spacing {spacing} mm leaves T{} (Ø{}) no room to clear between the parts (it needs {:.1} mm): the contour cuts what is left there",
                t.number,
                t.diameter,
                t.diameter + 2.0 * leave + 0.5
            ));
        }
        let [w, d] = job.stock.size;
        let sheet = Area::union_of(&[vec![[0.0, 0.0], [w, 0.0], [w, d], [0.0, d]]])?;
        let band = outlines.offset(spacing, PATH_TOL)?.intersection(&sheet)?;
        let allowed = band.offset(-rt, PATH_TOL)?.difference(&outlines.offset(rt + leave, PATH_TOL)?)?;
        let h = Heights { clearance: st.clearance, retract: st.retract };
        let ramp = st.ramp_angle_deg.to_radians();
        let params = adaptive_params(st, t, ramp);
        let skin_top = -job.stock.thickness + st.cut_out.skin;
        let mut from = 0.0;
        for z in levels(0.0, &[skin_top], t.stepdown()) {
            let mut b = PathBuilder::new(h, [0.0, 0.0]);
            for note in rough_area(&mut b, &allowed, z, from, t, st, &params)? {
                // the same note at every level once: without its height
                let note = match (note.find(" on z "), note.find(" on z ").and_then(|i| note.get(i + 6..)).and_then(|r| r.find(' '))) {
                    (Some(i), Some(j)) => format!("{}{}", note.get(..i).unwrap_or(""), note.get(i + 6 + j..).unwrap_or("")),
                    _ => note,
                };
                let w = format!("round the parts: {note}");
                if !warnings.contains(&w) {
                    warnings.push(w);
                }
            }
            b.up();
            if !b.moves.is_empty() {
                all.push((
                    usize::MAX,
                    Operation { kind: OpKind::Roughing, name: "round the parts: roughing".into(), tool: t.number, rpm: t.rpm, moves: b.moves },
                ));
            }
            from = z;
        }
    }
    let by_depth = |op: &Operation| match (op.kind, st.roughing.order) {
        (OpKind::Roughing, LevelOrder::Area) => 0.0,
        _ => -op.deepest(),
    };
    // a stable sort: copies (in nesting order) and each copy's operations keep their order
    // within a level
    all.sort_by(|a, b| {
        a.1.kind.rank().cmp(&b.1.kind.rank()).then(a.1.tool.cmp(&b.1.tool)).then(by_depth(&a.1).total_cmp(&by_depth(&b.1))).then(a.0.cmp(&b.0))
    });
    // from the sheet's corner to work zero
    let zero = job.stock.work_zero();
    Ok((
        all.into_iter()
            .map(|(_, mut op)| {
                op.moves = op.moves.iter().map(|m| place_move(m, 0, [-zero[0], -zero[1]], 0.0)).collect();
                op
            })
            .collect(),
        warnings,
    ))
}

/// A whole job planned: each part's features and toolpaths, the nesting, and the sheet's
/// operations in the order they run.
pub struct JobPlan {
    pub features: Vec<PartFeatures>,
    /// Which of each part's axes is up on the machine (its `orient`, or the one `auto` chose).
    pub ups: Vec<crate::job::Up>,
    pub plans: Vec<PartPlan>,
    /// The nesting in work coordinates.
    pub nesting: crate::nest::Nesting,
    pub ops: Vec<Operation>,
    /// Every part's warnings, and every copy that did not go on the sheet.
    pub warnings: Vec<String>,
}

/// Read, nest and plan `job`, whose parts' bodies are `bodies` (in the order of `job.parts`).
pub fn plan_job(job: &Job, bodies: &[solidcraft_kernel::Body]) -> Result<JobPlan> {
    if bodies.len() != job.parts.len() {
        return Err(CamError::Invalid(format!("{} bodies for {} parts", bodies.len(), job.parts.len())));
    }
    let mut features = Vec::new();
    let mut plans = Vec::new();
    let mut warnings = Vec::new();
    let mut ups = Vec::new();
    for (p, b) in job.parts.iter().zip(bodies) {
        let (up, _, f) = crate::part::oriented(b, p.orient, job.stock.thickness).map_err(|e| CamError::Invalid(format!("{}: {e}", p.label())))?;
        ups.push(up);
        let plan = plan_part(&f, job, p.label())?;
        warnings.extend(plan.warnings.iter().cloned());
        features.push(f);
        plans.push(plan);
    }
    let items: Vec<crate::nest::NestItem> = job
        .parts
        .iter()
        .zip(&features)
        .map(|(p, f)| crate::nest::NestItem { label: p.label(), quantity: p.quantity, outline: &f.outline, height: f.top_z - f.bottom_z })
        .collect();
    let nesting = crate::nest::nest(&job.stock, &job.nest, &items);
    warnings.extend(nesting.unplaced.iter().map(|u| format!("{} #{} is not on the sheet: {}", u.label, u.copy + 1, u.reason)));
    let pairs: Vec<(&PartFeatures, &PartPlan)> = features.iter().zip(&plans).collect();
    let (mut ops, sheet_warnings) = plan_sheet(job, &pairs, &nesting.placements)?;
    warnings.extend(sheet_warnings);
    if job.strategy.feeds.trim_air {
        crate::remaining::trim_air(job, &mut ops)?;
    }
    // tools the machine's changer does not hold
    if !job.machine.rack.is_empty() {
        let mut missing: Vec<u32> = Vec::new();
        for op in &ops {
            if !job.machine.rack.contains(&op.tool) && !missing.contains(&op.tool) {
                missing.push(op.tool);
            }
        }
        for n in missing {
            warnings.push(format!(
                "T{n} is not in the machine's rack (T{}): put it in the rack, renumber it in the tool library, or change it by hand",
                job.machine.rack.iter().map(u32::to_string).collect::<Vec<_>>().join(", T")
            ));
        }
    }
    let nesting = nesting.in_work(job.stock.work_zero());
    Ok(JobPlan { features, ups, plans, nesting, ops, warnings })
}

impl JobPlan {
    /// Every placed copy's outline, in work coordinates.
    pub fn placed_outlines(&self) -> Vec<Area> {
        self.nesting
            .placements
            .iter()
            .filter_map(|p| self.features.get(p.part).map(|f| f.outline.placed(p.quarter_turns, p.offset[0], p.offset[1])))
            .collect()
    }
}
