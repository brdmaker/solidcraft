//! Cutting simulation: the sheet as a height map, cut by every move of a planned job with the
//! real tool shapes, then compared with the parts as designed (finely triangulated).
//!
//! The map holds the top of the remaining material over a grid of square cells (Z0 the sheet's
//! top, its bottom at −thickness, work coordinates). A move lowers every cell its tool passes
//! over to the tool's underside there: flat for an end mill, a cone for a drill or chamfer mill.
//! Straight moves are swept exactly; arcs are split so the chords stay within a quarter cell.
//!
//! The comparison works cell by cell inside the parts' outlines, against the highest point of
//! each part's mesh over the cell (the bottom of the sheet where an opening goes through). Next
//! to a wall the target jumps within a cell, so a cut is a gouge only if it goes below the lowest
//! target of the cell and its neighbours, and material is left only if it stands above their
//! highest.

use schemars::JsonSchema;
use serde::Serialize;
use solidcraft_geom::Vec3;
use solidcraft_kernel::Body;

use crate::job::{Job, ToolKind};
use crate::plan::JobPlan;
use crate::toolpath::{Move, Operation};
use crate::{CamError, Result};

/// The most cells of a map; a finer cell over a large sheet is refused.
pub const MAX_CELLS: usize = 50_000_000;

/// Chord tolerance of the parts' meshes the result is compared with (mm).
pub const TARGET_TOL: f64 = 0.005;

/// How deep a cut must go into a part to count as a gouge (mm).
pub const GOUGE_TOL: f64 = 0.02;

/// How high material must stand on a part to count as left (mm).
pub const LEFT_TOL: f64 = 0.05;

/// The sheet's material as heights over a grid of cells (work coordinates).
#[derive(Clone, Debug)]
pub struct HeightMap {
    /// The first cell's lower-left corner.
    pub x0: f64,
    pub y0: f64,
    /// A cell's side, mm.
    pub h: f64,
    pub nx: usize,
    pub ny: usize,
    /// Top of the material over each cell, row by row from the front.
    pub z: Vec<f32>,
}

impl HeightMap {
    /// The block `lo`..`hi` in X and Y with its top at `top`, cells about `h` wide.
    pub fn new(lo: [f64; 2], hi: [f64; 2], top: f64, h: f64) -> Result<HeightMap> {
        let (w, d) = (hi[0] - lo[0], hi[1] - lo[1]);
        if !(w > 0.0 && d > 0.0 && w.is_finite() && d.is_finite() && h > 0.0 && h.is_finite()) {
            return Err(CamError::Invalid("simulation: the stock has no size".into()));
        }
        let (nx, ny) = ((w / h).ceil() as usize, (d / h).ceil() as usize);
        if nx.saturating_mul(ny) > MAX_CELLS {
            return Err(CamError::Invalid(format!("simulation: {nx} × {ny} cells of {h} mm are more than {MAX_CELLS}: use larger cells")));
        }
        Ok(HeightMap { x0: lo[0], y0: lo[1], h, nx, ny, z: vec![top as f32; nx * ny] })
    }

    /// The centre of cell (i, j).
    pub fn centre(&self, i: usize, j: usize) -> [f64; 2] {
        [self.x0 + (i as f64 + 0.5) * self.h, self.y0 + (j as f64 + 0.5) * self.h]
    }

    /// The cells whose centres may lie in `lo`..`hi`: first and last across and up.
    pub(crate) fn span(&self, lo: [f64; 2], hi: [f64; 2]) -> Option<(usize, usize, usize, usize)> {
        let f = |v: f64, o: f64| ((v - o) / self.h - 0.5).floor();
        let c = |v: f64, o: f64| ((v - o) / self.h - 0.5).ceil();
        let (i0, i1) = (f(lo[0], self.x0).max(0.0), c(hi[0], self.x0).min(self.nx as f64 - 1.0));
        let (j0, j1) = (f(lo[1], self.y0).max(0.0), c(hi[1], self.y0).min(self.ny as f64 - 1.0));
        (i0 <= i1 && j0 <= j1).then_some((i0 as usize, i1 as usize, j0 as usize, j1 as usize))
    }

    /// The height at cell (i, j).
    pub fn at(&self, i: usize, j: usize) -> f64 {
        self.z.get(j * self.nx + i).copied().map_or(f64::NAN, f64::from)
    }
}

/// A cutter's underside: its radius and how far above its tip the underside is at distance `d`
/// from its axis (a flat end mill: 0; a cone of half angle a: d / tan a).
#[derive(Clone, Copy, Debug)]
pub struct Cutter {
    pub r: f64,
    /// 1 / tan(half angle) for a cone; 0 for flat.
    pub cone: f64,
}

impl Cutter {
    pub(crate) fn rise(&self, d: f64) -> f64 {
        d * self.cone
    }
}

/// Where the simulation saw something worth a look: a spot (connected cells) with its worst
/// point, how far off it is there, and its area.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct Spot {
    pub at: [f64; 2],
    /// The worst cell's height after cutting.
    pub z: f64,
    /// How far off: deeper than the part (gouge), above it (material left), mm.
    pub by: f64,
    /// Area of the spot, mm².
    pub area: f64,
}

/// An operation's feed time, and the part of it that cut nothing, minutes.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct OpTime {
    pub operation: String,
    pub minutes: f64,
    pub air_minutes: f64,
}

/// A feed move lowering no cell by this much cuts nothing, mm.
pub const AIR_TOL: f64 = 0.005;

/// A rapid that went through material.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct RapidHit {
    pub operation: String,
    pub at: [f64; 3],
    /// How deep into the material, mm.
    pub depth: f64,
}

/// What the simulation found.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct SimReport {
    /// A cell's side, mm.
    pub cell: f64,
    /// Material removed from the sheet, mm³.
    pub removed_volume: f64,
    /// Where the cutters went into the parts by more than 0.02 mm: worst first (at most 50).
    pub gouges: Vec<Spot>,
    pub max_gouge: f64,
    /// Where material stands on the parts by more than 0.05 mm (corners the tool cannot reach,
    /// a countersink's steps...): largest first (at most 50).
    pub left: Vec<Spot>,
    pub max_left: f64,
    /// Area of the parts with material left on them, mm².
    pub left_area: f64,
    /// Rapids (G0) through material (at most 50).
    pub rapid_hits: Vec<RapidHit>,
    /// Time of the feed moves (G1, G2, G3) at their programmed feeds, minutes.
    pub feed_minutes: f64,
    /// Of that, moves that cut nothing (lowered no cell by 0.005 mm): through air.
    pub air_minutes: f64,
    /// The operations with the most time in air, most first (at most 20).
    pub air_by_operation: Vec<OpTime>,
    /// Cuts deeper into the spoilboard than the job allows (its allowance, or a drill's point
    /// past it), deepest first (at most 50).
    pub spoilboard: Vec<Spot>,
    /// Each placed copy (`Bracket #2`) and whether it is cut free all round.
    pub parts: Vec<PartFree>,
    /// Loose pieces cut free inside the parts' through openings (the skin's middle in a
    /// cutout): waste, but held by nothing once separated (largest first, at most 50).
    pub slugs: Vec<Spot>,
}

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct PartFree {
    pub name: String,
    pub free: bool,
    /// Where it is still held to the sheet (tabs, an uncut skin): one spot per place.
    pub held_at: Vec<Spot>,
}

/// The cutter of tool `n` in `job`.
pub(crate) fn cutter(job: &Job, n: u32) -> Result<Cutter> {
    let t = job.tool(n).ok_or_else(|| CamError::Invalid(format!("simulation: no tool T{n}")))?;
    let cone = match t.kind {
        ToolKind::Flat => 0.0,
        ToolKind::Drill | ToolKind::Chamfer => 1.0 / (t.tip_angle().to_radians() / 2.0).tan(),
    };
    Ok(Cutter { r: t.diameter / 2.0, cone })
}

/// Cut `map` along the straight move `a`→`b` with `c`; returns how deep the cut went into
/// material above the cutter (for a rapid's check): the most any cell was lowered.
pub fn sweep(map: &mut HeightMap, c: Cutter, a: [f64; 3], b: [f64; 3]) -> f64 {
    sweep_in(map, c, a, b, 0.0)
}

/// [`sweep`] cutting less: only cells whose every point the cutter covers, `pad` (half a cell's
/// diagonal or more) standing for the cell's reach from its centre.
pub(crate) fn sweep_in(map: &mut HeightMap, c: Cutter, a: [f64; 3], b: [f64; 3], pad: f64) -> f64 {
    let r = c.r - pad;
    if r <= 0.0 {
        return 0.0;
    }
    let Some((i0, i1, j0, j1)) = map.span([a[0].min(b[0]) - r, a[1].min(b[1]) - r], [a[0].max(b[0]) + r, a[1].max(b[1]) + r]) else {
        return 0.0;
    };
    let (ux, uy, uz) = (b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    let uu = ux * ux + uy * uy;
    let mut deepest = 0.0f64;
    for j in j0..=j1 {
        for i in i0..=i1 {
            let p = map.centre(i, j);
            let (px, py) = (p[0] - a[0], p[1] - a[1]);
            // the stretch of the move whose axis lies within r of the cell's centre
            let (t0, t1) = if uu < 1e-18 {
                if px * px + py * py > r * r {
                    continue;
                }
                (0.0, 1.0)
            } else {
                let tc = (px * ux + py * uy) / uu;
                let d2 = (px - tc * ux).powi(2) + (py - tc * uy).powi(2);
                if d2 > r * r {
                    continue;
                }
                let half = ((r * r - d2) / uu).sqrt();
                let (t0, t1) = ((tc - half).max(0.0), (tc + half).min(1.0));
                if t0 > t1 {
                    continue;
                }
                (t0, t1)
            };
            let z_at = |t: f64| {
                let d = (px - t * ux).hypot(py - t * uy);
                a[2] + t * uz + c.rise(d + pad)
            };
            let low = if c.cone == 0.0 {
                // flat: the underside's height runs linearly along the move
                z_at(t0).min(z_at(t1))
            } else {
                // a cone: convex along the move; narrow the stretch down to its lowest point
                let (mut lo, mut hi) = (t0, t1);
                for _ in 0..40 {
                    let (m1, m2) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
                    if z_at(m1) <= z_at(m2) {
                        hi = m2;
                    } else {
                        lo = m1;
                    }
                }
                z_at((lo + hi) / 2.0).min(z_at(t0)).min(z_at(t1))
            };
            if let Some(cell) = map.z.get_mut(j * map.nx + i) {
                let old = f64::from(*cell);
                if low < old {
                    deepest = deepest.max(old - low);
                    *cell = low as f32;
                }
            }
        }
    }
    deepest
}

/// The straight pieces of a move from `from`: an arc split so its chords stay within `sag`.
pub(crate) fn pieces(from: [f64; 3], m: &Move, sag: f64) -> Vec<([f64; 3], [f64; 3])> {
    match m {
        Move::Rapid { to } | Move::Line { to, .. } => vec![(from, *to)],
        Move::Arc { to, center, ccw, .. } => {
            let r = (from[0] - center[0]).hypot(from[1] - center[1]);
            let a0 = (from[1] - center[1]).atan2(from[0] - center[0]);
            let a1 = (to[1] - center[1]).atan2(to[0] - center[0]);
            let tau = std::f64::consts::TAU;
            let mut sweep = if *ccw { (a1 - a0).rem_euclid(tau) } else { -(a0 - a1).rem_euclid(tau) };
            if sweep.abs() < 1e-9 {
                sweep = if *ccw { tau } else { -tau };
            }
            let step = if r > sag { 2.0 * (1.0 - sag / r).clamp(-1.0, 1.0).acos() } else { tau };
            let n = ((sweep.abs() / step.max(1e-3)).ceil() as usize).clamp(1, 100_000);
            let mut out = Vec::with_capacity(n);
            let mut prev = from;
            for k in 1..=n {
                let t = k as f64 / n as f64;
                let a = a0 + sweep * t;
                let p = if k == n { *to } else { [center[0] + r * a.cos(), center[1] + r * a.sin(), from[2] + (to[2] - from[2]) * t] };
                out.push((prev, p));
                prev = p;
            }
            out
        }
        Move::Drill { at, back, bottom, .. } => vec![(from, [at[0], at[1], *back]), ([at[0], at[1], *back], [at[0], at[1], *bottom])],
    }
}

/// Run `ops` over `map`; returns the rapids that went through material.
pub fn run(map: &mut HeightMap, job: &Job, ops: &[Operation]) -> Result<Vec<RapidHit>> {
    Ok(run_with(map, None, job, ops)?.0)
}

/// [`run`], also lowering `allowed` (how deep each cell may be cut) under every drill's point:
/// a drill goes past the bottom by its tip to cut its full diameter there.
fn run_with(map: &mut HeightMap, mut allowed: Option<&mut HeightMap>, job: &Job, ops: &[Operation]) -> Result<(Vec<RapidHit>, Vec<OpTime>)> {
    let sag = map.h / 4.0;
    let mut hits = Vec::new();
    let mut times = Vec::new();
    let mut at = [f64::NAN; 3];
    for op in ops {
        let c = cutter(job, op.tool)?;
        let mut time = OpTime { operation: op.name.clone(), minutes: 0.0, air_minutes: 0.0 };
        for m in &op.moves {
            let start = if at[0].is_nan() {
                // the first move of a program comes down from above
                let e = m.end();
                [e[0], e[1], job.strategy.clearance]
            } else {
                at
            };
            if let (Move::Drill { at: p, back, bottom, .. }, Some(allow)) = (m, allowed.as_deref_mut()) {
                sweep(allow, c, [p[0], p[1], *back], [p[0], p[1], *bottom]);
            }
            let (mut cut, mut length) = (0.0f64, 0.0);
            for (a, b) in pieces(start, m, sag) {
                let depth = sweep(map, c, a, b);
                cut = cut.max(depth);
                length += (0..3).map(|k| (b[k] - a[k]).powi(2)).sum::<f64>().sqrt();
                if matches!(m, Move::Rapid { .. }) && depth > 0.01 && hits.len() < 50 {
                    hits.push(RapidHit { operation: op.name.clone(), at: b, depth });
                }
            }
            if let Move::Line { feed, .. } | Move::Arc { feed, .. } = m
                && *feed > 0.0
            {
                let t = length / feed;
                time.minutes += t;
                if cut < AIR_TOL {
                    time.air_minutes += t;
                }
            }
            at = match m {
                Move::Drill { at: p, back, .. } => [p[0], p[1], *back],
                _ => m.end(),
            };
        }
        times.push(time);
    }
    Ok((hits, times))
}

/// The highest point of `tris` over each cell inside them; NaN elsewhere.
fn rasterize(map: &HeightMap, tris: &[[Vec3; 3]], out: &mut [f32]) {
    for t in tris {
        let (a, b, c) = (t[0], t[1], t[2]);
        let det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
        if det.abs() < 1e-12 {
            continue; // seen edge on: a wall
        }
        let lo = [a.x.min(b.x).min(c.x), a.y.min(b.y).min(c.y)];
        let hi = [a.x.max(b.x).max(c.x), a.y.max(b.y).max(c.y)];
        let Some((i0, i1, j0, j1)) = map.span(lo, hi) else { continue };
        for j in j0..=j1 {
            for i in i0..=i1 {
                let p = map.centre(i, j);
                let l1 = ((b.y - c.y) * (p[0] - c.x) + (c.x - b.x) * (p[1] - c.y)) / ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
                let l2 = ((c.y - a.y) * (p[0] - c.x) + (a.x - c.x) * (p[1] - c.y)) / ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
                let l3 = 1.0 - l1 - l2;
                if l1 < -1e-9 || l2 < -1e-9 || l3 < -1e-9 {
                    continue;
                }
                let z = (l1 * a.z + l2 * b.z + l3 * c.z) as f32;
                if let Some(o) = out.get_mut(j * map.nx + i)
                    && (o.is_nan() || z > *o)
                {
                    *o = z;
                }
            }
        }
    }
}

/// Connected spots (8 neighbours) of the cells `on` marks, each with its worst cell by `by`.
fn spots(map: &HeightMap, on: &[bool], by: &[f32], limit: usize) -> Vec<Spot> {
    let mut seen = vec![false; on.len()];
    let mut out: Vec<Spot> = Vec::new();
    let (nx, ny) = (map.nx, map.ny);
    for start in 0..on.len() {
        if !on.get(start).copied().unwrap_or(false) || seen.get(start).copied().unwrap_or(true) {
            continue;
        }
        let mut stack = vec![start];
        if let Some(s) = seen.get_mut(start) {
            *s = true;
        }
        let (mut worst, mut worst_at, mut n) = (f32::NEG_INFINITY, start, 0usize);
        while let Some(k) = stack.pop() {
            n += 1;
            if let Some(&v) = by.get(k)
                && v > worst
            {
                worst = v;
                worst_at = k;
            }
            let (i, j) = ((k % nx) as i64, (k / nx) as i64);
            for dj in -1..=1i64 {
                for di in -1..=1i64 {
                    let (a, b) = (i + di, j + dj);
                    if a < 0 || b < 0 || a >= nx as i64 || b >= ny as i64 {
                        continue;
                    }
                    let q = b as usize * nx + a as usize;
                    if on.get(q).copied().unwrap_or(false) && !seen.get(q).copied().unwrap_or(true) {
                        if let Some(s) = seen.get_mut(q) {
                            *s = true;
                        }
                        stack.push(q);
                    }
                }
            }
        }
        let (i, j) = (worst_at % nx, worst_at / nx);
        out.push(Spot { at: map.centre(i, j), z: map.at(i, j), by: f64::from(worst), area: n as f64 * map.h * map.h });
    }
    out.sort_by(|a, b| b.by.total_cmp(&a.by));
    out.truncate(limit);
    out
}

/// Simulate a planned job on a map of `cell`-sized cells and compare the result with its parts
/// (`bodies` as designed, in `job.parts` order). Returns the report and the map.
pub fn simulate(job: &Job, bodies: &[Body], plan: &JobPlan, cell: f64) -> Result<(SimReport, HeightMap)> {
    let [w, d] = job.stock.size;
    let t = job.stock.thickness;
    let zero = job.stock.work_zero();
    let mut map = HeightMap::new([-zero[0], -zero[1]], [w - zero[0], d - zero[1]], 0.0, cell)?;
    let floor = -t - job.stock.spoilboard_allowance;
    let mut allowed = HeightMap::new([-zero[0], -zero[1]], [w - zero[0], d - zero[1]], floor, cell)?;
    let (rapid_hits, times) = run_with(&mut map, Some(&mut allowed), job, &plan.ops)?;
    let feed_minutes = times.iter().map(|t| t.minutes).sum();
    let air_minutes = times.iter().map(|t| t.air_minutes).sum();
    let mut air_by_operation = times;
    air_by_operation.sort_by(|a, b| b.air_minutes.total_cmp(&a.air_minutes));
    air_by_operation.truncate(20);
    let n = map.nx * map.ny;
    let removed_volume: f64 = map.z.iter().map(|z| (0.0 - f64::from(*z).max(-t)).max(0.0)).sum::<f64>() * cell * cell;

    // the parts as designed: the top of each over each cell; through openings the sheet's bottom
    let copies = crate::scene::placed_copies(job, bodies, plan, TARGET_TOL)?;
    let outlines = plan.placed_outlines();
    let mut target = vec![f32::NAN; n];
    let mut owner: Vec<Option<usize>> = vec![None; n];
    for (k, (c, outline)) in copies.iter().zip(&outlines).enumerate() {
        let mut mine = vec![f32::NAN; n];
        rasterize(&map, &c.tris, &mut mine);
        let Some((lo, hi)) = outline.bounds() else { continue };
        let Some((i0, i1, j0, j1)) = map.span(lo, hi) else { continue };
        for j in j0..=j1 {
            for i in i0..=i1 {
                let q = j * map.nx + i;
                if !outline.contains(map.centre(i, j)) {
                    continue;
                }
                if let (Some(tg), Some(o), Some(m)) = (target.get_mut(q), owner.get_mut(q), mine.get(q)) {
                    *tg = if m.is_nan() { -t as f32 } else { *m };
                    *o = Some(k);
                }
            }
        }
    }
    // neighbourhood bounds of the target (a wall jumps within a cell)
    let near = |q: usize, f: fn(f32, f32) -> f32| {
        let (i, j) = ((q % map.nx) as i64, (q / map.nx) as i64);
        let mut v = target.get(q).copied().unwrap_or(f32::NAN);
        for dj in -1..=1i64 {
            for di in -1..=1i64 {
                let (a, b) = (i + di, j + dj);
                if a < 0 || b < 0 || a >= map.nx as i64 || b >= map.ny as i64 {
                    continue;
                }
                let x = target.get(b as usize * map.nx + a as usize).copied().unwrap_or(f32::NAN);
                // beside the part counts as cut through
                let x = if x.is_nan() { -t as f32 } else { x };
                v = if v.is_nan() { x } else { f(v, x) };
            }
        }
        v
    };
    let mut gouge = vec![false; n];
    let mut gouge_by = vec![0f32; n];
    let mut left = vec![false; n];
    let mut left_by = vec![0f32; n];
    let mut spoil = vec![false; n];
    let mut spoil_by = vec![0f32; n];
    for q in 0..n {
        let z = map.z.get(q).copied().unwrap_or(0.0);
        let deepest = allowed.z.get(q).copied().map_or(floor, f64::from);
        if f64::from(z) < deepest - 0.01
            && let (Some(s), Some(b)) = (spoil.get_mut(q), spoil_by.get_mut(q))
        {
            *s = true;
            *b = (deepest - f64::from(z)) as f32;
        }
        if target.get(q).is_none_or(|v| v.is_nan()) {
            continue;
        }
        let (lowest, highest) = (near(q, f32::min), near(q, f32::max));
        // where the part goes through (or a wall of an opening is a cell away), how deep is the
        // spoilboard's question, not the part's
        if lowest > -t as f32 + 1e-6
            && z < lowest - GOUGE_TOL as f32
            && let (Some(g), Some(b)) = (gouge.get_mut(q), gouge_by.get_mut(q))
        {
            *g = true;
            *b = lowest - z;
        }
        if z > highest + LEFT_TOL as f32
            && let (Some(l), Some(b)) = (left.get_mut(q), left_by.get_mut(q))
        {
            *l = true;
            *b = z - highest;
        }
    }
    // loose pieces: material (above the sheet's bottom) cut off from everything else, lying
    // where the parts go through: waste dropped out of an opening, not material left on a part
    let mut slug = vec![false; n];
    let mut slug_by = vec![0f32; n];
    {
        let solid = |q: usize| map.z.get(q).is_some_and(|z| f64::from(*z) > -t + 1e-4);
        let mut seen = vec![false; n];
        for start in 0..n {
            if !solid(start) || seen.get(start).copied().unwrap_or(true) {
                continue;
            }
            let mut piece = vec![start];
            if let Some(s) = seen.get_mut(start) {
                *s = true;
            }
            let mut k = 0;
            let mut through = true;
            while let Some(&q) = piece.get(k) {
                k += 1;
                through &= target.get(q).is_some_and(|v| f64::from(*v) <= -t + 1e-6);
                let (i, j) = (q % map.nx, q / map.nx);
                let next =
                    [(i > 0).then(|| q - 1), (i + 1 < map.nx).then(|| q + 1), (j > 0).then(|| q - map.nx), (j + 1 < map.ny).then(|| q + map.nx)];
                for r in next.into_iter().flatten() {
                    if solid(r) && !seen.get(r).copied().unwrap_or(true) {
                        if let Some(s) = seen.get_mut(r) {
                            *s = true;
                        }
                        piece.push(r);
                    }
                }
            }
            if through {
                for q in piece {
                    if let (Some(s), Some(b), Some(l)) = (slug.get_mut(q), slug_by.get_mut(q), left.get_mut(q)) {
                        *s = true;
                        *b = (f64::from(map.z.get(q).copied().unwrap_or(0.0)) + t) as f32;
                        *l = false;
                    }
                }
            }
        }
    }
    let slugs = spots(&map, &slug, &slug_by, 50);
    let gouges = spots(&map, &gouge, &gouge_by, 50);
    let left_spots = spots(&map, &left, &left_by, 50);
    let left_area = left.iter().filter(|l| **l).count() as f64 * cell * cell;

    // cut free: every cell just outside each outline is cut through the sheet
    let mut parts = Vec::new();
    for (k, c) in copies.iter().enumerate() {
        let name = format!("{} #{}", job.parts.get(c.part).map_or("?", |p| p.label()), c.copy + 1);
        let Some(outline) = outlines.get(k) else { continue };
        let ring = outline.offset(2.0 * cell, 0.01)?.difference(&outline.offset(0.5 * cell, 0.01)?)?;
        let mut held = vec![false; n];
        let mut held_by = vec![0f32; n];
        if let Some((lo, hi)) = ring.bounds()
            && let Some((i0, i1, j0, j1)) = map.span(lo, hi)
        {
            for j in j0..=j1 {
                for i in i0..=i1 {
                    let q = j * map.nx + i;
                    let z = map.at(i, j);
                    if z > -t + 1e-4
                        && ring.contains(map.centre(i, j))
                        && owner.get(q).is_none_or(|o| o.is_none())
                        && let (Some(h), Some(b)) = (held.get_mut(q), held_by.get_mut(q))
                    {
                        *h = true;
                        *b = (z + t) as f32;
                    }
                }
            }
        }
        let held_at = spots(&map, &held, &held_by, 50);
        parts.push(PartFree { name, free: held_at.is_empty(), held_at });
    }
    let report = SimReport {
        cell,
        removed_volume,
        max_gouge: gouges.first().map_or(0.0, |s| s.by),
        gouges,
        max_left: left_spots.first().map_or(0.0, |s| s.by),
        left: left_spots,
        left_area,
        rapid_hits,
        feed_minutes,
        air_minutes,
        air_by_operation,
        spoilboard: spots(&map, &spoil, &spoil_by, 50),
        parts,
        slugs,
    };
    Ok((report, map))
}

/// The map as a shaded PNG, at most `max_px` wide: light where the sheet's top is left, darker
/// with depth, the through cuts darkest; lit from the back left. With a report, gouges in red,
/// material left in orange and held places in magenta.
pub fn png(map: &HeightMap, thickness: f64, report: Option<&SimReport>, max_px: u32) -> Result<Vec<u8>> {
    let step = ((map.nx as f64 / f64::from(max_px.max(64))).ceil() as usize).max(1);
    let (w, hgt) = (map.nx.div_ceil(step), map.ny.div_ceil(step));
    let mut rgba = vec![0u8; w * hgt * 4];
    let z = |i: usize, j: usize| map.at(i.min(map.nx - 1), j.min(map.ny - 1));
    for y in 0..hgt {
        for x in 0..w {
            let (i, j) = (x * step, (hgt - 1 - y) * step);
            let v = z(i, j);
            let depth = ((-v) / thickness.max(1e-6)).clamp(0.0, 1.2);
            // light from the back left: the slope towards it brightens
            let gx = z(i + step, j) - z(i.saturating_sub(step), j);
            let gy = z(i, j + step) - z(i, j.saturating_sub(step));
            let shade = (1.0 + (-gx + gy) / (2.0 * map.h * step as f64) * 0.35).clamp(0.55, 1.25);
            let base = [235.0 - 150.0 * depth, 238.0 - 120.0 * depth, 242.0 - 60.0 * depth];
            let px = rgba.get_mut((y * w + x) * 4..(y * w + x) * 4 + 4);
            if let Some(px) = px {
                for k in 0..3 {
                    px[k] = (base[k] * shade).clamp(0.0, 255.0) as u8;
                }
                px[3] = 255;
            }
        }
    }
    if let Some(r) = report {
        let mut mark = |s: &Spot, col: [u8; 3]| {
            let rad = ((s.area.sqrt() / map.h / step as f64) / 2.0 + 3.0).min(40.0) as i64;
            let cx = ((s.at[0] - map.x0) / map.h / step as f64) as i64;
            let cy = hgt as i64 - 1 - ((s.at[1] - map.y0) / map.h / step as f64) as i64;
            for dy in -rad..=rad {
                for dx in -rad..=rad {
                    let d2 = dx * dx + dy * dy;
                    if d2 > rad * rad || d2 < (rad - 2).max(0).pow(2) {
                        continue;
                    }
                    let (x, y) = (cx + dx, cy + dy);
                    if x >= 0
                        && y >= 0
                        && (x as usize) < w
                        && (y as usize) < hgt
                        && let Some(px) = rgba.get_mut((y as usize * w + x as usize) * 4..(y as usize * w + x as usize) * 4 + 3)
                    {
                        px.copy_from_slice(&col);
                    }
                }
            }
        };
        for s in &r.left {
            mark(s, [245, 124, 0]);
        }
        for p in &r.parts {
            for s in &p.held_at {
                mark(s, [200, 0, 200]);
            }
        }
        for s in r.gouges.iter().chain(&r.spoilboard) {
            mark(s, [220, 0, 0]);
        }
    }
    let mut out = Vec::new();
    {
        let mut enc = png::Encoder::new(&mut out, w as u32, hgt as u32);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        let mut wr = enc.write_header().map_err(|e| CamError::Invalid(format!("png: {e}")))?;
        wr.write_image_data(&rgba).map_err(|e| CamError::Invalid(format!("png: {e}")))?;
    }
    Ok(out)
}
