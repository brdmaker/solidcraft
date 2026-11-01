//! Adaptive clearing: the cleared area grows from a helix entry at the most open point of each
//! pocket, one stepover at a time, along the front of uncut material, so the cutter never takes
//! more than the stepover from the side.
//!
//! Each pass is the boundary of the tool centres' region grown by the step and clipped to where
//! the centre may go, less the stretches earlier passes already ran (along walls). Before a pass
//! is cut, the cutter's engagement (the arc of its edge in uncut material) is measured at every
//! point of it on a grid of what is already cleared; where it would pass the limit, as where the
//! front wraps into a corner or round an island, the pass is planned again at half the step, down
//! to a sixteenth. Within a step of the region's edge, where a front meets a wall and the material
//! beside the last pass lies on a quarter of the cutter whatever the step, the limit is that
//! quarter and a straight cut's engagement: roughing leaves the walls rough for the wall passes
//! rather than crawl along them. Moves between stretches stay at depth through cleared area, else lift just above
//! the level and re-enter over cleared ground.

use crate::job::Tool;
use crate::region::{Area, Ring};
use crate::toolpath::PathBuilder;
use crate::{CamError, Result};

/// Directions the engagement is sampled in round the cutter's edge.
const RAYS: usize = 192;

/// The most grid cells of the cleared map (made only where cut); past it the cells grow.
const MAX_CELLS: usize = 400_000_000;

/// The most passes one pocket level may take.
const MAX_PASSES: usize = 50_000;

#[derive(Clone, Copy, Debug)]
pub struct AdaptiveParams {
    /// Sideways step between passes (the radial engagement on a straight cut), mm.
    pub step: f64,
    /// The most engagement allowed, radians.
    pub max_engagement: f64,
    /// Cut stretches either way (else always climbing).
    pub both_ways: bool,
    /// Ramp angle of the helix entry, radians.
    pub ramp: f64,
    /// Height above a level's material top that links lift to.
    pub retract: f64,
    /// The machine's rapid traverse speed, mm/min: links take the quickest way.
    pub rapid: f64,
    /// The most chip thinning raises the feed, times the tool's feed (1: the tool's feed).
    pub max_thinning: f64,
    /// The feed in a full slot, times the tool's feed.
    pub slotting: f64,
    /// The feed through cleared ground between passes, mm/min.
    pub link: f64,
}

impl AdaptiveParams {
    /// The feed factor for a cut engaging `e` (radians) of the cutter's edge: the chip is as
    /// thick as the tool's chip load where 1 / sin(e) is applied below 90°, up to
    /// `max_thinning`, in steps of 0.1 (rounded down); the tool's feed from 90° to 170°; the
    /// slotting factor past it.
    pub fn feed_factor(&self, e: f64) -> f64 {
        if e > 170f64.to_radians() {
            self.slotting
        } else if e >= std::f64::consts::FRAC_PI_2 || self.max_thinning <= 1.0 {
            1.0
        } else {
            let f = (1.0 / e.sin().max(1e-6)).min(self.max_thinning);
            ((f * 10.0 + 1e-9).floor() / 10.0).max(1.0)
        }
    }

    /// The engagement of a straight cut `step` deep into the side of a cutter of radius `rt`:
    /// the arc of its edge, from straight ahead round to where the uncut material ends,
    /// acos(1 − step/rt) (180° in a full slot).
    pub fn straight_engagement(step: f64, rt: f64) -> f64 {
        (1.0 - (step / rt).clamp(0.0, 2.0)).clamp(-1.0, 1.0).acos()
    }
}

/// What has been cut, on a grid kept in blocks of 64 × 64 cells, made only where something is
/// cut: a band round the parts of a whole sheet costs what the band covers, not the sheet.
pub struct Cleared {
    x0: f64,
    y0: f64,
    h: f64,
    nx: usize,
    ny: usize,
    /// blocks across
    bx: usize,
    blocks: Vec<Option<Box<[bool]>>>,
}

/// A block's side, in cells.
const BLOCK: usize = 64;

impl Cleared {
    /// An uncut grid over `lo`..`hi` with cells about `h` wide (wider if there would be too many).
    pub fn new(lo: [f64; 2], hi: [f64; 2], h: f64) -> Result<Cleared> {
        let (w, d) = (hi[0] - lo[0], hi[1] - lo[1]);
        if !(w.is_finite() && d.is_finite() && w >= 0.0 && d >= 0.0 && h > 0.0) {
            return Err(CamError::Geometry("cleared map: bad bounds".into()));
        }
        let mut h = h;
        while ((w / h).ceil() + 1.0) * ((d / h).ceil() + 1.0) > MAX_CELLS as f64 {
            h *= 1.5;
        }
        let (nx, ny) = ((w / h).ceil() as usize + 1, (d / h).ceil() as usize + 1);
        let bx = nx.div_ceil(BLOCK);
        let by = ny.div_ceil(BLOCK);
        Ok(Cleared { x0: lo[0], y0: lo[1], h, nx, ny, bx, blocks: vec![None; bx * by] })
    }

    fn get(&self, i: usize, j: usize) -> bool {
        self.blocks
            .get((j / BLOCK) * self.bx + i / BLOCK)
            .and_then(|b| b.as_ref())
            .and_then(|b| b.get((j % BLOCK) * BLOCK + i % BLOCK))
            .copied()
            .unwrap_or(false)
    }

    fn set(&mut self, i: usize, j: usize) {
        if i >= self.nx || j >= self.ny {
            return;
        }
        if let Some(b) = self.blocks.get_mut((j / BLOCK) * self.bx + i / BLOCK)
            && let Some(cell) = b.get_or_insert_with(|| vec![false; BLOCK * BLOCK].into_boxed_slice()).get_mut((j % BLOCK) * BLOCK + i % BLOCK)
        {
            *cell = true;
        }
    }

    /// Is the point cut? Off the grid counts as uncut.
    pub fn is_cut(&self, p: [f64; 2]) -> bool {
        let (i, j) = ((p[0] - self.x0) / self.h, (p[1] - self.y0) / self.h);
        if !(i >= 0.0 && j >= 0.0) {
            return false;
        }
        let (i, j) = (i as usize, j as usize);
        i < self.nx && j < self.ny && self.get(i, j)
    }

    /// Mark the disc of radius `r` about `c` as cut.
    pub fn stamp(&mut self, c: [f64; 2], r: f64) {
        let lo_i = (((c[0] - r - self.x0) / self.h).floor().max(0.0)) as usize;
        let hi_i = (((c[0] + r - self.x0) / self.h).ceil().max(0.0) as usize).min(self.nx.saturating_sub(1));
        let lo_j = (((c[1] - r - self.y0) / self.h).floor().max(0.0)) as usize;
        let hi_j = (((c[1] + r - self.y0) / self.h).ceil().max(0.0) as usize).min(self.ny.saturating_sub(1));
        for j in lo_j..=hi_j {
            for i in lo_i..=hi_i {
                let (x, y) = (self.x0 + (i as f64 + 0.5) * self.h, self.y0 + (j as f64 + 0.5) * self.h);
                if (x - c[0]).hypot(y - c[1]) <= r {
                    self.set(i, j);
                }
            }
        }
    }

    /// Mark everything a cutter of radius `r` sweeps along the polyline as cut: each straight
    /// piece as one capsule (the cells within `r` of the piece).
    pub fn sweep(&mut self, pts: &[[f64; 2]], r: f64) {
        for w in pts.windows(2) {
            let (Some(a), Some(b)) = (w.first(), w.get(1)) else { continue };
            self.capsule(*a, *b, r);
        }
        if let [p] = pts {
            self.capsule(*p, *p, r);
        }
    }

    fn capsule(&mut self, a: [f64; 2], b: [f64; 2], r: f64) {
        let lo = [a[0].min(b[0]) - r, a[1].min(b[1]) - r];
        let hi = [a[0].max(b[0]) + r, a[1].max(b[1]) + r];
        let i0 = ((lo[0] - self.x0) / self.h).floor().max(0.0) as usize;
        let j0 = ((lo[1] - self.y0) / self.h).floor().max(0.0) as usize;
        let i1 = (((hi[0] - self.x0) / self.h).ceil().max(0.0) as usize).min(self.nx.saturating_sub(1));
        let j1 = (((hi[1] - self.y0) / self.h).ceil().max(0.0) as usize).min(self.ny.saturating_sub(1));
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l2 = dx * dx + dy * dy;
        for j in j0..=j1 {
            let y = self.y0 + (j as f64 + 0.5) * self.h;
            for i in i0..=i1 {
                let x = self.x0 + (i as f64 + 0.5) * self.h;
                let t = if l2 > 0.0 { (((x - a[0]) * dx + (y - a[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
                if (x - a[0] - t * dx).hypot(y - a[1] - t * dy) <= r {
                    self.set(i, j);
                }
            }
        }
    }

    /// The cutter's engagement at `p` travelling along `dir` (a unit vector): the arc of the
    /// forward half of its edge (radius `r`) over uncut material, radians (π in a full slot).
    pub fn engagement(&self, p: [f64; 2], dir: [f64; 2], r: f64) -> f64 {
        // just inside the edge, and outside where the replay in the tests measures (0.98): the
        // planner errs on the safe side
        let rr = r * 0.99;
        let uncut = (0..RAYS)
            .filter(|k| {
                let a = *k as f64 * std::f64::consts::TAU / RAYS as f64;
                let (c, s) = (a.cos(), a.sin());
                c * dir[0] + s * dir[1] > 0.0 && !self.is_cut([p[0] + rr * c, p[1] + rr * s])
            })
            .count();
        uncut as f64 * std::f64::consts::TAU / RAYS as f64
    }
}

/// The points of `segs` where cutting them would engage more than `limit` (radians), or more
/// than `wall_limit` within the tolerance of `walls` (the region's edge), thinned to one every
/// half millimetre or so.
fn hot_points(segs: &[Vec<[f64; 2]>], cleared: &Cleared, rt: f64, limit: f64, walls: &EdgeGrid, wall_limit: f64) -> Vec<[f64; 2]> {
    let mut hot: Vec<[f64; 2]> = Vec::new();
    for w in segs.iter().flat_map(|sg| sg.windows(2)) {
        let (Some(a), Some(b)) = (w.first(), w.get(1)) else { continue };
        let len = (b[0] - a[0]).hypot(b[1] - a[1]);
        if len <= 1e-9 {
            continue;
        }
        let dir = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
        // every 0.1 mm: where a stretch leaves a wall the engagement peaks within a fraction of
        // a millimetre
        let n = (len / 0.1).ceil().clamp(1.0, 10_000.0) as usize;
        for k in 1..=n {
            let t = k as f64 / n as f64;
            let q = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
            let e = cleared.engagement(q, dir, rt);
            if e > limit
                && (e > wall_limit || !walls.near(q))
                && hot.last().is_none_or(|h| (h[0] - q[0]).hypot(h[1] - q[1]) > 0.5)
                && hot.len() < 5_000
            {
                hot.push(q);
            }
        }
    }
    hot
}

/// Cut along `path` (from its first point, where the tool is) at `z`, each piece at the tool's
/// feed times the factor (`AdaptiveParams::feed_factor`) for the most engagement measured every
/// 0.1 mm along it and the two pieces either side (the feed changes before the load does), at
/// most the rapid feed.
fn cut(b: &mut PathBuilder, cleared: &Cleared, path: &[[f64; 2]], z: f64, t: &Tool, p: &AdaptiveParams) {
    let rt = t.diameter / 2.0;
    let most: Vec<f64> = path
        .windows(2)
        .map(|w| {
            let (Some(a), Some(q)) = (w.first(), w.get(1)) else { return std::f64::consts::PI };
            let len = (q[0] - a[0]).hypot(q[1] - a[1]);
            if len <= 1e-9 {
                return 0.0;
            }
            let dir = [(q[0] - a[0]) / len, (q[1] - a[1]) / len];
            let n = (len / 0.1).ceil().clamp(1.0, 10_000.0) as usize;
            (0..=n)
                .map(|k| {
                    let s = k as f64 / n as f64;
                    cleared.engagement([a[0] + (q[0] - a[0]) * s, a[1] + (q[1] - a[1]) * s], dir, rt)
                })
                .fold(0.0, f64::max)
        })
        .collect();
    for (i, q) in path.iter().skip(1).enumerate() {
        let e = most.get(i.saturating_sub(2)..(i + 3).min(most.len())).map_or(std::f64::consts::PI, |w| w.iter().copied().fold(0.0, f64::max));
        let feed = if p.max_thinning > 1.0 || p.slotting < 1.0 { t.feed * p.feed_factor(e) } else { t.feed };
        b.line([q[0], q[1], z], feed.min(p.rapid.max(t.feed)));
    }
}

/// Edges no longer than `max`, ring closed (first point repeated last).
fn densify(r: &Ring, max: f64) -> Vec<[f64; 2]> {
    let mut out = Vec::new();
    let n = r.len();
    for i in 0..n {
        let (Some(a), Some(b)) = (r.get(i), r.get((i + 1) % n)) else { continue };
        let k = ((a[0] - b[0]).hypot(a[1] - b[1]) / max).ceil().clamp(1.0, 1e5) as usize;
        for j in 0..k {
            let t = j as f64 / k as f64;
            out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
        }
    }
    if let Some(&f) = out.first() {
        out.push(f);
    }
    out
}

/// The most open point of `shape` and roughly how far it is from the edge: shrink it until
/// nothing is left and take the middle of what was left last.
fn most_open(shape: &Area, step: f64) -> Result<([f64; 2], f64)> {
    let mut last = shape.clone();
    let mut depth = 0.0;
    let mut a = shape.clone();
    for _ in 0..10_000 {
        let next = a.offset(-step, 0.01)?;
        if next.is_empty() {
            break;
        }
        depth += step;
        last = next.clone();
        a = next;
    }
    let ring = last.rings().next().cloned().unwrap_or_default();
    let n = ring.len().max(1) as f64;
    let mid = ring.iter().fold([0.0, 0.0], |s, p| [s[0] + p[0] / n, s[1] + p[1] / n]);
    let p = if last.contains(mid) { mid } else { ring.first().copied().unwrap_or(mid) };
    Ok((p, depth))
}

/// An area's edges by grid cell, to tell quickly whether a point lies within `tol` of them.
struct EdgeGrid {
    x0: f64,
    y0: f64,
    cell: f64,
    nx: usize,
    ny: usize,
    tol: f64,
    edges: Vec<([f64; 2], [f64; 2])>,
    cells: Vec<Vec<usize>>,
}

impl EdgeGrid {
    fn new(area: &Area, tol: f64) -> EdgeGrid {
        let edges: Vec<([f64; 2], [f64; 2])> = area.rings().flat_map(crate::region::ring_edges).collect();
        let (lo, hi) = area.bounds().unwrap_or(([0.0, 0.0], [0.0, 0.0]));
        // about as many cells as edges, at most 1000 a side
        let span = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1e-3);
        let cell = (span / (edges.len() as f64).sqrt().clamp(1.0, 1000.0)).max(tol * 4.0).max((span + 4.0 * tol) / 999.0).max(1e-3);
        let (x0, y0) = (lo[0] - 2.0 * tol, lo[1] - 2.0 * tol);
        let nx = (((hi[0] - lo[0] + 4.0 * tol) / cell).ceil() as usize).clamp(1, 1000);
        let ny = (((hi[1] - lo[1] + 4.0 * tol) / cell).ceil() as usize).clamp(1, 1000);
        let mut g = EdgeGrid { x0, y0, cell, nx, ny, tol, edges, cells: vec![Vec::new(); nx * ny] };
        for (k, (a, b)) in g.edges.iter().enumerate() {
            let (i0, j0) = g.index([a[0].min(b[0]) - tol, a[1].min(b[1]) - tol]);
            let (i1, j1) = g.index([a[0].max(b[0]) + tol, a[1].max(b[1]) + tol]);
            for i in i0..=i1 {
                for j in j0..=j1 {
                    if let Some(c) = g.cells.get_mut(i * ny + j) {
                        c.push(k);
                    }
                }
            }
        }
        g
    }

    fn index(&self, p: [f64; 2]) -> (usize, usize) {
        let i = ((p[0] - self.x0) / self.cell).floor().clamp(0.0, (self.nx - 1) as f64) as usize;
        let j = ((p[1] - self.y0) / self.cell).floor().clamp(0.0, (self.ny - 1) as f64) as usize;
        (i, j)
    }

    /// Is `p` within `tol` of an edge?
    fn near(&self, p: [f64; 2]) -> bool {
        let (i, j) = self.index(p);
        self.cells.get(i * self.ny + j).is_some_and(|c| {
            c.iter().filter_map(|k| self.edges.get(*k)).any(|(a, b)| {
                let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                let l2 = dx * dx + dy * dy;
                let t = if l2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
                (a[0] + t * dx - p[0]).hypot(a[1] + t * dy - p[1]) <= self.tol
            })
        })
    }
}

/// What the growth inside a window offers.
enum Pick {
    /// The piece nearest the tool, with the stretches of front that bound it.
    Piece(Area, Vec<Vec<[f64; 2]>>),
    /// A piece reaching the window's edge (it may go on outside), or none nearer than the
    /// edge: look again over a wider window.
    Wider,
    /// Nothing left to grow into.
    Done,
}

/// The piece of the growth from `visited` to `next` nearest `here` (by the start of one of its
/// stretches, or either end cutting both ways) with the stretches of front that bound it.
///
/// `window` is where `next` was found, `None` for the whole region: a piece reaching within
/// a hair of its edge may be cut short there, and one outside could be nearer than any inside
/// when the nearest inside is farther than the edge.
#[allow(clippy::too_many_arguments)]
fn nearest_piece(
    next: &Area,
    visited: &Area,
    here: [f64; 2],
    tol: f64,
    spacing: f64,
    both_ways: bool,
    window: Option<([f64; 2], [f64; 2])>,
) -> Result<Pick> {
    let growth = next.difference(visited)?;
    let near_visited = EdgeGrid::new(visited, tol);
    let d = |q: [f64; 2], r: [f64; 2]| (q[0] - r[0]).hypot(q[1] - r[1]);
    let reach = |sg: &Vec<[f64; 2]>| {
        let first = sg.first().map_or(f64::INFINITY, |q| d(*q, here));
        let last = if both_ways { sg.last().map_or(f64::INFINITY, |q| d(*q, here)) } else { f64::INFINITY };
        first.min(last)
    };
    let edge = |lo: [f64; 2], hi: [f64; 2]| match window {
        None => false,
        Some((wlo, whi)) => {
            let m = 4.0 * tol + 0.01;
            lo[0] < wlo[0] + m || lo[1] < wlo[1] + m || hi[0] > whi[0] - m || hi[1] > whi[1] - m
        }
    };
    let border = window.map_or(f64::INFINITY, |(wlo, whi)| (here[0] - wlo[0]).min(here[1] - wlo[1]).min(whi[0] - here[0]).min(whi[1] - here[1]));
    let mut best: Option<(f64, Area, Vec<Vec<[f64; 2]>>)> = None;
    let mut cut_short = f64::INFINITY;
    for sh in &growth.shapes {
        // a piece's front: its outline where it doesn't run along the visited region's edge
        let piece = Area { shapes: vec![sh.clone()] };
        let segs = fronts(&piece, &near_visited, spacing);
        if segs.is_empty() {
            continue;
        }
        let r = segs.iter().map(reach).fold(f64::INFINITY, f64::min);
        let Some((lo, hi)) = piece.bounds() else { continue };
        if edge(lo, hi) {
            cut_short = cut_short.min(r);
        } else if best.as_ref().is_none_or(|(b, ..)| r < *b) {
            best = Some((r, piece, segs));
        }
    }
    Ok(match best {
        Some((r, piece, segs)) if r <= cut_short && r <= border => Pick::Piece(piece, segs),
        _ if window.is_some() => Pick::Wider,
        Some((_, piece, segs)) => Pick::Piece(piece, segs),
        None => Pick::Done,
    })
}

/// An area kept in square tiles: a window of it is quick to read, and adding a small area
/// changes only the tiles under it.
struct Tiled {
    x0: f64,
    y0: f64,
    size: f64,
    nx: usize,
    ny: usize,
    tiles: Vec<Area>,
}

impl Tiled {
    /// Empty tiles about `size` wide over `lo`..`hi` (wider if there would be more than 200 a
    /// side); the outermost reach on outwards.
    fn new(lo: [f64; 2], hi: [f64; 2], size: f64) -> Tiled {
        let span = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1e-3);
        let size = size.max(span / 200.0).max(1e-3);
        let nx = (((hi[0] - lo[0]) / size).ceil() as usize).clamp(1, 200);
        let ny = (((hi[1] - lo[1]) / size).ceil() as usize).clamp(1, 200);
        Tiled { x0: lo[0], y0: lo[1], size, nx, ny, tiles: vec![Area::empty(); nx * ny] }
    }

    /// The tiles over `lo`..`hi`: first and last across, first and last up.
    fn span(&self, lo: [f64; 2], hi: [f64; 2]) -> (usize, usize, usize, usize) {
        let f = |v: f64, o: f64, n: usize| (((v - o) / self.size).floor().max(0.0) as usize).min(n - 1);
        (f(lo[0], self.x0, self.nx), f(hi[0], self.x0, self.nx), f(lo[1], self.y0, self.ny), f(hi[1], self.y0, self.ny))
    }

    fn rect(&self, i: usize, j: usize) -> Result<Area> {
        // the outermost tiles reach a metre on, for whatever lies past the bounds
        let far = 1000.0;
        let x0 = if i == 0 { self.x0 - far } else { self.x0 + i as f64 * self.size };
        let x1 = if i + 1 == self.nx { self.x0 + self.nx as f64 * self.size + far } else { self.x0 + (i + 1) as f64 * self.size };
        let y0 = if j == 0 { self.y0 - far } else { self.y0 + j as f64 * self.size };
        let y1 = if j + 1 == self.ny { self.y0 + self.ny as f64 * self.size + far } else { self.y0 + (j + 1) as f64 * self.size };
        Area::rect(x0, y0, x1, y1)
    }

    fn add(&mut self, a: &Area) -> Result<()> {
        let Some((lo, hi)) = a.bounds() else { return Ok(()) };
        let (i0, i1, j0, j1) = self.span(lo, hi);
        for j in j0..=j1 {
            for i in i0..=i1 {
                let part = a.intersection(&self.rect(i, j)?)?;
                if part.is_empty() {
                    continue;
                }
                if let Some(t) = self.tiles.get_mut(j * self.nx + i) {
                    *t = if t.is_empty() { part } else { t.union(&part)? };
                }
            }
        }
        Ok(())
    }

    /// Add `a`, then close each tile it touched (fill gaps narrower than 6 µm, corners kept
    /// sharp) and thin it out (within 2 µm): the hairline gaps between neighbouring pieces go,
    /// and the chords of many pieces along a curved wall become one outline. Read back with
    /// [`Tiled::window_closed`], which closes the seams thinning leaves between tiles.
    fn add_closed(&mut self, a: &Area) -> Result<()> {
        self.add(a)?;
        let Some((lo, hi)) = a.bounds() else { return Ok(()) };
        let (i0, i1, j0, j1) = self.span(lo, hi);
        for j in j0..=j1 {
            for i in i0..=i1 {
                let rect = self.rect(i, j)?;
                if let Some(t) = self.tiles.get_mut(j * self.nx + i)
                    && !t.is_empty()
                {
                    *t = t.closed(0.003)?.simplified(0.002)?.intersection(&rect)?;
                }
            }
        }
        Ok(())
    }

    /// The area within `lo`..`hi`, closed as in [`Tiled::add_closed`].
    fn window_closed(&self, lo: [f64; 2], hi: [f64; 2]) -> Result<Area> {
        self.window(lo, hi)?.closed(0.003)
    }

    /// The area within `lo`..`hi`.
    fn window(&self, lo: [f64; 2], hi: [f64; 2]) -> Result<Area> {
        let (i0, i1, j0, j1) = self.span(lo, hi);
        let parts: Vec<&Area> =
            (j0..=j1).flat_map(|j| (i0..=i1).map(move |i| j * self.nx + i)).filter_map(|k| self.tiles.get(k)).filter(|t| !t.is_empty()).collect();
        Area::union_all(&parts)?.intersection(&Area::rect(lo[0], lo[1], hi[0], hi[1])?)
    }
}

/// `lo`..`hi` of the points, grown by `m`.
fn bbox(pts: &[[f64; 2]], m: f64) -> ([f64; 2], [f64; 2]) {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in pts {
        lo = [lo[0].min(p[0]), lo[1].min(p[1])];
        hi = [hi[0].max(p[0]), hi[1].max(p[1])];
    }
    ([lo[0] - m, lo[1] - m], [hi[0] + m, hi[1] + m])
}

/// The open stretches of `next`'s boundary not already run (within the grid's tolerance of
/// the visited region's edges), each climbing: the uncut side on the left.
fn fronts(next: &Area, visited: &EdgeGrid, spacing: f64) -> Vec<Vec<[f64; 2]>> {
    let mut out = Vec::new();
    for ring in next.rings() {
        // outer rings run anticlockwise and holes clockwise: reverse both to keep the uncut
        // material (outside the region) on the left
        let rev: Ring = ring.iter().rev().copied().collect();
        let pts = densify(&rev, spacing);
        let new: Vec<bool> = pts.iter().map(|p| !visited.near(*p)).collect();
        if new.iter().all(|n| *n) {
            out.push(pts);
            continue;
        }
        // rotate so a run never wraps round the ring's start
        let Some(start) = new.iter().position(|n| !*n) else { continue };
        let len = pts.len().saturating_sub(1);
        let mut cur: Vec<[f64; 2]> = Vec::new();
        for k in 0..=len {
            let i = (start + k) % len.max(1);
            let (Some(&p), Some(&is_new)) = (pts.get(i), new.get(i)) else { continue };
            if is_new {
                if cur.is_empty()
                    && let Some(&prev) = pts.get((i + len - 1) % len.max(1))
                {
                    cur.push(prev);
                }
                cur.push(p);
            } else if !cur.is_empty() {
                cur.push(p);
                out.push(std::mem::take(&mut cur));
            }
        }
        if cur.len() >= 2 {
            out.push(cur);
        }
    }
    out.into_iter().filter(|s| s.len() >= 2).collect()
}

/// Is the straight move from `a` to `b` inside `area` everywhere, checked every 0.05 mm (a
/// chord between two points of an edge can graze outside for a fraction of a millimetre)?
///
/// The ends themselves are not checked when they lie on (within 0.02 mm of) the area's edge,
/// where a stretch of a pass starts and ends: inside or outside is a matter of rounding there.
/// Every point between them is, to within 0.005 mm (the paths' tolerance) of the edge: a link
/// along the edge, as along a pocket's wall from one pass's end to the next one's start, is
/// inside.
fn inside_all_the_way(area: &Area, a: [f64; 2], b: [f64; 2]) -> bool {
    let len = (a[0] - b[0]).hypot(a[1] - b[1]);
    let n = (len / 0.05).ceil().clamp(1.0, 20_000.0) as usize;
    let near_edge = |p: [f64; 2], tol: f64| nearest_on(area, p).is_some_and(|q| (q[0] - p[0]).hypot(q[1] - p[1]) <= tol);
    (0..=n).all(|i| {
        let t = i as f64 / n as f64;
        let p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
        area.contains(p) || near_edge(p, if i == 0 || i == n { 0.02 } else { 0.005 })
    })
}

/// The point of `area`'s outline nearest `p`: on its edges, not just at its vertices (after
/// thinning, a long straight edge has a vertex at each end only).
fn nearest_on(area: &Area, p: [f64; 2]) -> Option<[f64; 2]> {
    let mut best: Option<([f64; 2], f64)> = None;
    for ring in area.rings() {
        for (a, b) in crate::region::ring_edges(ring) {
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let l2 = dx * dx + dy * dy;
            let t = if l2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
            let q = [a[0] + t * dx, a[1] + t * dy];
            let d = (q[0] - p[0]).hypot(q[1] - p[1]);
            if best.is_none_or(|(_, bd)| d < bd) {
                best = Some((q, d));
            }
        }
    }
    best.map(|(q, _)| q)
}

/// Rough one level of a pocket: `allowed` is where the cutter's centre may go, `from` the top of
/// the material there (the level above), `z` the level.
///
/// Returns where the engagement could not be kept under the limit, as notes for the plan's
/// warnings: a region too narrow for anything but slotting, and passes still over the limit at
/// the finest step.
pub fn clear_level(b: &mut PathBuilder, allowed: &Area, z: f64, from: f64, tool: &Tool, p: &AdaptiveParams) -> Result<Vec<String>> {
    let mut notes = Vec::new();
    for shape in &allowed.shapes {
        let s = Area { shapes: vec![shape.clone()] };
        notes.extend(clear_shape(b, &s, z, from, tool, p)?);
    }
    Ok(notes)
}

fn at(shape: &Area) -> String {
    shape.bounds().map(|(lo, hi)| format!("({:.1}, {:.1})", (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0)).unwrap_or_default()
}

fn clear_shape(b: &mut PathBuilder, shape: &Area, z: f64, from: f64, t: &Tool, p: &AdaptiveParams) -> Result<Vec<String>> {
    let rt = t.diameter / 2.0;
    let step = p.step.clamp(rt * 0.02, rt * 0.9);
    let (Some((lo, hi)), true) = (shape.bounds(), z < from) else { return Ok(Vec::new()) };
    let mut cleared = Cleared::new([lo[0] - rt, lo[1] - rt], [hi[0] + rt, hi[1] + rt], (rt / 40.0).max(0.01))?;
    let spacing = (step / 2.0).clamp(0.05, 1.0);

    // Entry: a helix at the most open point, as wide as there is room for, up to half the
    // cutter's radius.
    let shrink = (rt * 0.1).max(0.02);
    let (c, room) = most_open(shape, shrink)?;
    let rh = (rt * 0.5).min(room * 0.9);
    if rh < rt * 0.05 {
        // a sliver: run its outline, ramping in
        for ring in shape.rings() {
            let r: Ring = ring.iter().rev().copied().collect();
            let start = crate::toolpath::nearest_vertex(&r, [b.at[0], b.at[1]]);
            let Some(&q) = r.get(start) else { continue };
            b.travel_to(q, from);
            b.line([q[0], q[1], from], t.plunge);
            b.ramp_ring(&r, start, z, p.ramp, t.feed * p.slotting, t.plunge);
            cleared.sweep(&densify(&r, spacing), rt);
            b.up();
        }
        return Ok(vec![format!(
            "adaptive: the region at {} on z {z:.3} leaves T{}'s centre less than {:.2} mm of room: it is cut by slotting (up to 180° engaged); a smaller cutter keeps the engagement down",
            at(shape),
            t.number,
            // the room is found by shrinking in steps of `shrink`: it is less than one more step
            2.0 * (room.max(0.0) + shrink)
        )]);
    }
    // Where a front meets the region's edge (a wall, either side of a channel), the material
    // beside the last pass lies on a quarter of the cutter's edge whatever the step: within a
    // step of the edge a pass may take that quarter and a straight cut's engagement.
    let wall_limit = p.max_engagement.max(std::f64::consts::FRAC_PI_2 + AdaptiveParams::straight_engagement(step, rt));
    b.travel_to([c[0] + rh, c[1]], from);
    b.line([c[0] + rh, c[1], from], t.plunge);
    let pitch = (std::f64::consts::TAU * rh * p.ramp.tan()).max(0.01);
    // a helix as wide as half the cutter is a slot going down
    b.helix(c, rh, z, pitch, t.feed * p.slotting);
    cleared.stamp(c, rh + rt);
    let centres = Area::circle(c[0], c[1], rh, 0.01)?.intersection(shape)?;
    // The centres' region, the same grown by the full step, and the region itself, in tiles:
    // each pass reads a window round the tool, and adds what it cut to the tiles under it, so
    // a pass costs what is near it, not what the whole region holds. Offsetting a union is the
    // union of the offsets, so each piece cut adds its own offset to the grown region.
    let tile = (8.0 * rt).max(6.0 * step).max(2.0);
    let margin = 2.0 * step + rt + 1.0;
    let (blo, bhi) = ([lo[0] - margin, lo[1] - margin], [hi[0] + margin, hi[1] + margin]);
    let mut cen = Tiled::new(blo, bhi, tile);
    cen.add(&centres)?;
    let mut grown = Tiled::new(blo, bhi, tile);
    grown.add(&centres.offset(step, 0.01)?)?;
    let mut region = Tiled::new(blo, bhi, tile);
    region.add(shape)?;
    // the first point still over the limit at the finest step, if any
    let mut over: Option<[f64; 2]> = None;
    // the step the last pass took: the next starts at twice that (up to the full step), not
    // at the full step every time, where a front runs along a wall pass after pass
    let mut last = step;

    let mut passes = 0usize;
    for _ in 0..MAX_PASSES {
        passes += 1;
        // The next pass at the full step, one piece of it at a time: where the region branches
        // (the arms of an L or a cross, the two ends of a long slot) the growth falls apart in
        // pieces, and the tool finishes the piece nearest it, pass after pass, before moving
        // on, instead of hopping from arm to arm every pass. Where a piece would engage more
        // than the limit anywhere (where its front meets a wall or wraps round an island), it
        // is grown again at half the step, down to a sixteenth: smaller steps, smooth fronts.
        // The growth is looked for in a window round the tool, twice as wide each time the
        // nearest piece might reach past it, up to the whole region.
        let here = [b.at[0], b.at[1]];
        let s0 = if last * 2.0 >= step - 1e-12 { step } else { last * 2.0 };
        let mut half = tile;
        let picked = 'window: loop {
            let (wlo, whi) = ([here[0] - half, here[1] - half], [here[0] + half, here[1] + half]);
            let whole = wlo[0] <= blo[0] && wlo[1] <= blo[1] && whi[0] >= bhi[0] && whi[1] >= bhi[1];
            let (wlo, whi) = if whole { (blo, bhi) } else { (wlo, whi) };
            // read a step and a millimetre wider: the growth inside the window is exact
            let pad = step + 1.0;
            let visited = cen.window_closed([wlo[0] - pad, wlo[1] - pad], [whi[0] + pad, whi[1] + pad])?;
            let inside = region.window(wlo, whi)?;
            // the region's edge, read wider than the window so its cut has no edge of its own
            let walls = EdgeGrid::new(&region.window([wlo[0] - pad, wlo[1] - pad], [whi[0] + pad, whi[1] + pad])?, step + 0.05);
            let mut s = s0;
            loop {
                let next = if s == step { grown.window(wlo, whi)? } else { visited.offset(s, 0.01)? };
                let next = next.intersection(&inside)?;
                let window = (!whole).then_some((wlo, whi));
                match nearest_piece(&next, &visited, here, (s * 0.05).max(0.005), spacing, p.both_ways, window)? {
                    Pick::Done => break 'window None,
                    Pick::Wider => {
                        half *= 2.0;
                        continue 'window;
                    }
                    Pick::Piece(piece, segs) => {
                        let hot = hot_points(&segs, &cleared, rt, p.max_engagement, &walls, wall_limit);
                        if hot.is_empty() {
                            break 'window Some((piece, segs, s));
                        }
                        if s <= step / 16.0 + 1e-12 {
                            if over.is_none() {
                                over = hot.first().copied();
                            }
                            break 'window Some((piece, segs, s));
                        }
                        s /= 2.0;
                    }
                }
            }
        };
        let Some((piece, segs, s)) = picked else { break };
        last = s;
        let maps = Maps { cen: &cen, region: &region, pad: step + rt + 1.0 };
        let mut todo = segs;
        while !todo.is_empty() {
            let here = [b.at[0], b.at[1]];
            let d = |q: &[f64; 2]| (q[0] - here[0]).hypot(q[1] - here[1]);
            // nearest start (or end, cutting both ways)
            let (i, rev) = todo
                .iter()
                .enumerate()
                .flat_map(|(i, sg)| {
                    let first = sg.first().map(d).unwrap_or(f64::INFINITY);
                    let last = if p.both_ways { sg.last().map(d).unwrap_or(f64::INFINITY) } else { f64::INFINITY };
                    [(i, false, first), (i, true, last)]
                })
                .min_by(|a, b| a.2.total_cmp(&b.2))
                .map(|(i, rev, _)| (i, rev))
                .unwrap_or((0, false));
            let mut sg = todo.swap_remove(i);
            if rev {
                sg.reverse();
            }
            let Some(&start) = sg.first() else { continue };
            if sg.len() > 2 && sg.first() == sg.last() {
                // A whole new ring: no stretch of it is cut yet, so its start lies a step out
                // from the cleared ground. Spiral onto it: over the first quarter of the ring
                // (at least 2 mm) each point goes from the cleared edge out to the ring in
                // proportion, so the cut deepens from nothing to the step; round the ring; then
                // that first stretch again at full width.
                let (rlo, rhi) = bbox(&sg, maps.pad);
                let near = cen.window(rlo, rhi)?;
                let len: f64 = sg.windows(2).map(|w| w.first().zip(w.get(1)).map_or(0.0, |(a, b)| (b[0] - a[0]).hypot(b[1] - a[1]))).sum();
                let lead_len = (len / 4.0).max(2.0).min(len * 0.9);
                let mut along = 0.0;
                let mut lead: Vec<[f64; 2]> = Vec::new();
                for w in sg.windows(2) {
                    let (Some(a), Some(q)) = (w.first(), w.get(1)) else { continue };
                    if lead.is_empty() {
                        lead.push(nearest_on(&near, *a).unwrap_or(*a));
                    }
                    along += (q[0] - a[0]).hypot(q[1] - a[1]);
                    if along >= lead_len {
                        break;
                    }
                    let f = along / lead_len;
                    let n = nearest_on(&near, *q).unwrap_or(*q);
                    lead.push([n[0] + (q[0] - n[0]) * f, n[1] + (q[1] - n[1]) * f]);
                }
                let k = lead.len();
                let Some(&first) = lead.first() else { continue };
                link(b, &maps, c, first, z, from, t, p)?;
                // on to the ring where the lead-in reaches it, round, and over the start again
                let path: Vec<[f64; 2]> = lead.iter().chain(sg.iter().skip(k)).chain(sg.iter().skip(1).take(k)).copied().collect();
                cut(b, &cleared, &path, z, t, p);
                cleared.sweep(&sg, rt);
                continue;
            } else {
                link(b, &maps, c, start, z, from, t, p)?;
            }
            cut(b, &cleared, &sg, z, t, p);
            cleared.sweep(&sg, rt);
        }
        // offsets add a vertex at every round corner: thin each piece out before it is kept
        let piece = piece.simplified(0.002)?;
        cen.add_closed(&piece)?;
        piece.offset(step, 0.01).and_then(|g| g.simplified(0.002)).and_then(|g| grown.add(&g))?;
    }
    let mut notes: Vec<String> = Vec::new();
    if passes >= MAX_PASSES {
        notes.push(format!("adaptive: the region at {} on z {z:.3} stopped after {MAX_PASSES} passes: part of it is left uncut", at(shape)));
    }
    if let Some(q) = over {
        notes.push(format!(
            "adaptive: near ({:.2}, {:.2}) on z {z:.3} T{} engages more than {:.0}° even at a sixteenth of the stepover",
            q[0],
            q[1],
            t.number,
            p.max_engagement.to_degrees()
        ));
    }
    Ok(notes)
}

/// Windows of the centres' region (cleared ground) and of the region the centre may go, for
/// links: each read round the link's ends, `pad` wider.
struct Maps<'a> {
    cen: &'a Tiled,
    region: &'a Tiled,
    pad: f64,
}

/// From where the tool is (at depth `z`) to `to`, the quickest way: straight at depth through
/// cleared ground; at depth through the hub (round a corner, as from one arm of a cross to
/// another); lifted just above the level and across over the region; or up to clearance and
/// across at rapid speed. A way slower than going up and over is not looked at.
#[allow(clippy::too_many_arguments)]
fn link(b: &mut PathBuilder, maps: &Maps, hub: [f64; 2], to: [f64; 2], z: f64, from: f64, t: &Tool, p: &AdaptiveParams) -> Result<()> {
    let here = [b.at[0], b.at[1]];
    if (here[0] - to[0]).hypot(here[1] - to[1]) < 1e-9 && (b.at[2] - z).abs() < 1e-9 {
        return Ok(());
    }
    let down = (b.at[2] - z).abs() < 1e-9;
    let d = |a: [f64; 2], q: [f64; 2]| (a[0] - q[0]).hypot(a[1] - q[1]);
    let (tlo, thi) = bbox(&[to], maps.pad);
    let over = nearest_on(&maps.cen.window(tlo, thi)?, to).unwrap_or(to);
    let lift = from + p.retract;
    let (feed, rapid) = (p.link.max(t.feed).max(1e-6), p.rapid.max(1e-6));
    let up_feed = t.plunge.max(t.feed).max(1e-6);
    let clear = b.h.clearance;
    let above = (from + p.retract).min(clear);
    let up_time = ((clear - b.at[2]).max(0.0) + d(here, over) + (clear - above).max(0.0)) / rapid
        + (above - z).max(0.0) / feed
        + d(over, to) / t.feed.max(1e-6);
    let mut ways = [
        (d(here, to) / feed, 0u8),
        ((d(here, hub) + d(hub, to)) / feed, 1),
        ((lift - z) / up_feed + d(here, over) / feed + d(over, to) / t.feed.max(1e-6) + (lift - z) / feed, 2),
    ];
    ways.sort_by(|a, b| a.0.total_cmp(&b.0));
    let inside = |tiles: &Tiled, a: [f64; 2], q: [f64; 2]| -> Result<bool> {
        let (lo, hi) = bbox(&[a, q], 1.0);
        Ok(inside_all_the_way(&tiles.window(lo, hi)?, a, q))
    };
    let mut way = 3u8;
    if down {
        for (time, w) in ways {
            if time >= up_time {
                break;
            }
            let ok = match w {
                0 => inside(maps.cen, here, to)?,
                1 => inside(maps.cen, here, hub)? && inside(maps.cen, hub, to)?,
                _ => inside(maps.region, here, over)?,
            };
            if ok {
                way = w;
                break;
            }
        }
    }
    match way {
        0 => {
            b.line([to[0], to[1], z], feed);
            return Ok(());
        }
        1 => {
            b.line([hub[0], hub[1], z], feed);
            b.line([to[0], to[1], z], feed);
            return Ok(());
        }
        2 => {
            // above this level's material (the levels above cleared all of the region), but at
            // feed: rapids across run at clearance height only
            b.line([here[0], here[1], lift], up_feed);
            b.line([over[0], over[1], lift], feed);
        }
        _ => {
            b.up();
            b.travel_to(over, from);
        }
    }
    // down onto ground this level has cleared (the cutter's centre has been there), through the
    // levels above it: in air
    b.line([over[0], over[1], z], feed);
    b.line([to[0], to[1], z], t.feed);
    Ok(())
}
