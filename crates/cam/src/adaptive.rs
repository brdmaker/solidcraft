//! Adaptive clearing: the cleared area grows from a helix entry at the most open point of each
//! pocket, one stepover at a time, along the front of uncut material, so the cutter never takes
//! more than the stepover from the side.
//!
//! Each pass is the boundary of the tool centres' region grown by the step and clipped to where
//! the centre may go, less the stretches earlier passes already ran (along walls). Before a pass
//! is cut, the cutter's engagement (the arc of its edge in uncut material) is measured at every
//! point of it on a grid of what is already cleared; where it would pass the limit, as where the
//! front wraps into a corner or round an island, the pass is planned again at half the step, down
//! to a sixteenth. Moves between stretches stay at depth through cleared area, else lift just above
//! the level and re-enter over cleared ground.

use crate::job::Tool;
use crate::region::{Area, Ring};
use crate::toolpath::PathBuilder;
use crate::{CamError, Result};

/// Directions the engagement is sampled in round the cutter's edge.
const RAYS: usize = 96;

/// The most grid cells of the cleared map; past it the cells grow.
const MAX_CELLS: usize = 4_000_000;

/// The most passes one pocket level may take.
const MAX_PASSES: usize = 5_000;

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
}

impl AdaptiveParams {
    /// The engagement of a straight cut `step` deep into the side of a cutter of radius `rt`:
    /// the arc of its edge, from straight ahead round to where the uncut material ends,
    /// acos(1 − step/rt) (180° in a full slot).
    pub fn straight_engagement(step: f64, rt: f64) -> f64 {
        (1.0 - (step / rt).clamp(0.0, 2.0)).clamp(-1.0, 1.0).acos()
    }
}

/// What has been cut, on a grid.
pub struct Cleared {
    x0: f64,
    y0: f64,
    h: f64,
    nx: usize,
    ny: usize,
    cells: Vec<bool>,
}

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
        Ok(Cleared { x0: lo[0], y0: lo[1], h, nx, ny, cells: vec![false; nx * ny] })
    }

    fn cell(&self, p: [f64; 2]) -> Option<usize> {
        let (i, j) = ((p[0] - self.x0) / self.h, (p[1] - self.y0) / self.h);
        if i < 0.0 || j < 0.0 {
            return None;
        }
        let (i, j) = (i as usize, j as usize);
        (i < self.nx && j < self.ny).then_some(j * self.nx + i)
    }

    /// Is the point cut? Off the grid counts as uncut.
    pub fn is_cut(&self, p: [f64; 2]) -> bool {
        self.cell(p).and_then(|c| self.cells.get(c)).copied().unwrap_or(false)
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
                if (x - c[0]).hypot(y - c[1]) <= r
                    && let Some(cell) = self.cells.get_mut(j * self.nx + i)
                {
                    *cell = true;
                }
            }
        }
    }

    /// Mark everything a cutter of radius `r` sweeps along the polyline as cut.
    pub fn sweep(&mut self, pts: &[[f64; 2]], r: f64) {
        let step = (self.h * 0.7).max(1e-4);
        for w in pts.windows(2) {
            let (Some(a), Some(b)) = (w.first(), w.get(1)) else { continue };
            let n = ((a[0] - b[0]).hypot(a[1] - b[1]) / step).ceil().clamp(1.0, 1e6) as usize;
            for k in 0..=n {
                let t = k as f64 / n as f64;
                self.stamp([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t], r);
            }
        }
        if let [p] = pts {
            self.stamp(*p, r);
        }
    }

    /// The cutter's engagement at `p` travelling along `dir` (a unit vector): the arc of the
    /// forward half of its edge (radius `r`) over uncut material, radians (π in a full slot).
    pub fn engagement(&self, p: [f64; 2], dir: [f64; 2], r: f64) -> f64 {
        let rr = r * 0.97;
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

/// The points of `segs` where cutting them would engage more than `limit` (radians), thinned
/// to one every half millimetre or so.
fn hot_points(segs: &[Vec<[f64; 2]>], cleared: &Cleared, rt: f64, limit: f64) -> Vec<[f64; 2]> {
    let mut hot: Vec<[f64; 2]> = Vec::new();
    for w in segs.iter().flat_map(|sg| sg.windows(2)) {
        let (Some(a), Some(b)) = (w.first(), w.get(1)) else { continue };
        let len = (b[0] - a[0]).hypot(b[1] - a[1]);
        if len > 1e-9
            && cleared.engagement(*b, [(b[0] - a[0]) / len, (b[1] - a[1]) / len], rt) > limit
            && hot.last().is_none_or(|h| (h[0] - b[0]).hypot(h[1] - b[1]) > 0.5)
            && hot.len() < 5_000
        {
            hot.push(*b);
        }
    }
    hot
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

/// The open stretches of `next`'s boundary not already run (inside `visited` grown by `tol`),
/// each climbing: the uncut side on the left.
fn fronts(next: &Area, visited: &Area, tol: f64, spacing: f64) -> Result<Vec<Vec<[f64; 2]>>> {
    let grown = visited.offset(tol, 0.01)?;
    let mut out = Vec::new();
    for ring in next.rings() {
        // outer rings run anticlockwise and holes clockwise: reverse both to keep the uncut
        // material (outside the region) on the left
        let rev: Ring = ring.iter().rev().copied().collect();
        let pts = densify(&rev, spacing);
        let new: Vec<bool> = pts.iter().map(|p| !grown.contains(*p)).collect();
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
    Ok(out.into_iter().filter(|s| s.len() >= 2).collect())
}

fn inside_all_the_way(area: &Area, a: [f64; 2], b: [f64; 2]) -> bool {
    let n = ((a[0] - b[0]).hypot(a[1] - b[1]) / 0.25).ceil().clamp(1.0, 2000.0) as usize;
    (0..=n).all(|i| {
        let t = i as f64 / n as f64;
        area.contains([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t])
    })
}

/// The point of `area`'s rings nearest `p`.
fn nearest_on(area: &Area, p: [f64; 2]) -> Option<[f64; 2]> {
    area.rings().flat_map(|r| r.iter()).min_by(|a, b| (a[0] - p[0]).hypot(a[1] - p[1]).total_cmp(&(b[0] - p[0]).hypot(b[1] - p[1]))).copied()
}

/// Rough one level of a pocket: `allowed` is where the cutter's centre may go, `from` the top of
/// the material there (the level above), `z` the level.
pub fn clear_level(b: &mut PathBuilder, allowed: &Area, z: f64, from: f64, tool: &Tool, p: &AdaptiveParams) -> Result<()> {
    for shape in &allowed.shapes {
        let s = Area { shapes: vec![shape.clone()] };
        clear_shape(b, &s, z, from, tool, p)?;
    }
    Ok(())
}

fn clear_shape(b: &mut PathBuilder, shape: &Area, z: f64, from: f64, t: &Tool, p: &AdaptiveParams) -> Result<()> {
    let rt = t.diameter / 2.0;
    let step = p.step.clamp(rt * 0.02, rt * 0.9);
    let (Some((lo, hi)), true) = (shape.bounds(), z < from) else { return Ok(()) };
    let mut cleared = Cleared::new([lo[0] - rt, lo[1] - rt], [hi[0] + rt, hi[1] + rt], (rt / 40.0).max(0.01))?;
    let spacing = (step / 2.0).clamp(0.05, 1.0);

    // Entry: a helix at the most open point, as wide as there is room for, up to half the
    // cutter's radius.
    let (c, room) = most_open(shape, (rt * 0.1).max(0.02))?;
    let rh = (rt * 0.5).min(room * 0.9);
    if rh < rt * 0.05 {
        // a sliver: run its outline, ramping in
        for ring in shape.rings() {
            let r: Ring = ring.iter().rev().copied().collect();
            let start = crate::toolpath::nearest_vertex(&r, [b.at[0], b.at[1]]);
            let Some(&q) = r.get(start) else { continue };
            b.travel_to(q, from);
            b.line([q[0], q[1], from], t.plunge);
            b.ramp_ring(&r, start, z, p.ramp, t.feed, t.plunge);
            cleared.sweep(&densify(&r, spacing), rt);
            b.up();
        }
        return Ok(());
    }
    b.travel_to([c[0] + rh, c[1]], from);
    b.line([c[0] + rh, c[1], from], t.plunge);
    let pitch = (std::f64::consts::TAU * rh * p.ramp.tan()).max(0.01);
    b.helix(c, rh, z, pitch, t.feed);
    cleared.stamp(c, rh + rt);
    let mut centres = Area::circle(c[0], c[1], rh, 0.01)?.intersection(shape)?;

    for _ in 0..MAX_PASSES {
        // The next pass at the full step; where it would engage more than the limit anywhere
        // (where the front meets a wall or wraps round an island), the whole pass again at half
        // the step, down to a sixteenth: smaller steps, smooth fronts.
        let mut s = step;
        let (next, segs) = loop {
            let next = centres.offset(s, 0.01)?.intersection(shape)?;
            let segs = fronts(&next, &centres, (s * 0.05).max(0.005), spacing)?;
            if s <= step / 16.0 + 1e-12 || hot_points(&segs, &cleared, rt, p.max_engagement).is_empty() {
                break (next, segs);
            }
            s /= 2.0;
        };
        if next.area() - centres.area() < 1e-6 || segs.is_empty() {
            break;
        }
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
            link(b, &centres, shape, start, z, from, t, p);
            for q in sg.iter().skip(1) {
                b.line([q[0], q[1], z], t.feed);
            }
            cleared.sweep(&sg, rt);
        }
        // repeated offsets add a vertex at every round corner: thin them out
        centres = next.simplified(0.002)?;
    }
    Ok(())
}

/// From where the tool is (at depth `z`) to `to`: straight at depth through cleared area, or
/// lifted over it and back down where it is cleared, or up and over at clearance.
#[allow(clippy::too_many_arguments)]
fn link(b: &mut PathBuilder, centres: &Area, shape: &Area, to: [f64; 2], z: f64, from: f64, t: &Tool, p: &AdaptiveParams) {
    let here = [b.at[0], b.at[1]];
    if (here[0] - to[0]).hypot(here[1] - to[1]) < 1e-9 && (b.at[2] - z).abs() < 1e-9 {
        return;
    }
    if (b.at[2] - z).abs() < 1e-9 && inside_all_the_way(centres, here, to) {
        b.line([to[0], to[1], z], t.feed);
        return;
    }
    // re-enter over cleared ground next to the start
    let over = nearest_on(centres, to).unwrap_or(to);
    let lift = from + p.retract;
    if (b.at[2] - z).abs() < 1e-9 && inside_all_the_way(shape, here, over) {
        b.line([here[0], here[1], lift], t.plunge.max(t.feed));
        b.line([over[0], over[1], lift], t.feed);
    } else {
        b.up();
        b.travel_to(over, from);
    }
    b.line([over[0], over[1], z], t.plunge);
    b.line([to[0], to[1], z], t.feed);
}
