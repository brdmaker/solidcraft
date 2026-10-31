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
const RAYS: usize = 192;

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
    /// The machine's rapid traverse speed, mm/min: links take the quickest way.
    pub rapid: f64,
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
                if (x - a[0] - t * dx).hypot(y - a[1] - t * dy) <= r
                    && let Some(cell) = self.cells.get_mut(j * self.nx + i)
                {
                    *cell = true;
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

/// The points of `segs` where cutting them would engage more than `limit` (radians), thinned
/// to one every half millimetre or so.
fn hot_points(segs: &[Vec<[f64; 2]>], cleared: &Cleared, rt: f64, limit: f64) -> Vec<[f64; 2]> {
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
            if cleared.engagement(q, dir, rt) > limit && hot.last().is_none_or(|h| (h[0] - q[0]).hypot(h[1] - q[1]) > 0.5) && hot.len() < 5_000 {
                hot.push(q);
            }
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
        if let Some(n) = clear_shape(b, &s, z, from, tool, p)? {
            notes.push(n);
        }
    }
    Ok(notes)
}

fn at(shape: &Area) -> String {
    shape.bounds().map(|(lo, hi)| format!("({:.1}, {:.1})", (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0)).unwrap_or_default()
}

fn clear_shape(b: &mut PathBuilder, shape: &Area, z: f64, from: f64, t: &Tool, p: &AdaptiveParams) -> Result<Option<String>> {
    let rt = t.diameter / 2.0;
    let step = p.step.clamp(rt * 0.02, rt * 0.9);
    let (Some((lo, hi)), true) = (shape.bounds(), z < from) else { return Ok(None) };
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
            b.ramp_ring(&r, start, z, p.ramp, t.feed, t.plunge);
            cleared.sweep(&densify(&r, spacing), rt);
            b.up();
        }
        return Ok(Some(format!(
            "adaptive: the region at {} on z {z:.3} leaves T{}'s centre less than {:.2} mm of room: it is cut by slotting (up to 180° engaged); a smaller cutter keeps the engagement down",
            at(shape),
            t.number,
            // the room is found by shrinking in steps of `shrink`: it is less than one more step
            2.0 * (room.max(0.0) + shrink)
        )));
    }
    b.travel_to([c[0] + rh, c[1]], from);
    b.line([c[0] + rh, c[1], from], t.plunge);
    let pitch = (std::f64::consts::TAU * rh * p.ramp.tan()).max(0.01);
    b.helix(c, rh, z, pitch, t.feed);
    cleared.stamp(c, rh + rt);
    let mut centres = Area::circle(c[0], c[1], rh, 0.01)?.intersection(shape)?;
    // the first point still over the limit at the finest step, if any
    let mut over: Option<[f64; 2]> = None;

    for _ in 0..MAX_PASSES {
        // The next pass at the full step; where it would engage more than the limit anywhere
        // (where the front meets a wall or wraps round an island), the whole pass again at half
        // the step, down to a sixteenth: smaller steps, smooth fronts.
        let mut s = step;
        let (next, segs) = loop {
            let next = centres.offset(s, 0.01)?.intersection(shape)?;
            let segs = fronts(&next, &centres, (s * 0.05).max(0.005), spacing)?;
            let hot = hot_points(&segs, &cleared, rt, p.max_engagement);
            if hot.is_empty() {
                break (next, segs);
            }
            if s <= step / 16.0 + 1e-12 {
                if over.is_none() {
                    over = hot.first().copied();
                }
                break (next, segs);
            }
            s /= 2.0;
        };
        if next.area() - centres.area() < 1e-6 || segs.is_empty() {
            break;
        }
        // links at depth go through the centres' region (cleared ground), checked every 0.05 mm
        let safe = centres.clone();
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
                let len: f64 = sg.windows(2).map(|w| w.first().zip(w.get(1)).map_or(0.0, |(a, b)| (b[0] - a[0]).hypot(b[1] - a[1]))).sum();
                let lead_len = (len / 4.0).max(2.0).min(len * 0.9);
                let mut along = 0.0;
                let mut lead: Vec<[f64; 2]> = Vec::new();
                for w in sg.windows(2) {
                    let (Some(a), Some(q)) = (w.first(), w.get(1)) else { continue };
                    if lead.is_empty() {
                        lead.push(nearest_on(&centres, *a).unwrap_or(*a));
                    }
                    along += (q[0] - a[0]).hypot(q[1] - a[1]);
                    if along >= lead_len {
                        break;
                    }
                    let f = along / lead_len;
                    let n = nearest_on(&centres, *q).unwrap_or(*q);
                    lead.push([n[0] + (q[0] - n[0]) * f, n[1] + (q[1] - n[1]) * f]);
                }
                let k = lead.len();
                let Some(&first) = lead.first() else { continue };
                link(b, &centres, &safe, c, shape, first, z, from, t, p);
                for q in lead.iter().skip(1) {
                    b.line([q[0], q[1], z], t.feed);
                }
                // on to the ring where the lead-in reaches it, round, and over the start again
                for q in sg.iter().skip(k).chain(sg.iter().skip(1).take(k)) {
                    b.line([q[0], q[1], z], t.feed);
                }
                cleared.sweep(&sg, rt);
                continue;
            } else {
                link(b, &centres, &safe, c, shape, start, z, from, t, p);
            }
            for q in sg.iter().skip(1) {
                b.line([q[0], q[1], z], t.feed);
            }
            cleared.sweep(&sg, rt);
        }
        // repeated offsets add a vertex at every round corner: thin them out
        centres = next.simplified(0.002)?;
    }
    Ok(over.map(|q| {
        format!(
            "adaptive: near ({:.2}, {:.2}) on z {z:.3} T{} engages more than {:.0}° even at a sixteenth of the stepover",
            q[0],
            q[1],
            t.number,
            p.max_engagement.to_degrees()
        )
    }))
}

/// From where the tool is (at depth `z`) to `to`: straight at depth through cleared area, or
/// lifted over it and back down where it is cleared, or up and over at clearance.
#[allow(clippy::too_many_arguments)]
fn link(
    b: &mut PathBuilder,
    centres: &Area,
    safe: &Area,
    hub: [f64; 2],
    shape: &Area,
    to: [f64; 2],
    z: f64,
    from: f64,
    t: &Tool,
    p: &AdaptiveParams,
) {
    let here = [b.at[0], b.at[1]];
    if (here[0] - to[0]).hypot(here[1] - to[1]) < 1e-9 && (b.at[2] - z).abs() < 1e-9 {
        return;
    }
    let down = (b.at[2] - z).abs() < 1e-9;
    let d = |a: [f64; 2], q: [f64; 2]| (a[0] - q[0]).hypot(a[1] - q[1]);
    // the ways there, by time: straight at depth through cleared ground; at depth through the
    // hub (round a corner, as from one arm of a cross to another); lifted just above the level
    // and across over the pocket; up to clearance and across at rapid speed
    let over = nearest_on(centres, to).unwrap_or(to);
    let lift = from + p.retract;
    let (feed, plunge, rapid) = (t.feed.max(1e-6), t.plunge.max(1e-6), p.rapid.max(1e-6));
    let up_feed = t.plunge.max(t.feed).max(1e-6);
    let straight = down && inside_all_the_way(safe, here, to);
    let via_hub = down && inside_all_the_way(safe, here, hub) && inside_all_the_way(safe, hub, to);
    let lifted = down && inside_all_the_way(shape, here, over);
    let clear = b.h.clearance;
    let above = (from + p.retract).min(clear);
    let up_time =
        ((clear - b.at[2]).max(0.0) + d(here, over) + (clear - above).max(0.0)) / rapid + (above - z).max(0.0) / plunge + d(over, to) / feed;
    let mut best: Option<(f64, u8)> = None;
    for (ok, time, way) in [
        (straight, d(here, to) / feed, 0u8),
        (via_hub, (d(here, hub) + d(hub, to)) / feed, 1),
        (lifted, (lift - z) / up_feed + (d(here, over) + d(over, to)) / feed + (lift - z) / plunge, 2),
        (true, up_time, 3),
    ] {
        if ok && best.is_none_or(|(l, _)| time < l) {
            best = Some((time, way));
        }
    }
    match best.map(|(_, w)| w) {
        Some(0) => {
            b.line([to[0], to[1], z], t.feed);
            return;
        }
        Some(1) => {
            b.line([hub[0], hub[1], z], t.feed);
            b.line([to[0], to[1], z], t.feed);
            return;
        }
        Some(2) => {
            // above this level's material (the levels above cleared all of `shape`), but at
            // feed: rapids across run at clearance height only
            b.line([here[0], here[1], lift], up_feed);
            b.line([over[0], over[1], lift], t.feed);
        }
        _ => {
            b.up();
            b.travel_to(over, from);
        }
    }
    b.line([over[0], over[1], z], t.plunge);
    b.line([to[0], to[1], z], t.feed);
}
