//! Turning a part: facing the bar's front, drilling and boring its bore, roughing it down in
//! radial steps, finishing the profile, grooving what the turning tools can't reach, cutting its
//! threads and parting it off.
//!
//! Each tool's reach is worked out on the profile's grid along the axis, in the (z, r) half
//! section: the nose's centre may come within the nose radius of every point of the profile
//! (the profile offset by the nose), and going toward the chuck it may only fall as steeply as
//! the insert's trailing edge clears ([`super::job::LatheTool::descent_deg`]). What that leaves
//! (a groove, the far side of a V, a bore) is not cut and is reported.
//!
//! Moves are of the tool's imaginary tip, the corner of the tangents to its nose at the front
//! and below (where it is touched off): the nose's centre less its radius in X and Z. Work
//! coordinates: Z0 the finished front face, X a radius here (the program writes diameters).

use super::job::{LatheJob, LatheTool, LatheToolKind};
use super::profile::Profile;
use crate::{CamError, Result};

/// Tolerance of the paths' simplification, mm.
pub const PATH_TOL: f64 = 0.002;

/// How far up a threading tool's V goes before its body (as wide as the V there) stands up, mm.
pub const THREAD_V: f64 = 1.5;

/// One move of the tool's tip, in work coordinates `[x (radius), z]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TipMove {
    Rapid([f64; 2]),
    Feed([f64; 2]),
    /// A threading pass along Z at the spindle's turn: `pitch` mm a revolution (G33).
    Thread {
        to: [f64; 2],
        pitch: f64,
    },
}

impl TipMove {
    pub fn to(&self) -> [f64; 2] {
        match self {
            TipMove::Rapid(p) | TipMove::Feed(p) | TipMove::Thread { to: p, .. } => *p,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LatheOpKind {
    Facing,
    Drilling,
    Roughing,
    Boring,
    Finishing,
    Grooving,
    Threading,
    Parting,
}

/// How the spindle turns for an operation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Spindle {
    /// Constant surface speed, m/min (G96, held under the machine's top speed).
    SurfaceSpeed(f64),
    /// A fixed speed, rpm (G97).
    Rpm(f64),
}

/// An operation: one tool, its spindle speed and feed (mm/rev), its moves.
#[derive(Clone, Debug)]
pub struct LatheOp {
    pub kind: LatheOpKind,
    pub name: String,
    pub tool: u32,
    pub spindle: Spindle,
    pub feed: f64,
    pub moves: Vec<TipMove>,
}

/// A thread as cut, in work coordinates: the Z span of the passes, major and minor radius and
/// pitch, and how far the tool's V reaches along Z past either end at the thread's full depth.
#[derive(Clone, Copy, Debug)]
pub struct CutThread {
    pub z: [f64; 2],
    pub major: f64,
    pub minor: f64,
    pub pitch: f64,
    pub flank: f64,
}

/// The bar as set up: in work coordinates.
#[derive(Clone, Copy, Debug)]
pub struct Setup {
    /// The bar's radius.
    pub radius: f64,
    /// The bar's length, and how far it stands out of the chuck.
    pub length: f64,
    pub stick_out: f64,
    /// Z of the bar's front before facing (the face allowance), and of the jaws' face.
    pub front: f64,
    pub jaws: f64,
    /// Z of the part's chuck end, and how far back the turning goes (the parting allowance).
    pub part_end: f64,
    pub turned_to: f64,
}

#[derive(Clone, Debug)]
pub struct LathePlan {
    pub setup: Setup,
    pub profile: Profile,
    pub ops: Vec<LatheOp>,
    pub warnings: Vec<String>,
    /// The threads cut.
    pub threads: Vec<CutThread>,
}

/// The nose centre's reach on a grid along the axis, in the part's own z (0 at its chuck end).
struct Reach {
    /// z of sample 0, and the step.
    z0: f64,
    dz: f64,
    /// The centre's lowest radius at each sample (−∞: nothing to keep it up, in front of the part).
    c: Vec<f64>,
}

impl Reach {
    fn z(&self, g: usize) -> f64 {
        self.z0 + g as f64 * self.dz
    }

    fn at(&self, g: usize) -> f64 {
        self.c.get(g).copied().unwrap_or(f64::NEG_INFINITY)
    }
}

/// The reach of a nose of radius `rn` that falls at most `descent` (degrees) toward the chuck,
/// over the profile extended back by `back` (the parting allowance, at the part's end radius) and
/// forward by the nose and `ahead` (open air in front of the part, beyond any allowance).
fn reach(p: &Profile, rn: f64, descent: f64, back: f64, ahead: f64) -> Reach {
    let dz = p.dz;
    let k_back = (back / dz).ceil() as usize;
    let k_front = ((rn + ahead) / dz).ceil() as usize + 2;
    let n = k_back + p.od.len() + k_front;
    let z0 = -(k_back as f64) * dz;
    let first = p.od.first().copied().unwrap_or(0.0);
    // the profile on the grid: −∞ in front of the part
    let od: Vec<f64> = (0..n).map(|g| if g < k_back { first } else { p.od.get(g - k_back).copied().unwrap_or(f64::NEG_INFINITY) }).collect();
    // the profile as points: each sample, and where the radius changes by more than a step between
    // two samples (a face nearly square to the axis, a dome near its apex) points along the stretch
    // between them, so the nose can't slip in between samples
    let mut pts: Vec<[f64; 2]> = Vec::with_capacity(n);
    for g in 0..n {
        let (Some(&r), z) = (od.get(g), z0 + g as f64 * dz) else { continue };
        if !r.is_finite() {
            continue;
        }
        if let Some(&q) = od.get(g + 1)
            && q.is_finite()
            && (q - r).abs() > dz
        {
            pts.push([z, r]);
            let k = ((q - r).abs() / dz).ceil() as usize;
            for i in 1..k {
                let t = i as f64 / k as f64;
                pts.push([z + dz * t, r + (q - r) * t]);
            }
        } else {
            pts.push([z, r]);
        }
    }
    let mut c = vec![f64::NEG_INFINITY; n];
    let mut first = 0usize;
    for (g, slot) in c.iter_mut().enumerate() {
        let z = z0 + g as f64 * dz;
        while pts.get(first).is_some_and(|p| p[0] < z - rn) {
            first += 1;
        }
        let mut best = f64::NEG_INFINITY;
        for p in pts.iter().skip(first) {
            let d = p[0] - z;
            if d > rn {
                break;
            }
            best = best.max(p[1] + (rn * rn - d * d).max(0.0).sqrt());
        }
        *slot = best;
    }
    // toward the chuck it falls no faster than the trailing edge allows
    let fall = dz * descent.to_radians().tan();
    for g in (0..n.saturating_sub(1)).rev() {
        let next = c.get(g + 1).copied().unwrap_or(f64::NEG_INFINITY);
        if let Some(v) = c.get_mut(g)
            && next.is_finite()
        {
            *v = v.max(next - fall);
        }
    }
    Reach { z0, dz, c }
}

/// The surface a nose of radius `rn` leaves, its centre kept to `r`: at each of `p`'s samples,
/// the lowest its round reaches over every position within `rn` along the axis.
fn surface(r: &Reach, rn: f64, p: &Profile) -> Vec<f64> {
    let w = (rn / r.dz).floor() as usize;
    let n = r.c.len();
    (0..p.od.len())
        .map(|i| {
            let z = i as f64 * p.dz;
            let g = ((z - r.z0) / r.dz).round().max(0.0) as usize;
            (g.saturating_sub(w)..=(g + w).min(n.saturating_sub(1)))
                .map(|h| {
                    let d = r.z(h) - z;
                    r.at(h) - (rn * rn - d * d).max(0.0).sqrt()
                })
                .fold(f64::INFINITY, f64::min)
        })
        .collect()
}

/// `r` widened along the axis by `axial` and raised by `radial`: what roughing leaves.
fn leave(r: &Reach, axial: f64, radial: f64) -> Reach {
    let w = (axial / r.dz).round() as usize;
    let n = r.c.len();
    let c = (0..n)
        .map(|g| {
            let v = (g.saturating_sub(w)..=(g + w).min(n.saturating_sub(1))).map(|h| r.at(h)).fold(f64::NEG_INFINITY, f64::max);
            if v.is_finite() { v + radial } else { v }
        })
        .collect();
    Reach { z0: r.z0, dz: r.dz, c }
}

/// Douglas–Peucker: the points of `pts` a line within `tol` of the rest keeps.
pub(crate) fn simplify(pts: &[[f64; 2]], tol: f64) -> Vec<[f64; 2]> {
    if pts.len() < 3 {
        return pts.to_vec();
    }
    let mut keep = vec![false; pts.len()];
    if let Some(k) = keep.first_mut() {
        *k = true;
    }
    if let Some(k) = keep.last_mut() {
        *k = true;
    }
    let mut stack = vec![(0usize, pts.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        let (Some(pa), Some(pb)) = (pts.get(a), pts.get(b)) else { continue };
        let (dx, dy) = (pb[0] - pa[0], pb[1] - pa[1]);
        let l2 = dx * dx + dy * dy;
        let mut far = (0.0, a);
        for i in a + 1..b {
            let Some(p) = pts.get(i) else { continue };
            let t = if l2 > 0.0 { (((p[0] - pa[0]) * dx + (p[1] - pa[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
            let d = (pa[0] + t * dx - p[0]).hypot(pa[1] + t * dy - p[1]);
            if d > far.0 {
                far = (d, i);
            }
        }
        if far.0 > tol {
            if let Some(k) = keep.get_mut(far.1) {
                *k = true;
            }
            stack.push((a, far.1));
            stack.push((far.1, b));
        }
    }
    pts.iter().zip(keep).filter(|(_, k)| *k).map(|(p, _)| *p).collect()
}

/// Builds an operation's moves from nose-centre positions in the part's frame.
struct Moves {
    rn: f64,
    length: f64,
    out: Vec<TipMove>,
}

impl Moves {
    /// The tip, in work coordinates, of the nose centred at (r, z) in the part's frame.
    fn tip(&self, r: f64, z: f64) -> [f64; 2] {
        [r - self.rn, z - self.rn - self.length]
    }

    fn rapid(&mut self, r: f64, z: f64) {
        let p = self.tip(r, z);
        if self.out.last().map(TipMove::to) != Some(p) {
            self.out.push(TipMove::Rapid(p));
        }
    }

    /// Feed through the centre positions `pts` ((r, z) pairs), simplified.
    fn feed(&mut self, pts: &[[f64; 2]]) {
        let tips: Vec<[f64; 2]> = pts.iter().map(|p| self.tip(p[0], p[1])).collect();
        for p in simplify(&tips, PATH_TOL) {
            if self.out.last().map(TipMove::to) != Some(p) {
                self.out.push(TipMove::Feed(p));
            }
        }
    }
}

/// The runs of samples `lo..=hi` where `cut(g)` holds, as (first, last) pairs from the back.
fn runs(lo: usize, hi: usize, cut: impl Fn(usize) -> bool) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut start = None;
    for g in lo..=hi {
        match (cut(g), start) {
            (true, None) => start = Some(g),
            (false, Some(s)) => {
                out.push((s, g - 1));
                start = None;
            }
            _ => {}
        }
    }
    if let Some(s) = start {
        out.push((s, hi));
    }
    out
}

/// What one turning pass set needs: the profile (outside, or a bore mirrored across the axis),
/// the bar's radius in the same terms, the tools and allowances.
struct Turn<'a> {
    profile: &'a Profile,
    top: f64,
    rough: &'a LatheTool,
    finish: &'a LatheTool,
    leave_radial: f64,
    leave_axial: f64,
    back: f64,
    ahead: f64,
    clr: f64,
    /// Finish the front face from the middle out (outside only).
    face: bool,
}

/// What turning left: the roughing and finishing moves, the surface the finishing tool leaves
/// and the best its nose alone could, per profile sample.
struct Turned {
    rough: Vec<TipMove>,
    finish: Vec<TipMove>,
    cut: Vec<f64>,
    best: Vec<f64>,
}

fn turn(t: &Turn) -> Turned {
    let p = t.profile;
    let l = p.length;
    let (rough, finish, clr) = (t.rough, t.finish, t.clr);
    let rr = reach(p, rough.nose_radius, rough.descent_deg(), t.back, t.ahead);
    let rl = leave(&rr, t.leave_axial, t.leave_radial);
    let fr = reach(p, finish.nose_radius, finish.descent_deg(), t.back, t.ahead);
    let front_g = |r: &Reach, rn: f64| (((l + rn) - r.z0) / r.dz).round().max(0.0) as usize;

    // Roughing: levels of the nose centre from the bar's surface down to the reach with the
    // allowances, at most the tool's depth apart; each level cuts every run where the reach lies
    // below it, the last stretch climbing the profile back up the step it leaves
    let rough_moves;
    {
        let rn = rough.nose_radius;
        let top = t.top + rn;
        let g_lo = 0usize;
        let g_front = rl.c.len().saturating_sub(1);
        let low = (g_lo..=g_front).filter(|&g| rl.z(g) <= l).map(|g| rl.at(g)).filter(|v| v.is_finite()).fold(f64::INFINITY, f64::min);
        let mut m = Moves { rn, length: l, out: Vec::new() };
        if low.is_finite() && top - low > 1e-6 {
            let n = ((top - low) / rough.depth).ceil().max(1.0) as usize;
            let step = (top - low) / n as f64;
            let z_start = l + t.leave_axial + rn + clr;
            let safe = top + clr;
            for k in 1..=n {
                let rho = top - step * k as f64;
                let below = |g: usize| {
                    let v = rl.at(g);
                    !v.is_finite() || v < rho - 1e-9
                };
                for (ga, gb) in runs(g_lo, g_front, below).into_iter().rev() {
                    let za = rl.z(ga);
                    let mut path: Vec<[f64; 2]> = Vec::new();
                    if gb >= g_front {
                        m.rapid(safe, z_start);
                        m.rapid(rho, z_start);
                    } else {
                        // a valley: come down the profile from where it stands a step higher
                        let gc = (gb + 1..=g_front).find(|&g| !rl.at(g).is_finite() || rl.at(g) >= rho + step - 1e-9).unwrap_or(g_front);
                        let zc = rl.z(gc);
                        let entry = if rl.at(gc).is_finite() { rl.at(gc).max(rho + step) } else { rho + step };
                        m.rapid(safe, zc);
                        m.rapid(entry + 0.2, zc);
                        for g in (gb + 1..=gc).rev() {
                            path.push([rl.at(g).max(rho).min(entry), rl.z(g)]);
                        }
                        path.push([rho, rl.z(gb)]);
                    }
                    path.push([rho, za]);
                    let mut top_reached = rho;
                    if ga > g_lo {
                        for g in (g_lo..ga).rev() {
                            let v = rl.at(g);
                            if !v.is_finite() {
                                break;
                            }
                            let r = v.min(rho + step);
                            path.push([r, rl.z(g)]);
                            top_reached = r;
                            if v >= rho + step - 1e-9 {
                                break;
                            }
                        }
                    }
                    m.feed(&path);
                    let last = path.last().copied().unwrap_or([rho, za]);
                    let off = 0.5 * step.min(1.0);
                    m.feed(&[[top_reached + off, last[1] + off]]);
                    m.rapid(safe, last[1] + off);
                }
            }
            m.rapid(safe, z_start);
        }
        rough_moves = m.out;
    }

    // Finishing: the profile to size (from the front face's middle out, outside), along to the
    // parting allowance; where roughing left more than the tool's depth, passes standing off
    // evenly before it. In a bore the passes stop where the bore does (the bar's surface).
    let rn = finish.nose_radius;
    let finish_moves;
    {
        let g_front = front_g(&fr, rn).min(fr.c.len().saturating_sub(1));
        let grid = |g: usize| fr.z(g);
        let rough_at = |z: f64| {
            let g = ((z - rl.z0) / rl.dz).round();
            if g.is_finite() && g >= 0.0 { rl.at(g as usize) - rough.nose_radius } else { f64::NEG_INFINITY }
        };
        // where the bar's surface stands (a blind bore's end): no deeper
        let solid = |g: usize| fr.at(g) > t.top + rn + 1e-6;
        let g_end = (0..=g_front).rev().find(|&g| solid(g)).map_or(0, |g| g + 1);
        let gap = (g_end..=g_front)
            .map(|g| {
                let (a, b) = (rough_at(grid(g)), fr.at(g) - rn);
                if a.is_finite() && b.is_finite() { a - b } else { 0.0 }
            })
            .fold(0.0, f64::max);
        let n = (gap / finish.depth).ceil().max(1.0) as usize;
        let d = gap / n as f64;
        let mut m = Moves { rn, length: l, out: Vec::new() };
        let safe = t.top + rn + clr;
        let z_face = l + rn;
        if g_end <= g_front {
            for j in (0..n).rev() {
                let off = d * j as f64;
                let path: Vec<[f64; 2]> = (g_end..=g_front)
                    .rev()
                    .filter_map(|g| {
                        let v = fr.at(g);
                        v.is_finite().then(|| {
                            let cap = rough_at(grid(g)) + rn;
                            [if cap.is_finite() { (v + off).min(cap.max(v)) } else { v + off }, grid(g)]
                        })
                    })
                    .collect();
                let Some(&first) = path.first() else { continue };
                m.rapid(safe, z_face + clr);
                if j == 0 && t.face {
                    m.rapid(0.0, z_face + clr);
                    m.feed(&[[0.0, z_face], [first[0], z_face]]);
                } else {
                    m.rapid(first[0], z_face + clr);
                    m.feed(&[[first[0], z_face]]);
                }
                m.feed(&path);
                let last = path.last().copied().unwrap_or(first);
                let out = if g_end > 0 { (last[0] + clr).min(safe) } else { last[0] + clr };
                m.feed(&[[out, last[1] + clr]]);
                m.rapid(safe, last[1] + clr);
            }
            m.rapid(safe, z_face + clr);
        }
        finish_moves = m.out;
    }
    let cut = surface(&fr, rn, p);
    let free = reach(p, rn, 89.9, t.back, t.ahead);
    let best = surface(&free, rn, p);
    Turned { rough: rough_moves, finish: finish_moves, cut, best }
}

/// `p` mirrored across the axis about the bore: each sample's radius the bore's (at least
/// `hole`, the drilled hole's) negated; solid to the axis where there is no bore, beyond `depth`
/// from the front (the bar's reach) the hole as drilled.
fn mirrored(p: &Profile, hole: f64, depth: f64) -> Profile {
    let l = p.length;
    let od =
        p.id.iter()
            .enumerate()
            .map(|(i, r)| {
                let z = i as f64 * p.dz;
                if *r <= 0.0 {
                    0.0
                } else if l - z > depth + 0.01 {
                    -hole
                } else {
                    -r.max(hole)
                }
            })
            .collect();
    Profile { length: l, dz: p.dz, od, id: vec![0.0; p.id.len()] }
}

/// Moves in a mirrored profile's terms back in the bore: each radius negated.
fn unmirror(moves: Vec<TipMove>) -> Vec<TipMove> {
    let flip = |p: [f64; 2]| [-p[0], p[1]];
    moves
        .into_iter()
        .map(|m| match m {
            TipMove::Rapid(p) => TipMove::Rapid(flip(p)),
            TipMove::Feed(p) => TipMove::Feed(flip(p)),
            TipMove::Thread { to, pitch } => TipMove::Thread { to: flip(to), pitch },
        })
        .collect()
}

/// Spans of samples where `left` stands more than 0.01 above what could be cut: (first, last,
/// most).
fn left_runs(n: usize, left: impl Fn(usize) -> f64) -> Vec<(usize, usize, f64)> {
    runs(0, n.saturating_sub(1), |i| left(i) > 0.01).into_iter().map(|(a, b)| (a, b, (a..=b).map(&left).fold(0.0, f64::max))).collect()
}

/// Plan turning `profile` from `job`'s bar.
pub fn plan(job: &LatheJob, profile: Profile) -> Result<LathePlan> {
    let s = &job.stock;
    let st = &job.strategy;
    let l = profile.length;
    let r_bar = s.diameter / 2.0;
    let mut warnings = Vec::new();
    let max_od = profile.max_od();
    if max_od > r_bar + 1e-6 {
        return Err(CamError::Invalid(format!("the part is Ø{:.3}; the bar is Ø{:.3}", 2.0 * max_od, s.diameter)));
    }
    let need = l + s.face_allowance + s.parting_allowance + s.chuck_clearance;
    let length = s.length.unwrap_or_else(|| (need + s.in_chuck).ceil());
    let stick_out = length - s.in_chuck;
    if stick_out < need - 1e-9 {
        return Err(CamError::Invalid(format!(
            "the bar stands {stick_out:.3} out of the chuck; turning the part ({l:.3}), facing ({}), parting ({}) and clearing the jaws ({}) need {need:.3}",
            s.face_allowance, s.parting_allowance, s.chuck_clearance
        )));
    }
    if stick_out > job.machine.z_travel {
        warnings.push(format!("the bar stands {stick_out:.1} out of the chuck, more than the {} of Z travel", job.machine.z_travel));
    }
    let setup = Setup {
        radius: r_bar,
        length,
        stick_out,
        front: s.face_allowance,
        jaws: s.face_allowance - stick_out,
        part_end: -l,
        turned_to: -l - s.parting_allowance,
    };
    let rough = job.rough_tool().ok_or_else(|| CamError::Invalid("the job has no turning tool".into()))?;
    let finish = job.finish_tool().unwrap_or(rough);
    let css = |t: &LatheTool| Spindle::SurfaceSpeed(t.speed(&s.material).unwrap_or(100.0));
    let max_rpm = job.machine.max_rpm;
    let mut ops = Vec::new();
    let clr = st.clearance;
    let back = s.parting_allowance;
    let ahead = st.leave_axial + clr + 1.0;
    let n_samples = profile.od.len();
    let zw = |i: usize| i as f64 * profile.dz - l;

    // Facing: the face allowance off in steps of the roughing depth, to leave_axial
    {
        let rn = rough.nose_radius;
        let mut m = Moves { rn, length: l, out: Vec::new() };
        let take = s.face_allowance - st.leave_axial;
        if take > 1e-9 {
            let n = (take / rough.depth).ceil().max(1.0) as usize;
            for k in 1..=n {
                let z = l + st.leave_axial + take * (1.0 - k as f64 / n as f64) + rn;
                m.rapid(r_bar + rn + clr, z);
                m.feed(&[[r_bar + rn + clr, z], [0.0, z]]);
                m.feed(&[[0.0, z + clr]]);
                m.rapid(r_bar + rn + clr, z + clr);
            }
        }
        if !m.out.is_empty() {
            ops.push(LatheOp {
                kind: LatheOpKind::Facing,
                name: "facing".into(),
                tool: rough.number,
                spindle: css(rough),
                feed: rough.feed,
                moves: m.out,
            });
        }
    }

    // Drilling the bore open at the front: the largest drill that fits it (the turret's first),
    // through the parting allowance where the bore goes through
    let mut hole = 0.0;
    // the bore's radius at the part's chuck end once drilled and bored (parting stops short of it)
    let mut bore_end = 0.0;
    let bore = runs(0, n_samples.saturating_sub(1), |i| profile.id.get(i).copied().unwrap_or(0.0) > 0.0);
    let open = bore.iter().find(|(_, b)| *b + 1 >= n_samples).copied();
    for (a, b) in &bore {
        if Some((*a, *b)) != open {
            warnings.push(format!("Z {:.3} to {:.3}: a bore not open to the front face is not machined", zw(*a), zw(*b)));
        }
    }
    if let Some((a, _)) = open {
        let smallest = (a..n_samples).filter_map(|i| profile.id.get(i)).copied().fold(f64::INFINITY, f64::min);
        let turret = &job.machine.turret;
        let drill = job
            .tools
            .iter()
            .filter(|t| t.kind == LatheToolKind::Drill && t.diameter.unwrap_or(0.0) <= 2.0 * (smallest + super::profile::MESH_TOL) + 1e-6)
            .max_by(|x, y| {
                let k = |t: &LatheTool| (turret.is_empty() || turret.contains(&t.number), t.diameter.unwrap_or(0.0));
                let (kx, ky) = (k(x), k(y));
                kx.0.cmp(&ky.0).then(kx.1.total_cmp(&ky.1))
            });
        match drill {
            None => warnings.push(format!("the bore (Ø{:.3} at its narrowest) is not machined: no drill in the job fits it", 2.0 * smallest)),
            Some(d) => {
                let dia = d.diameter.unwrap_or(0.0);
                hole = dia / 2.0;
                bore_end = hole;
                let tip = hole / (d.point_angle().to_radians() / 2.0).tan();
                let through = a == 0;
                // the point's tip, in work Z
                let bottom = if through { -l - back - tip - 0.5 } else { a as f64 * profile.dz - l };
                if !through {
                    warnings.push(format!("the blind bore's floor at Z{:.3} is left with the drill's {}° point", bottom, d.point_angle()));
                }
                let peck = d.peck.unwrap_or(dia);
                let start = s.face_allowance + clr;
                let mut mv = vec![TipMove::Rapid([0.0, start])];
                let mut at = 0.0f64.min(s.face_allowance);
                while at > bottom + 1e-9 {
                    let next = (at - peck).max(bottom);
                    mv.push(TipMove::Feed([0.0, next]));
                    mv.push(TipMove::Rapid([0.0, start]));
                    if next > bottom + 1e-9 {
                        mv.push(TipMove::Rapid([0.0, next + 0.5]));
                    }
                    at = next;
                }
                ops.push(LatheOp {
                    kind: LatheOpKind::Drilling,
                    name: format!("drilling Ø{}", crate::post::num(dia, 3)),
                    tool: d.number,
                    spindle: Spindle::Rpm(d.rpm(&s.material, dia, max_rpm)),
                    feed: d.feed,
                    moves: mv,
                });
            }
        }
    }

    // The outside: roughing now, finishing after the bore
    let outside = turn(&Turn {
        profile: &profile,
        top: r_bar,
        rough,
        finish,
        leave_radial: st.leave_radial,
        leave_axial: st.leave_axial,
        back,
        ahead,
        clr,
        face: true,
    });
    if !outside.rough.is_empty() {
        ops.push(LatheOp {
            kind: LatheOpKind::Roughing,
            name: "roughing".into(),
            tool: rough.number,
            spindle: css(rough),
            feed: rough.feed,
            moves: outside.rough,
        });
    }

    // Boring: the bore beyond the drilled hole, mirrored across the axis and turned like the outside
    if hole > 0.0 && profile.id.iter().any(|r| *r > hole + 0.01) {
        match job.first_of(LatheToolKind::Boring) {
            None => warnings.push(format!("Z {:.3} to 0.000: the bore is left as drilled: no boring bar in the job", -l)),
            Some(bar) if bar.min_bore.unwrap_or(0.0) > 2.0 * hole + 1e-6 => warnings.push(format!(
                "Z {:.3} to 0.000: the bore is left as drilled: T{} needs Ø{} to go in, the drill leaves Ø{}",
                -l,
                bar.number,
                crate::post::num(bar.min_bore.unwrap_or(0.0), 3),
                crate::post::num(2.0 * hole, 3)
            )),
            Some(bar) => {
                let deep = (0..n_samples)
                    .filter(|&i| profile.id.get(i).copied().unwrap_or(0.0) > hole + 0.01)
                    .map(|i| l - i as f64 * profile.dz)
                    .fold(0.0, f64::max);
                if deep > bar.reach() + 0.01 {
                    warnings.push(format!(
                        "Z {:.3} to {:.3}: the bore goes {deep:.1} deep; T{} reaches {}: left as drilled beyond",
                        -deep,
                        -bar.reach(),
                        bar.number,
                        crate::post::num(bar.reach(), 3)
                    ));
                } else if open.is_some_and(|(a, _)| a == 0) {
                    bore_end = profile.id.first().copied().unwrap_or(hole).max(hole);
                }
                let m = mirrored(&profile, hole, bar.reach());
                let inside = turn(&Turn {
                    profile: &m,
                    top: -hole,
                    rough: bar,
                    finish: bar,
                    leave_radial: st.leave_radial.min(bar.depth),
                    leave_axial: st.leave_axial,
                    back,
                    ahead,
                    clr: clr.min(hole * 0.5),
                    face: false,
                });
                let mut moves = unmirror(inside.rough);
                moves.extend(unmirror(inside.finish));
                if !moves.is_empty() {
                    ops.push(LatheOp {
                        kind: LatheOpKind::Boring,
                        name: "boring".into(),
                        tool: bar.number,
                        spindle: css(bar),
                        feed: bar.feed,
                        moves,
                    });
                }
                for (a, b, most) in left_runs(n_samples, |i| inside.cut.get(i).copied().unwrap_or(0.0) - inside.best.get(i).copied().unwrap_or(0.0)) {
                    warnings.push(format!(
                        "Z {:.3} to {:.3}: T{} can't reach into the bore there (an internal groove, or the bore widening toward the chuck): up to {most:.2} mm is left; it needs an internal grooving tool",
                        zw(a),
                        zw(b),
                        bar.number
                    ));
                }
            }
        }
    }

    if !outside.finish.is_empty() {
        ops.push(LatheOp {
            kind: LatheOpKind::Finishing,
            name: "finishing".into(),
            tool: finish.number,
            spindle: css(finish),
            feed: finish.feed,
            moves: outside.finish,
        });
    }

    // Grooving what the finishing tool can't reach, with the blade; what is left after it
    let mut surface_now = outside.cut.clone();
    let unreached = left_runs(n_samples, |i| outside.cut.get(i).copied().unwrap_or(0.0) - outside.best.get(i).copied().unwrap_or(0.0));
    let blade = job.first_of(LatheToolKind::Grooving);
    if let Some(bl) = blade
        && !unreached.is_empty()
    {
        let w = bl.blade_width();
        let k = |z: f64| ((z / profile.dz).round().max(0.0) as usize).min(n_samples.saturating_sub(1));
        let over = |zl: f64, f: &dyn Fn(usize) -> f64| (k(zl)..=k(zl + w)).map(f).fold(f64::NEG_INFINITY, f64::max);
        let od_at = |i: usize| profile.od.get(i).copied().unwrap_or(0.0);
        let mut mv: Vec<TipMove> = Vec::new();
        for (a, b, _) in &unreached {
            let (za, zb) = (*a as f64 * profile.dz, *b as f64 * profile.dz);
            let mut plunges: Vec<f64> = Vec::new();
            if zb - za <= w {
                // one plunge covering it, as low as the profile under the blade lets it go
                let n = ((w - (zb - za)) / profile.dz).ceil() as usize;
                let best = (0..=n)
                    .map(|j| zb - w + j as f64 * profile.dz)
                    .filter(|zl| *zl >= -back + 1e-9 && zl + w <= l + 1e-9)
                    .min_by(|x, y| over(*x, &od_at).total_cmp(&over(*y, &od_at)));
                plunges.extend(best);
            } else {
                let mut zl = zb - w;
                while zl > za + 1e-9 {
                    plunges.push(zl);
                    zl -= 0.8 * w;
                }
                plunges.push(za);
            }
            for zl in plunges {
                let bottom = over(zl, &od_at);
                let top = over(zl, &|i| surface_now.get(i).copied().unwrap_or(r_bar)).min(r_bar);
                if !(bottom < top - 0.005) {
                    continue;
                }
                let bottom = if top - bottom > bl.reach() {
                    warnings.push(format!("Z {:.3}: T{} reaches {}: the plunge stops short", zl - l, bl.number, crate::post::num(bl.reach(), 3)));
                    top - bl.reach()
                } else {
                    bottom
                };
                let zt = zl - l;
                let start = top + clr;
                mv.push(TipMove::Rapid([r_bar + clr, zt]));
                mv.push(TipMove::Rapid([start, zt]));
                let mut r = top;
                while r > bottom + 1e-9 {
                    r = (r - bl.depth).max(bottom);
                    mv.push(TipMove::Feed([r, zt]));
                    if r > bottom + 1e-9 {
                        mv.push(TipMove::Rapid([r + 0.2, zt]));
                    }
                }
                mv.push(TipMove::Rapid([start, zt]));
                mv.push(TipMove::Rapid([r_bar + clr, zt]));
                for i in k(zl)..=k(zl + w) {
                    if let Some(v) = surface_now.get_mut(i) {
                        *v = v.min(bottom);
                    }
                }
            }
        }
        if !mv.is_empty() {
            mv.push(TipMove::Rapid([r_bar + clr, s.face_allowance + clr]));
            ops.push(LatheOp { kind: LatheOpKind::Grooving, name: "grooving".into(), tool: bl.number, spindle: css(bl), feed: bl.feed, moves: mv });
        }
    }
    let after = if blade.is_some() { "the grooving blade" } else { "a grooving tool (none in the job)" };
    for (a, b, most) in left_runs(n_samples, |i| surface_now.get(i).copied().unwrap_or(0.0) - outside.best.get(i).copied().unwrap_or(0.0)) {
        warnings.push(format!(
            "Z {:.3} to {:.3}: up to {most:.2} mm is left that neither T{} nor {after} reaches (a V or slope steeper than they follow, or narrower than the blade)",
            zw(a),
            zw(b),
            finish.number
        ));
    }

    // Threading: radial passes in G33, degressive, and a spring pass
    let mut threads = Vec::new();
    if !job.part.threads.is_empty() {
        match job.first_of(LatheToolKind::Threading) {
            None => warnings.push("the threads are not cut: no threading tool in the job".into()),
            Some(th) => {
                let mut mv: Vec<TipMove> = Vec::new();
                for spec in &job.part.threads {
                    let (dia, pitch) = spec.size()?;
                    let major = dia / 2.0;
                    let depth = 0.61343 * pitch;
                    let minor = major - depth;
                    let turned =
                        (k_of(&profile, spec.z_from)..=k_of(&profile, spec.z_to)).filter_map(|i| profile.od.get(i)).copied().fold(0.0, f64::max);
                    if (turned - major).abs() > 0.15 {
                        warnings.push(format!(
                            "thread {}: the part is Ø{:.3} there, not Ø{}",
                            spec.designation,
                            2.0 * turned,
                            crate::post::num(dia, 3)
                        ));
                    }
                    // run in and out where the part is no wider than the thread's root (a relief
                    // groove, the air in front, the parting allowance behind), the tool's V clear
                    // of the shoulders beyond
                    let tan = (th.point_angle().to_radians() / 2.0).tan();
                    let in_part = |z: f64| (0.0..=l).contains(&z);
                    // how far from `from` (a step at a time) the part stays under the root, and the
                    // room the V then needs from the shoulder there (as tall as the shoulder, up to
                    // where its body stands up)
                    let room = |from: f64, step: f64| {
                        let mut z = from;
                        for _ in 0..100_000 {
                            let nz = z + step;
                            if !(nz > -back && nz < l + 3.0 * pitch + THREAD_V + 1.0) || (in_part(nz) && profile.od_at(nz) > minor + 0.01) {
                                break;
                            }
                            z = nz;
                        }
                        let k = ((THREAD_V * tan + 0.1) / profile.dz).ceil() as usize;
                        let wall =
                            (1..=k).map(|j| z + step * j as f64).filter(|z| in_part(*z)).map(|z| profile.od_at(z)).fold(f64::NEG_INFINITY, f64::max);
                        let need = if wall.is_finite() { (wall - minor).clamp(0.0, THREAD_V) * tan + 0.05 } else { 0.0 };
                        (z - from).abs() - need
                    };
                    // at most two pitches to run in, half a pitch to run out; short of the end
                    // where a shoulder stands at it
                    let run_in = room(spec.z_to, profile.dz).min(2.0 * pitch);
                    let run_out = room(spec.z_from, -profile.dz).min(0.5 * pitch);
                    for (run, end, z) in [(run_in, "front", spec.z_to), (run_out, "chuck", spec.z_from)] {
                        if run < -1e-9 {
                            warnings.push(format!(
                                "thread {}: no room for the tool at its {end} end: it stops {:.2} short of Z{:.3} (a relief groove to Ø{:.2} gives it room)",
                                spec.designation,
                                -run,
                                z - l,
                                2.0 * minor
                            ));
                        }
                    }
                    if (0.0..pitch).contains(&run_in) {
                        warnings.push(format!("thread {}: only {run_in:.2} mm to run in before it", spec.designation));
                    }
                    let (zs, ze) = (spec.z_to + run_in - l, spec.z_from - run_out - l);
                    if zs - ze < pitch {
                        warnings.push(format!("thread {}: not cut, no room for it", spec.designation));
                        continue;
                    }
                    let n = ((depth / th.depth).powi(2)).ceil().clamp(3.0, 30.0) as usize;
                    let out = major + clr;
                    // going in clear of the thread's start, rapid; else fed
                    let in_air = run_in > depth * tan + 0.05;

                    mv.push(TipMove::Rapid([r_bar + clr, zs]));
                    for kk in 1..=n + 1 {
                        let d = depth * ((kk.min(n)) as f64 / n as f64).sqrt();
                        mv.push(TipMove::Rapid([out, zs]));
                        if in_air {
                            mv.push(TipMove::Rapid([major - d, zs]));
                        } else {
                            mv.push(TipMove::Rapid([major + 0.2, zs]));
                            mv.push(TipMove::Feed([major - d, zs]));
                        }
                        mv.push(TipMove::Thread { to: [major - d, ze], pitch });
                        mv.push(TipMove::Rapid([out, ze]));
                    }
                    mv.push(TipMove::Rapid([out, zs]));
                    mv.push(TipMove::Rapid([r_bar + clr, zs]));
                    threads.push(CutThread { z: [ze, zs], major, minor, pitch, flank: depth * tan });
                }
                if !mv.is_empty() {
                    let d = threads.iter().map(|t| 2.0 * t.major).fold(0.0, f64::max);
                    mv.push(TipMove::Rapid([r_bar + clr, s.face_allowance + clr]));
                    ops.push(LatheOp {
                        kind: LatheOpKind::Threading,
                        name: "threading".into(),
                        tool: th.number,
                        spindle: Spindle::Rpm(th.rpm(&s.material, d, max_rpm)),
                        feed: th.feed,
                        moves: mv,
                    });
                }
            }
        }
    }

    // Parting off behind the part, the blade's face toward the front at the part's end
    match blade {
        None => warnings.push("the part is not parted off: no grooving or parting blade in the job".into()),
        Some(bl) => {
            let w = bl.blade_width();
            let top = profile.od.first().copied().unwrap_or(r_bar);
            let end = if bore_end > 0.0 && open.is_some_and(|(a, _)| a == 0) { bore_end - 0.5 } else { -0.2 };
            if w > back + 1e-9 {
                warnings.push(format!("the part is not parted off: T{} is {w} wide, the parting allowance {back}", bl.number));
            } else if top - end > bl.reach() {
                warnings.push(format!("the part is not parted off: Ø{:.1} is deeper than T{} reaches; part it off by hand", 2.0 * top, bl.number));
            } else {
                let zt = -l - w;
                let start = top + clr;
                let mut mv = vec![TipMove::Rapid([r_bar + clr, zt]), TipMove::Rapid([start, zt])];
                let mut r = top;
                while r > end + 1e-9 {
                    r = (r - bl.depth).max(end);
                    mv.push(TipMove::Feed([r, zt]));
                    if r > end + 1e-9 {
                        mv.push(TipMove::Rapid([r + 0.2, zt]));
                    }
                }
                mv.push(TipMove::Rapid([start, zt]));
                mv.push(TipMove::Rapid([r_bar + clr, zt]));
                ops.push(LatheOp { kind: LatheOpKind::Parting, name: "parting".into(), tool: bl.number, spindle: css(bl), feed: bl.feed, moves: mv });
            }
        }
    }

    // tools the turret does not hold
    let turret = &job.machine.turret;
    if !turret.is_empty() {
        let mut seen: Vec<u32> = Vec::new();
        for op in &ops {
            if !turret.contains(&op.tool) && !seen.contains(&op.tool) {
                seen.push(op.tool);
                warnings.push(format!("T{} is not one of the turret's stations: change it by hand", op.tool));
            }
        }
    }
    Ok(LathePlan { setup, profile, ops, warnings, threads })
}

/// The sample nearest the part's own `z`.
fn k_of(p: &Profile, z: f64) -> usize {
    p.index(z)
}
