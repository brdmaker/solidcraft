//! Turning a part: facing the bar's front, roughing it down in radial steps and finishing the
//! profile, all with turning tools cutting toward the chuck.
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

use super::job::{LatheJob, LatheTool};
use super::profile::Profile;
use crate::{CamError, Result};

/// Tolerance of the paths' simplification, mm.
pub const PATH_TOL: f64 = 0.002;

/// One move of the tool's tip, in work coordinates `[x (radius), z]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TipMove {
    Rapid([f64; 2]),
    Feed([f64; 2]),
}

impl TipMove {
    pub fn to(&self) -> [f64; 2] {
        match self {
            TipMove::Rapid(p) | TipMove::Feed(p) => *p,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LatheOpKind {
    Facing,
    Roughing,
    Finishing,
}

/// An operation: one tool, its surface speed (m/min) and feed (mm/rev), its moves.
#[derive(Clone, Debug)]
pub struct LatheOp {
    pub kind: LatheOpKind,
    pub name: String,
    pub tool: u32,
    pub surface_speed: f64,
    pub feed: f64,
    pub moves: Vec<TipMove>,
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
    let speed = |t: &LatheTool| t.speed(&s.material).unwrap_or(100.0);
    let mut ops = Vec::new();
    let clr = st.clearance;

    // the reaches: roughing's with the allowances, finishing's to size
    let back = s.parting_allowance;
    let ahead = st.leave_axial + clr + 1.0;
    let rr = reach(&profile, rough.nose_radius, rough.descent_deg(), back, ahead);
    let rl = leave(&rr, st.leave_axial, st.leave_radial);
    let fr = reach(&profile, finish.nose_radius, finish.descent_deg(), back, ahead);
    // the part's front face in the grids, the nose resting on it
    let front_g = |r: &Reach, rn: f64| (((l + rn) - r.z0) / r.dz).round().max(0.0) as usize;

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
                surface_speed: speed(rough),
                feed: rough.feed,
                moves: m.out,
            });
        }
    }

    // Roughing: levels of the nose centre from the bar's surface down to the reach with the
    // allowances, at most the tool's depth apart; each level cuts every run where the reach lies
    // below it, the last stretch climbing the profile back up the step it leaves
    {
        let rn = rough.nose_radius;
        let top = r_bar + rn;
        let g_lo = 0usize;
        // to the grid's end, open air in front of the part
        let g_front = rl.c.len().saturating_sub(1);
        // the lowest the part itself holds it (in front of its face it may come lower: air)
        let low = (g_lo..=g_front).filter(|&g| rl.z(g) <= l).map(|g| rl.at(g)).filter(|v| v.is_finite()).fold(f64::INFINITY, f64::min);
        let mut m = Moves { rn, length: l, out: Vec::new() };
        if low.is_finite() && top - low > 1e-6 {
            let n = ((top - low) / rough.depth).ceil().max(1.0) as usize;
            let step = (top - low) / n as f64;
            let z_start = l + st.leave_axial + rn + clr;
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
                        // open to the front
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
                    }
                    if gb < g_front {
                        path.push([rho, rl.z(gb)]);
                    }
                    path.push([rho, za]);
                    // up the step behind, as far as the level above
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
                    m.feed(&[[top_reached + 0.5 * step.min(1.0), last[1] + 0.5 * step.min(1.0)]]);
                    m.rapid(safe, last[1] + 0.5 * step.min(1.0));
                }
            }
            m.rapid(safe, z_start);
        }
        if !m.out.is_empty() {
            ops.push(LatheOp {
                kind: LatheOpKind::Roughing,
                name: "roughing".into(),
                tool: rough.number,
                surface_speed: speed(rough),
                feed: rough.feed,
                moves: m.out,
            });
        }
    }

    // Finishing: the profile to size, from the front face's middle out and along to the parting
    // allowance; where roughing left more than the tool's depth (slopes only it reaches), passes
    // standing off evenly before it
    {
        let rn = finish.nose_radius;
        let g_front = front_g(&fr, rn).min(fr.c.len().saturating_sub(1));
        let grid = |g: usize| fr.z(g);
        // what roughing left above the finished surface, in the finishing grid
        let rough_at = |z: f64| {
            let g = ((z - rl.z0) / rl.dz).round();
            if g.is_finite() && g >= 0.0 { rl.at(g as usize) - rough.nose_radius } else { f64::NEG_INFINITY }
        };
        let gap = (0..=g_front)
            .map(|g| {
                let (a, b) = (rough_at(grid(g)), fr.at(g) - rn);
                if a.is_finite() && b.is_finite() { a - b } else { 0.0 }
            })
            .fold(0.0, f64::max);
        let n = (gap / finish.depth).ceil().max(1.0) as usize;
        let d = gap / n as f64;
        let mut m = Moves { rn, length: l, out: Vec::new() };
        let safe = r_bar + rn + clr;
        let z_face = l + rn;
        for j in (0..n).rev() {
            let off = d * j as f64;
            let path: Vec<[f64; 2]> = (0..=g_front)
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
            if j == 0 {
                // the front face, from the middle out
                m.rapid(0.0, z_face + clr);
                m.feed(&[[0.0, z_face], [first[0], z_face]]);
            } else {
                m.rapid(first[0], z_face + clr);
                m.feed(&[[first[0], z_face]]);
            }
            m.feed(&path);
            let last = path.last().copied().unwrap_or(first);
            m.feed(&[[last[0] + clr, last[1] + clr]]);
            m.rapid(safe, last[1] + clr);
        }
        m.rapid(safe, z_face + clr);
        if !m.out.is_empty() {
            ops.push(LatheOp {
                kind: LatheOpKind::Finishing,
                name: "finishing".into(),
                tool: finish.number,
                surface_speed: speed(finish),
                feed: finish.feed,
                moves: m.out,
            });
        }
        // what the finishing tool's insert cannot reach: beyond what its nose alone would leave
        // (the round in an inside corner) if it could fall at any angle
        let cut = surface(&fr, rn, &profile);
        let free = reach(&profile, rn, 89.9, back, ahead);
        let best = surface(&free, rn, &profile);
        let left = |i: usize| cut.get(i).copied().unwrap_or(0.0) - best.get(i).copied().unwrap_or(0.0);
        for (a, b) in runs(0, profile.od.len().saturating_sub(1), |i| left(i) > 0.01) {
            let most = (a..=b).map(left).fold(0.0, f64::max);
            warnings.push(format!(
                "Z {:.3} to {:.3}: T{} can't reach into the profile there (a groove, an undercut or a slope steeper than {:.0}°): up to {:.2} mm is left; it needs a grooving or form tool",
                a as f64 * profile.dz - l,
                b as f64 * profile.dz - l,
                finish.number,
                finish.descent_deg(),
                most
            ));
        }
    }
    // bores: no drill or boring bar yet
    for (a, b) in runs(0, profile.id.len().saturating_sub(1), |i| profile.id.get(i).copied().unwrap_or(0.0) > 0.0) {
        let most = (a..=b).filter_map(|i| profile.id.get(i)).copied().fold(0.0, f64::max);
        warnings.push(format!(
            "Z {:.3} to {:.3}: a bore up to Ø{:.3} is not machined (no drill or boring bar in the job)",
            a as f64 * profile.dz - l,
            b as f64 * profile.dz - l,
            2.0 * most
        ));
    }
    // tools the turret does not hold
    let turret = &job.machine.turret;
    if !turret.is_empty() {
        let mut seen: Vec<u32> = Vec::new();
        for op in &ops {
            if !turret.contains(&op.tool) && !seen.contains(&op.tool) {
                seen.push(op.tool);
                warnings.push(format!("T{} is not one of the turret's stations", op.tool));
            }
        }
    }
    Ok(LathePlan { setup, profile, ops, warnings })
}
