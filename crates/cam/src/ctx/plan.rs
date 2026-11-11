//! Planning a part on the twin: side 1 turns it from the bar (its front, the whole outside, the
//! bore open to the front, threads) and parts it off with chuck 2 holding it, leaving the back
//! allowance; side 2 faces the back to length and machines the bore open to the back.

use solidcraft_kernel::Body;

use super::job::CtxJob;
use crate::lathe::job::Front;
use crate::lathe::plan::{LatheOpKind, LathePlan, Scope, Spindle, TipMove};
use crate::lathe::profile::{Profile, read_envelope};
use crate::{CamError, Result};

/// Chuck 2 grips at most this much of the part by default, mm.
const GRIP_MAX: f64 = 20.0;
/// ... and at least this much.
const GRIP_MIN: f64 = 5.0;

#[derive(Clone, Debug)]
pub struct CtxPlan {
    /// Side 1: turning from the bar; its last operation parts the part off (with spindle 2
    /// following).
    pub side1: LathePlan,
    /// Side 2: the back, the part held by `grip` of its front in chuck 2.
    pub side2: LathePlan,
    pub grip: f64,
    pub warnings: Vec<String>,
}

impl CtxPlan {
    /// Side 1's parting operation, if the part is parted off.
    pub fn parting(&self) -> Option<&crate::lathe::plan::LatheOp> {
        self.side1.ops.iter().find(|o| o.kind == LatheOpKind::Parting)
    }
}

/// Only the bore that is open to the front end (the last sample) is kept.
fn front_bore_only(p: &mut Profile) {
    let n = p.id.len();
    let mut open = p.id.last().copied().unwrap_or(0.0) > 0.0;
    for i in (0..n).rev() {
        let Some(d) = p.id.get_mut(i) else { continue };
        if *d <= 0.0 {
            open = false;
        }
        if !open {
            *d = 0.0;
        }
    }
}

/// How much of the part chuck 2 holds: the job's, or half the part, from [`GRIP_MIN`] to
/// [`GRIP_MAX`] (no more than 80 % of it).
fn grip(job: &CtxJob, length: f64) -> f64 {
    job.transfer.grip.unwrap_or_else(|| (0.5 * length).clamp(GRIP_MIN, GRIP_MAX).min(0.8 * length))
}

/// Plan `job` for `body`.
pub fn plan(job: &CtxJob, body: &Body) -> Result<CtxPlan> {
    let front = job.part.front;
    let back = match front {
        Front::PlusZ => Front::MinusZ,
        Front::MinusZ => Front::PlusZ,
    };
    let mut p1 = read_envelope(body, front)?;
    let p2 = read_envelope(body, back)?;
    let l = p1.length;
    let mut warnings = Vec::new();
    // side 1 bores what is open to the front; side 2 what is open to the back, unless side 1
    // drills it through
    front_bore_only(&mut p1);
    let grip = grip(job, l);
    if grip > l - job.stock.chuck_clearance {
        warnings.push(format!("chuck 2 grips {grip:.1} of the part, {l:.1} long: its back is not clear of the jaws"));
    }
    let job1 = job.side_job(1, 0.0)?;
    let mut side1 = crate::lathe::plan::plan_with(
        &job1,
        p1.clone(),
        &Scope { outside: true, bore: true, part_off: true, back_extra: job.stock.back_allowance, second_side: false },
    )?;
    // parting with spindle 2 following: a fixed speed, under the transfer's limit
    let bar = job.stock.diameter;
    for op in side1.ops.iter_mut().filter(|o| o.kind == LatheOpKind::Parting) {
        let rpm = job1.tool(op.tool).map_or(job.machine.sync_max_rpm, |t| t.rpm(&job.stock.material, bar, job.machine.sync_max_rpm));
        op.spindle = Spindle::Rpm(rpm.min(job.machine.sync_max_rpm));
    }
    if side1.ops.iter().all(|o| o.kind != LatheOpKind::Parting) {
        return Err(CamError::Invalid(format!("the part can't be parted off on side 1: {}", side1.warnings.join("; "))));
    }
    // through the part on side 1: nothing left of the bore for side 2
    let through = side1.ops.iter().filter(|o| o.kind == LatheOpKind::Drilling).flat_map(|o| o.moves.iter()).any(|m| match m {
        TipMove::Feed(p) => p[1] <= -l - job.stock.back_allowance - 1e-6,
        _ => false,
    });
    let bored_on_1 = p1.id.first().copied().unwrap_or(0.0) > 0.0;
    let open_back = p2.id.last().copied().unwrap_or(0.0) > 0.0;
    let job2 = job.side_job(2, grip)?;
    let side2 = crate::lathe::plan::plan_with(
        &job2,
        p2,
        &Scope { outside: false, bore: open_back && !(through && bored_on_1), part_off: false, back_extra: 0.0, second_side: true },
    )?;
    for w in &side1.warnings {
        warnings.push(format!("side 1: {w}"));
    }
    for w in &side2.warnings {
        warnings.push(format!("side 2: {w}"));
    }
    Ok(CtxPlan { side1, side2, grip, warnings })
}

/// Roughly how long `ops` take on `job`'s lathe, s: feeds at their spindle's speed at each
/// move's diameter, rapids at 10 m/min, a tool change 2 s.
pub fn estimate(job: &crate::lathe::job::LatheJob, ops: &[crate::lathe::plan::LatheOp]) -> f64 {
    let mut t = 0.0;
    let mut at: Option<[f64; 2]> = None;
    let mut tool = None;
    for op in ops {
        if tool != Some(op.tool) {
            t += 2.0;
            tool = Some(op.tool);
        }
        for m in &op.moves {
            let to = m.to();
            let from = at.unwrap_or(to);
            let len = (to[0] - from[0]).hypot(to[1] - from[1]);
            let r = from[0].abs().max(to[0].abs()).max(1.0);
            let rpm = match op.spindle {
                Spindle::Rpm(n) => n,
                Spindle::SurfaceSpeed(v) => (v * 1000.0 / (std::f64::consts::PI * 2.0 * r)).min(job.machine.max_rpm),
            }
            .max(1.0);
            t += match m {
                TipMove::Rapid(_) => len / 10_000.0 * 60.0,
                TipMove::Feed(_) => len / (op.feed * rpm) * 60.0,
                TipMove::Thread { pitch, .. } => len / (pitch * rpm) * 60.0,
            };
            at = Some(to);
        }
    }
    t
}
