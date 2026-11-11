//! The two LinuxCNC programs for the twin, one per side, following its programming manual
//! (brdmot `linuxcnc/ctxtwin/docs/programming.md`): the lathe's modes and blocks
//! ([`crate::lathe::post`]), each side's `o<twin_init>`, chuck 1 clamped on the bar, T`n` M6 G43
//! with `n` the station, G96 with its D limit. Side 1 brings the parting blade to the cut before
//! the transfer (the turret doesn't turn with chuck 2 near), meets side 2 at sync point 3 (side 2
//! waits there while side 1 machines), calls `o<twin_grip>`, parts off at a fixed speed with
//! spindle 2 following and calls `o<twin_cut_done>`. Side 2 waits at sync point 3, calls
//! `o<twin_catch>` and machines the back.

use super::job::CtxJob;
use super::plan::{CtxPlan, estimate};
use crate::lathe::job::LatheJob;
use crate::lathe::plan::{LatheOpKind, LathePlan, TipMove};
use crate::lathe::post::{OpWriter, tool_line};
use crate::post::{comment, num, word};
use crate::{CamError, Result};

/// The sync point both programs meet at before the transfer.
pub const TRANSFER_SYNC: u32 = 3;

/// How long side 2 waits at the transfer's sync point, s.
pub fn transfer_wait(job: &CtxJob, plan: &CtxPlan) -> f64 {
    if let Some(w) = job.post.transfer_wait {
        return w;
    }
    let job1 = job.side_job(1, 0.0).ok();
    let t = job1.map_or(0.0, |j| estimate(&j, &plan.side1.ops));
    (2.0 * t + 120.0).max(600.0).ceil()
}

fn header(job: &CtxJob, lathe: &LatheJob, plan: &LathePlan, side: u8, title: &str, warnings: &[String]) -> Vec<String> {
    let d = job.post.lathe.decimals;
    let plain = |x: f64| num(x, d);
    let su = &plan.setup;
    let mut g = vec!["%".to_string(), comment(&format!("SolidCraft turn-mill CAM: {title}, side {side}"))];
    g.push(comment(&format!(
        "{}, side {side} of 2; bar D{} {}, X is a diameter, tools on their imaginary tips, no nose radius compensation",
        job.machine.name,
        plain(job.stock.diameter),
        job.stock.material
    )));
    if side == 1 {
        g.push(comment(&format!(
            "Z0 the finished front face; the bar {} out of chuck 1, faced from Z{}; the part to Z{}, parted off at Z{}",
            plain(su.stick_out),
            plain(su.front),
            plain(su.part_end),
            plain(su.part_end - job.stock.back_allowance)
        )));
    } else {
        g.push(comment(&format!(
            "Z0 the finished back face; the part in chuck 2 to Z{}, its jaws at Z{}; faced from Z{}",
            plain(su.part_end),
            plain(su.jaws),
            plain(su.front)
        )));
    }
    let mut order: Vec<u32> = Vec::new();
    for op in &plan.ops {
        if !order.contains(&op.tool) {
            order.push(op.tool);
        }
    }
    for n in &order {
        g.push(comment(&tool_line(lathe, plan, *n, d)));
    }
    for w in warnings {
        for chunk in w.as_bytes().chunks(200) {
            g.push(comment(&format!("WARNING: {}", String::from_utf8_lossy(chunk))));
        }
    }
    g.push("G18 G7 G21 G90 G40 G80 G95".into());
    let work = if side == 1 { job.post.work_z_1 } else { job.post.work_z_2 };
    if let Some(z) = work {
        g.push(format!("G10 L2 P1 Z{}", word(z, d)));
    }
    g.push("G54".into());
    g.push(format!("G64 P{}", word(job.post.lathe.blend_tolerance, d)));
    g.push("o<twin_init> call".into());
    g
}

/// Tool numbers as stations: T`n` with `n` the station's (the manual's T mod 12).
fn station_check(job: &CtxJob, plan: &LathePlan, warnings: &mut Vec<String>) {
    let n = job.machine.stations;
    for op in &plan.ops {
        if op.tool > n && !warnings.iter().any(|w| w.starts_with(&format!("T{} ", op.tool))) {
            warnings.push(format!("T{} turns the turret to station {}: the tool must be there", op.tool, job.station(op.tool)));
        }
    }
}

/// Side 1's program.
pub fn side1(job: &CtxJob, plan: &CtxPlan, title: &str) -> Result<String> {
    let lathe = job.side_job(1, 0.0)?;
    let p = &plan.side1;
    let d = job.post.lathe.decimals;
    let f = |x: f64| word(x, d);
    let mut warnings: Vec<String> = plan.warnings.clone();
    station_check(job, p, &mut warnings);
    let mut g = header(job, &lathe, p, 1, title, &warnings);
    g.push("M10".into());
    let su = &p.setup;
    let change = [su.radius + job.strategy.change_clearance, su.front + job.strategy.change_clearance];
    let mut w = OpWriter::new(change);
    let parting = p.ops.iter().find(|o| o.kind == LatheOpKind::Parting).ok_or_else(|| CamError::Invalid("side 1 doesn't part off".into()))?;
    for op in p.ops.iter().filter(|o| o.kind != LatheOpKind::Parting) {
        g.extend(w.op(&lathe, op));
    }
    // the blade to the cut, out at the bar's radius, before chuck 2 comes
    g.push(comment(&format!("{}: spindle 2 grips the part and follows", parting.name)));
    g.extend(w.tool_change(&lathe, parting.tool));
    let first = parting.moves.first().map(TipMove::to).unwrap_or(change);
    g.push(format!("G0 Z{}", f(first[1])));
    g.push(format!("o<twin_sync> call [{TRANSFER_SYNC}] [{}]", word(transfer_wait(job, plan), 0)));
    g.push("o<twin_grip> call".into());
    w.lost();
    // twin_grip leaves X at the clearance: back down at the cut's Z only
    g.push(OpWriter::spindle(&lathe, parting));
    g.push("G95".into());
    g.push(format!("F{}", f(parting.feed)));
    g.extend(w.moves(&lathe, parting));
    g.push("o<twin_cut_done> call".into());
    g.push(format!("G0 X{} Z{}", f(2.0 * change[0]), f(change[1])));
    g.push("M5".into());
    g.push("G97 S0".into());
    g.push("M30".into());
    g.push("%".into());
    Ok(g.join("\n") + "\n")
}

/// Side 2's program.
pub fn side2(job: &CtxJob, plan: &CtxPlan, title: &str) -> Result<String> {
    let lathe = job.side_job(2, plan.grip)?;
    let p = &plan.side2;
    let d = job.post.lathe.decimals;
    let f = |x: f64| word(x, d);
    let mut warnings: Vec<String> = Vec::new();
    station_check(job, p, &mut warnings);
    let mut g = header(job, &lathe, p, 2, title, &warnings);
    g.push(comment("waits here while side 1 machines the front"));
    g.push(format!("o<twin_sync> call [{TRANSFER_SYNC}] [{}]", word(transfer_wait(job, plan), 0)));
    match job.transfer.pickoff_z {
        Some(z) => g.push(format!("o<twin_catch> call [{}]", f(z))),
        None => g.push("o<twin_catch> call".into()),
    }
    let su = &p.setup;
    let change = [su.radius + job.strategy.change_clearance, su.front + job.strategy.change_clearance];
    let mut w = OpWriter::new(change);
    g.push("G95".into());
    for op in &p.ops {
        g.extend(w.op(&lathe, op));
    }
    g.push(format!("G0 X{} Z{}", f(2.0 * change[0]), f(change[1])));
    g.push("M5".into());
    g.push("G97 S0".into());
    g.push("M30".into());
    g.push("%".into());
    Ok(g.join("\n") + "\n")
}
