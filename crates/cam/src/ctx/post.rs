//! The two LinuxCNC programs for the twin, one per side, following its programming manual
//! (brdmot `linuxcnc/ctxtwin/docs/programming.md`): the lathe's modes and blocks
//! ([`crate::lathe::post`]), each side's `o<twin_init>`, chuck 1 clamped on the bar, T`n` M6 G43
//! with `n` the station, G96 with its D limit. Side 1 brings the parting blade to the cut before
//! the transfer (the turret doesn't turn with chuck 2 near), meets side 2 at sync point 3 (side 2
//! waits there while side 1 machines), calls `o<twin_grip>`, parts off at a fixed speed with
//! spindle 2 following and calls `o<twin_cut_done>`. Side 2 waits at sync point 3, calls
//! `o<twin_catch>` and machines the back.

use super::job::CtxJob;
use super::live::{LiveKind, LiveMove, LiveOp, R_MIN};
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
    let t = job1.map_or(0.0, |j| estimate(&j, &plan.side1.ops)) + plan.live1.iter().map(super::live::estimate).sum::<f64>();
    (2.0 * t + 120.0).max(600.0).ceil()
}

fn header(job: &CtxJob, lathe: &LatheJob, plan: &LathePlan, live: &[LiveOp], side: u8, title: &str, warnings: &[String]) -> Vec<String> {
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
    let mut seen: Vec<u32> = Vec::new();
    for op in live {
        if seen.contains(&op.tool) {
            continue;
        }
        seen.push(op.tool);
        if let Some(t) = job.live(op.tool) {
            let mount = match t.mount {
                super::job::Mount::Axial => "axial",
                super::job::Mount::Radial => "radial",
            };
            let kind = match t.kind {
                crate::job::ToolKind::Drill => "drill",
                _ => "end_mill",
            };
            let name = if t.name.is_empty() { kind.to_string() } else { t.name.clone() };
            g.push(comment(&format!(
                "TOOL T{} live {kind} {mount} diameter={} flutes={} rpm={} feed={} plunge={} name={name}",
                t.number,
                plain(t.diameter),
                t.flutes,
                plain(op.rpm),
                plain(t.feed),
                plain(t.plunge)
            )));
        }
    }
    if !live.is_empty() {
        g.push(comment(if side == 1 {
            "live tools: C0 puts the part's +X at the tool; in polar interpolation X Y are the part's own on its front face"
        } else {
            "live tools: the part turned end for end about its Y axis: X Y on the back face are the part's -X and Y; check C's zero against side 1's"
        }));
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

/// The blocks of a live tool's operation: the tool, C on, milling mode, the live tool turning,
/// its moves, then all off again and feed per revolution back.
fn live_blocks(job: &CtxJob, lathe: &LatheJob, op: &LiveOp, w: &mut OpWriter) -> Vec<String> {
    let d = job.post.lathe.decimals;
    let f = |x: f64| word(x, d);
    let mut g = vec![comment(&op.name)];
    g.extend(w.tool_change(lathe, op.tool));
    let change = w.change;
    g.push("M14".into());
    g.push("G94".into());
    g.push("M16".into());
    g.push(format!("M3 $1 S{}", num(op.rpm.round(), 0)));
    let clear = CLEAR_Z;
    match op.kind {
        LiveKind::AxialDrill => {
            g.push(format!("G0 Z{}", f(clear)));
            g.push("G17".into());
            for h in &op.holes {
                g.push(format!("G0 C{}", f(h.c)));
                g.push(format!("G0 X{}", f(2.0 * h.r)));
                let cycle = match op.peck {
                    Some(q) => format!("G98 G83 X{} Z{} R{} Q{} F{}", f(2.0 * h.r), f(h.to), f(h.z + 1.0), f(q), f(op.plunge)),
                    None => format!("G98 G81 X{} Z{} R{} F{}", f(2.0 * h.r), f(h.to), f(h.z + 1.0), f(op.plunge)),
                };
                g.push(cycle);
                g.push("G80".into());
            }
            g.push("G18".into());
            g.push(format!("G0 X{}", f(2.0 * change[0])));
        }
        LiveKind::RadialDrill => {
            let out = op.holes.iter().map(|h| h.r).fold(0.0, f64::max) + 2.0;
            g.push("G8".into());
            g.push(format!("G0 X{}", f(out.max(change[0]))));
            if let Some(h) = op.holes.first() {
                g.push(format!("G0 Z{}", f(h.z)));
            }
            g.push("G19".into());
            for h in &op.holes {
                g.push(format!("G0 C{}", f(h.c)));
                let cycle = match op.peck {
                    Some(q) => format!("G98 G83 X{} R{} Q{} F{}", f(h.to), f(h.r + 1.0), f(q), f(op.plunge)),
                    None => format!("G98 G81 X{} R{} F{}", f(h.to), f(h.r + 1.0), f(op.plunge)),
                };
                g.push(cycle);
                g.push("G80".into());
            }
            g.push("G18 G7".into());
            g.push(format!("G0 X{}", f(2.0 * change[0])));
        }
        LiveKind::Keyway => {
            g.push(format!("G0 C{}", f(op.c)));
            if let Some(first) = op.moves.first().map(LiveMove::to) {
                g.push(format!("G0 Z{}", f(first[2])));
                g.push(format!("G0 X{}", f(2.0 * first[0])));
            }
            for m in &op.moves {
                match m {
                    LiveMove::Rapid(p) => g.push(format!("G0 X{} Z{}", f(2.0 * p[0]), f(p[2]))),
                    LiveMove::Feed(p, fr) => g.push(format!("G1 X{} Z{} F{}", f(2.0 * p[0]), f(p[2]), f(*fr))),
                }
            }
            g.push(format!("G0 X{}", f(2.0 * change[0])));
        }
        LiveKind::FaceMill => {
            g.push(format!("G0 Z{}", f(clear)));
            // the first move is where polar interpolation comes on: off the centre, on X (lathe
            // coordinates, a diameter)
            let skip = match op.moves.first() {
                Some(LiveMove::Rapid(p)) => {
                    g.push(format!("G0 X{} Z{}", f(2.0 * p[0].hypot(p[1]).max(R_MIN + 1.0)), f(p[2])));
                    1
                }
                _ => 0,
            };
            g.push("G12.1".into());
            let mut last_f: Option<f64> = None;
            for m in op.moves.iter().skip(skip) {
                match m {
                    LiveMove::Rapid(p) => g.push(format!("G0 X{} Y{} Z{}", f(p[0]), f(p[1]), f(p[2]))),
                    LiveMove::Feed(p, fr) => {
                        let fr = fr.max(1.0).round();
                        if last_f.is_none_or(|l| (fr - l).abs() > 0.5) {
                            g.push(format!("G1 X{} Y{} Z{} F{}", f(p[0]), f(p[1]), f(p[2]), f(fr)));
                            last_f = Some(fr);
                        } else {
                            g.push(format!("G1 X{} Y{} Z{}", f(p[0]), f(p[1]), f(p[2])));
                        }
                    }
                }
            }
            g.push("G13.1".into());
            g.push(format!("G0 X{}", f(2.0 * change[0])));
        }
    }
    g.push("M17".into());
    g.push("M15".into());
    g.push("G95".into());
    w.lost();
    g
}

/// Live tools move between cuts this far in front of the face, mm.
const CLEAR_Z: f64 = 2.0;

/// Side 1's program.
pub fn side1(job: &CtxJob, plan: &CtxPlan, title: &str) -> Result<String> {
    let lathe = job.side_job(1, 0.0)?;
    let p = &plan.side1;
    let d = job.post.lathe.decimals;
    let f = |x: f64| word(x, d);
    let mut warnings: Vec<String> = plan.warnings.clone();
    station_check(job, p, &mut warnings);
    let mut g = header(job, &lathe, p, &plan.live1, 1, title, &warnings);
    g.push("M10".into());
    let su = &p.setup;
    let change = [su.radius + job.strategy.change_clearance, su.front + job.strategy.change_clearance];
    let mut w = OpWriter::new(change);
    let parting = p.ops.iter().find(|o| o.kind == LatheOpKind::Parting).ok_or_else(|| CamError::Invalid("side 1 doesn't part off".into()))?;
    for op in p.ops.iter().filter(|o| o.kind != LatheOpKind::Parting) {
        g.extend(w.op(&lathe, op));
    }
    for op in &plan.live1 {
        g.extend(live_blocks(job, &lathe, op, &mut w));
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
    let mut g = header(job, &lathe, p, &plan.live2, 2, title, &warnings);
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
    for op in &plan.live2 {
        g.extend(live_blocks(job, &lathe, op, &mut w));
    }
    g.push(format!("G0 X{} Z{}", f(2.0 * change[0]), f(change[1])));
    g.push("M5".into());
    g.push("G97 S0".into());
    g.push("M30".into());
    g.push("%".into());
    Ok(g.join("\n") + "\n")
}
