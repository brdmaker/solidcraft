//! The LinuxCNC lathe program for a turning plan: XZ plane (G18), X as a diameter (G7), feed per
//! revolution (G95). Turning, boring, grooving and parting run at constant surface speed held
//! under the machine's (and the tool's) top speed (G96 … D); drilling and threading at a fixed
//! speed (G97). Threads are cut a pass at a time with G33 (X set by the rapid before it), drills
//! peck with plain feeds and rapids. The moves are the tools' touch-off points (turning tools'
//! and boring bars' imaginary tips), so the program runs without nose radius compensation (G40).
//! Moves are straight lines; curves are within the plan's path tolerance of their chords.
//! Coordinates and feeds always carry a decimal point (`X20.`), as the Emco's programming manual
//! asks.

use super::job::{LatheJob, LatheToolKind, SpindleDirection, ToolSide};
use super::plan::{LathePlan, Spindle, TipMove};
use crate::job::Coolant;
use crate::{CamError, Result};

fn num(x: f64, decimals: u32) -> String {
    crate::post::num(x, decimals)
}

/// The tool table comment for tool `n`.
pub fn tool_line(job: &LatheJob, plan: &LathePlan, n: u32, decimals: u32) -> String {
    let f = |x: f64| num(x, decimals);
    let Some(t) = job.tool(n) else { return format!("TOOL T{n}") };
    let kind = t.kind.name();
    let z_min = plan.ops.iter().filter(|o| o.tool == n).flat_map(|o| o.moves.iter()).map(|m| m.to()[1]).fold(f64::INFINITY, f64::min);
    let mut line = format!("TOOL T{n} kind={kind}");
    match t.kind {
        LatheToolKind::Turning | LatheToolKind::Boring => {
            line.push_str(&format!(" insert={} approach={} nose_radius={}", f(t.insert_angle_deg), f(t.approach_deg), f(t.nose_radius)));
            if let Some(b) = t.min_bore {
                line.push_str(&format!(" min_bore={}", f(b)));
            }
        }
        LatheToolKind::Grooving => line.push_str(&format!(" width={} nose_radius={}", f(t.blade_width()), f(t.nose_radius))),
        LatheToolKind::Threading => line.push_str(&format!(" tip_angle={} nose_radius={}", f(t.point_angle()), f(t.nose_radius))),
        LatheToolKind::Drill => {
            line.push_str(&format!(" diameter={} tip_angle={}", f(t.diameter.unwrap_or(0.0)), f(t.point_angle())));
            if let Some(p) = t.peck {
                line.push_str(&format!(" peck={}", f(p)));
            }
        }
    }
    if matches!(t.kind, LatheToolKind::Grooving | LatheToolKind::Boring) {
        line.push_str(&format!(" reach={}", f(t.reach())));
    }
    line.push_str(&format!(" feed={} depth={} surface_speed={}", f(t.feed), f(t.depth), f(t.speed(&job.stock.material).unwrap_or(0.0))));
    if let Some(r) = t.max_rpm {
        line.push_str(&format!(" max_rpm={}", f(r)));
    }
    if z_min.is_finite() {
        line.push_str(&format!(" z_min={}", f(z_min)));
    }
    let name = if t.name.is_empty() { kind.to_string() } else { t.name.clone() };
    line.push_str(&format!(" name={name}"));
    line
}

/// The program.
pub fn linuxcnc(job: &LatheJob, plan: &LathePlan, title: &str) -> Result<String> {
    use std::fmt::Write as _;
    let d = job.post.decimals;
    let f = |x: f64| crate::post::word(x, d);
    let dia = |r: f64| crate::post::word(2.0 * r, d);
    let plain = |x: f64| num(x, d);
    let mut g = String::new();
    let mut out = |s: String| -> Result<()> { writeln!(g, "{s}").map_err(|e| CamError::Invalid(e.to_string())) };
    let su = &plan.setup;
    let m = &job.machine;
    out("%".into())?;
    out(crate::post::comment(&format!("SolidCraft lathe CAM: {title}")))?;
    out(crate::post::comment(&format!(
        "{}, {} tool lathe; bar D{} x {} ({}), {} in the chuck, {} out",
        m.name,
        if m.tool_side == ToolSide::Front { "front" } else { "back" },
        plain(job.stock.diameter),
        plain(su.length),
        job.stock.material,
        plain(job.stock.in_chuck),
        plain(su.stick_out)
    )))?;
    out(crate::post::comment(&format!(
        "Z0 the finished front face; the part to Z{}, turned to Z{} for parting; jaws at Z{}; X is a diameter",
        plain(su.part_end),
        plain(su.turned_to),
        plain(su.jaws)
    )))?;
    out(crate::post::comment("tools are driven on their imaginary tip: touch them off there, no nose radius compensation"))?;
    let mut order: Vec<u32> = Vec::new();
    for op in &plan.ops {
        if !order.contains(&op.tool) {
            order.push(op.tool);
        }
    }
    for n in &order {
        out(crate::post::comment(&tool_line(job, plan, *n, d)))?;
    }
    for w in &plan.warnings {
        for chunk in w.as_bytes().chunks(200) {
            out(crate::post::comment(&format!("WARNING: {}", String::from_utf8_lossy(chunk))))?;
        }
    }
    out("G18 G7 G21 G90 G40 G80 G95".into())?;
    out("G54".into())?;
    out(format!("G64 P{}", f(job.post.blend_tolerance)))?;
    let change = [su.radius + job.strategy.change_clearance, su.front + job.strategy.change_clearance];
    let spindle = match m.spindle {
        SpindleDirection::M3 => "M3",
        SpindleDirection::M4 => "M4",
    };
    let coolant = match job.post.coolant {
        Coolant::Off => None,
        Coolant::Mist => Some("M7"),
        Coolant::Flood => Some("M8"),
    };
    let mut tool = None;
    let mut motion = "";
    // the last point written: unchanged axes are not repeated
    let mut last: Option<[f64; 2]> = None;
    for op in &plan.ops {
        out(crate::post::comment(&op.name))?;
        if tool != Some(op.tool) {
            out(format!("G0 X{} Z{}", dia(change[0]), f(change[1])))?;
            out(format!("T{} M6 G43", op.tool))?;
            tool = Some(op.tool);
            motion = "";
            last = Some(change);
        }
        match op.spindle {
            Spindle::SurfaceSpeed(v) => {
                let top = job.tool(op.tool).and_then(|t| t.max_rpm).map_or(m.max_rpm, |r| r.min(m.max_rpm));
                out(format!("G96 S{} D{} {spindle}", plain(v), plain(top)))?
            }
            Spindle::Rpm(r) => out(format!("G97 S{} {spindle}", plain(r.min(m.max_rpm).round())))?,
        }
        if let Some(c) = coolant {
            out(c.into())?;
        }
        out(format!("F{}", f(op.feed)))?;
        for mv in &op.moves {
            let (code, p) = match mv {
                TipMove::Rapid(p) => ("G0", p),
                TipMove::Feed(p) => ("G1", p),
                TipMove::Thread { to, pitch } => {
                    // synchronised with the spindle, one pass along Z (X set by the move before)
                    out(format!("G33 Z{} K{}", f(to[1]), f(*pitch)))?;
                    motion = "G33";
                    last = Some(*to);
                    continue;
                }
            };
            let word = if code == motion { String::new() } else { format!("{code} ") };
            let x = if last.is_some_and(|q| (q[0] - p[0]).abs() < 1e-9) { String::new() } else { format!("X{} ", dia(p[0])) };
            let z = if last.is_some_and(|q| (q[1] - p[1]).abs() < 1e-9) { String::new() } else { format!("Z{}", f(p[1])) };
            last = Some(*p);
            if x.is_empty() && z.is_empty() {
                continue;
            }
            motion = code;
            out(format!("{word}{x}{z}").trim_end().to_string())?;
        }
    }
    out(format!("G0 X{} Z{}", dia(change[0]), f(change[1])))?;
    out("M5 M9".into())?;
    out("M30".into())?;
    out("%".into())?;
    Ok(g)
}
