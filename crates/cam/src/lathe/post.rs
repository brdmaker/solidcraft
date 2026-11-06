//! The LinuxCNC lathe program for a turning plan: XZ plane (G18), X as a diameter (G7), feed per
//! revolution (G95) and constant surface speed held under the machine's top speed (G96 … D). The
//! moves are the tools' imaginary tips, so the program runs without nose radius compensation
//! (G40): touch each tool off on that tip. Moves are straight lines; curves are within the plan's
//! path tolerance of their chords.

use super::job::{LatheJob, LatheToolKind, SpindleDirection, ToolSide};
use super::plan::{LathePlan, TipMove};
use crate::job::Coolant;
use crate::{CamError, Result};

fn num(x: f64, decimals: u32) -> String {
    crate::post::num(x, decimals)
}

/// The tool table comment for tool `n`.
pub fn tool_line(job: &LatheJob, plan: &LathePlan, n: u32, decimals: u32) -> String {
    let f = |x: f64| num(x, decimals);
    let Some(t) = job.tool(n) else { return format!("TOOL T{n}") };
    let kind = match t.kind {
        LatheToolKind::Turning => "turning",
    };
    let z_min = plan.ops.iter().filter(|o| o.tool == n).flat_map(|o| o.moves.iter()).map(|m| m.to()[1]).fold(f64::INFINITY, f64::min);
    let mut line = format!(
        "TOOL T{n} kind={kind} insert={} approach={} nose_radius={} feed={} depth={} surface_speed={}",
        f(t.insert_angle_deg),
        f(t.approach_deg),
        f(t.nose_radius),
        f(t.feed),
        f(t.depth),
        f(t.speed(&job.stock.material).unwrap_or(0.0))
    );
    if z_min.is_finite() {
        line.push_str(&format!(" z_min={}", f(z_min)));
    }
    let name = if t.name.is_empty() { format!("{kind} R{}", f(t.nose_radius)) } else { t.name.clone() };
    line.push_str(&format!(" name={name}"));
    line
}

/// The program.
pub fn linuxcnc(job: &LatheJob, plan: &LathePlan, title: &str) -> Result<String> {
    use std::fmt::Write as _;
    let d = job.post.decimals;
    let f = |x: f64| num(x, d);
    let dia = |r: f64| num(2.0 * r, d);
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
        f(job.stock.diameter),
        f(su.length),
        job.stock.material,
        f(job.stock.in_chuck),
        f(su.stick_out)
    )))?;
    out(crate::post::comment(&format!(
        "Z0 the finished front face; the part to Z{}, turned to Z{} for parting; jaws at Z{}; X is a diameter",
        f(su.part_end),
        f(su.turned_to),
        f(su.jaws)
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
    for op in &plan.ops {
        out(crate::post::comment(&op.name))?;
        if tool != Some(op.tool) {
            out(format!("G0 X{} Z{}", dia(change[0]), f(change[1])))?;
            out(format!("T{} M6 G43", op.tool))?;
            tool = Some(op.tool);
            motion = "";
        }
        out(format!("G96 S{} D{} {spindle}", f(op.surface_speed), f(m.max_rpm)))?;
        if let Some(c) = coolant {
            out(c.into())?;
        }
        out(format!("F{}", f(op.feed)))?;
        for mv in &op.moves {
            let (code, p) = match mv {
                TipMove::Rapid(p) => ("G0", p),
                TipMove::Feed(p) => ("G1", p),
            };
            let word = if code == motion { String::new() } else { format!("{code} ") };
            motion = code;
            out(format!("{word}X{} Z{}", dia(p[0]), f(p[1])))?;
        }
    }
    out(format!("G0 X{} Z{}", dia(change[0]), f(change[1])))?;
    out("M5 M9".into())?;
    out("M30".into())?;
    out("%".into())?;
    Ok(g)
}
