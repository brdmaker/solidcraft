//! The LinuxCNC post-processor: a planned job as an RS274/NGC program.
//!
//! The program opens with a comment block: the parts, the stock and work zero, then a tool
//! table, one `(TOOL T<n> key=value ... name=...)` line per tool in order of first use (see
//! [`tool_line`]), for a machine's own screen to read.
//!
//! Metric, absolute, XY plane, feed per minute, work offset G54 with work zero where
//! `stock.origin` puts it (Z0 the sheet's top). Each tool is loaded with `T<n> M6` and its length
//! taken from the tool table (`G43 H<n>`); the spindle runs `M3` at the operation's speed. Arcs are
//! `G2`/`G3` with I and J from their start (helices change Z on the way); drilling cycles are
//! `G98 G81` or `G83` with pecks, back to the height the tool came from. Words that do not change
//! (an axis, the feed) are left out, as the control keeps them.

use std::fmt::Write as _;

use crate::job::{Coolant, Job};
use crate::toolpath::{Move, Operation};
use crate::{CamError, Result};

/// The most lines a program may have (a guard against runaway output).
const MAX_LINES: usize = 20_000_000;

/// `x` with at most `decimals` decimals, trailing zeros and a negative zero dropped.
pub fn num(x: f64, decimals: u32) -> String {
    let d = decimals.clamp(0, 9) as usize;
    let s = format!("{x:.d$}");
    let s = if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s };
    if s == "-0" { "0".to_string() } else { s }
}

/// A comment: LinuxCNC comments are in parentheses, may not hold any, and are kept to plain
/// ASCII (Ø as D, ° as deg, anything else not ASCII as ?).
pub fn comment(text: &str) -> String {
    let mut t = String::new();
    for c in text.chars() {
        match c {
            '(' => t.push('['),
            ')' => t.push(']'),
            'Ø' | 'ø' | '⌀' => t.push('D'),
            '°' => t.push_str(" deg"),
            '×' => t.push('x'),
            c if c.is_control() => t.push(' '),
            c if c.is_ascii() => t.push(c),
            _ => t.push('?'),
        }
    }
    format!("({t})")
}

/// The tool table line for tool `n`, one `key=value` after another, the name last (it may hold
/// spaces): `TOOL T6 kind=flat D=6 flutes=1 flute_length=22 stepdown=3 stepover=0.9 rpm=18000
/// feed=1000 plunge=250 z_min=-10.2 name=...`. A drill or chamfer mill has `tip_angle` (a drill
/// `peck` if it pecks) instead of stepdown and stepover; `rpm` is the tool's own, `z_min` the
/// deepest it goes in `ops`.
pub fn tool_line(job: &Job, ops: &[Operation], n: u32, decimals: u32) -> String {
    let f = |x: f64| num(x, decimals);
    let z_min = ops.iter().filter(|o| o.tool == n).map(Operation::deepest).fold(f64::INFINITY, f64::min);
    let Some(t) = job.tool(n) else { return format!("TOOL T{n}") };
    let kind = match t.kind {
        crate::job::ToolKind::Flat => "flat",
        crate::job::ToolKind::Drill => "drill",
        crate::job::ToolKind::Chamfer => "chamfer",
    };
    let mut line = format!("TOOL T{n} kind={kind} D={} flutes={} flute_length={}", f(t.diameter), t.flutes, f(t.flute_length));
    match t.kind {
        crate::job::ToolKind::Flat => {
            line.push_str(&format!(" stepdown={} stepover={}", f(t.stepdown()), f(t.stepover())));
        }
        crate::job::ToolKind::Drill | crate::job::ToolKind::Chamfer => {
            line.push_str(&format!(" tip_angle={}", f(t.tip_angle())));
            if let Some(q) = t.peck {
                line.push_str(&format!(" peck={}", f(q)));
            }
        }
    }
    line.push_str(&format!(" rpm={} feed={} plunge={}", f(t.rpm), f(t.feed), f(t.plunge)));
    if z_min.is_finite() {
        line.push_str(&format!(" z_min={}", f(z_min)));
    }
    let name = if t.name.is_empty() { format!("{kind} D{}", f(t.diameter)) } else { t.name.clone() };
    line.push_str(&format!(" name={name}"));
    line
}

/// The program for `ops` (a planned job's operations, in work coordinates); `warnings` (the
/// plan's) go into its header as comments, so whoever runs it sees them.
pub fn linuxcnc(job: &Job, ops: &[Operation], title: &str, warnings: &[String]) -> Result<String> {
    let post = &job.post;
    let dec = post.decimals;
    let n = |x: f64| num(x, dec);
    let clearance = job.strategy.clearance;
    let mut g = String::new();
    let mut lines = 0usize;
    let mut out = |g: &mut String, line: String| -> Result<()> {
        lines += 1;
        if lines > MAX_LINES {
            return Err(CamError::Invalid(format!("the program would pass {MAX_LINES} lines")));
        }
        g.push_str(&line);
        g.push('\n');
        Ok(())
    };

    out(&mut g, "%".into())?;
    out(&mut g, comment(&format!("SolidCraft CAM: {title}")))?;
    let s = &job.stock;
    out(
        &mut g,
        comment(&format!(
            "stock {} x {} x {} mm {}; work zero: {} of the top face",
            n(s.size[0]),
            n(s.size[1]),
            n(s.thickness),
            s.material,
            match s.origin {
                crate::job::Origin::TopCenter => "centre",
                crate::job::Origin::TopFrontLeft => "front-left corner",
            }
        )),
    )?;
    let mut used: Vec<u32> = Vec::new();
    for op in ops {
        if !used.contains(&op.tool) {
            used.push(op.tool);
        }
    }
    for t in &used {
        out(&mut g, comment(&tool_line(job, ops, *t, dec)))?;
    }
    for w in warnings {
        // LinuxCNC reads at most 256 characters a line
        let text = comment(w);
        let words: Vec<&str> = text.trim_start_matches('(').trim_end_matches(')').split(' ').collect();
        let mut line = String::from("WARNING:");
        for word in words {
            if line.len() + word.len() + 1 > 200 {
                out(&mut g, format!("({line})"))?;
                line = String::from("WARNING ...");
            }
            line.push(' ');
            line.push_str(word);
        }
        out(&mut g, format!("({line})"))?;
    }
    out(&mut g, "G21 G90 G17 G40 G49 G80 G94".into())?;
    out(&mut g, "G54".into())?;
    out(&mut g, if post.blend_tolerance > 0.0 { format!("G64 P{}", n(post.blend_tolerance)) } else { "G61".into() })?;

    let coolant_on = match post.coolant {
        Coolant::Off => None,
        Coolant::Mist => Some("M7"),
        Coolant::Flood => Some("M8"),
    };
    let mut tool: Option<u32> = None;
    let mut rpm: Option<f64> = None;
    // the control's position and feed as far as the program has set them
    let mut at: [Option<f64>; 3] = [None; 3];
    let mut feed: Option<f64> = None;
    let mut motion: Option<&'static str> = None;
    let mut in_cycle = false;
    for op in ops {
        if tool != Some(op.tool) {
            if in_cycle {
                out(&mut g, "G80".into())?;
                in_cycle = false;
            }
            if tool.is_some() {
                out(&mut g, "M5 M9".into())?;
            }
            out(&mut g, comment(&format!("tool T{}", op.tool)))?;
            out(&mut g, format!("T{} M6", op.tool))?;
            out(&mut g, format!("G43 H{}", op.tool))?;
            tool = Some(op.tool);
            rpm = None;
            // after a change only the height is set: straight up (or down) to clearance first,
            // then across
            out(&mut g, format!("G0 Z{}", n(clearance)))?;
            at = [None, None, Some(clearance)];
            motion = Some("G0");
        }
        if rpm != Some(op.rpm) {
            out(&mut g, format!("S{} M3", n(op.rpm)))?;
            if let Some(c) = coolant_on {
                out(&mut g, c.into())?;
            }
            if post.spindle_wait > 0.0 {
                out(&mut g, format!("G4 P{}", n(post.spindle_wait)))?;
            }
            rpm = Some(op.rpm);
        }
        out(&mut g, comment(&op.name))?;
        for m in &op.moves {
            if !matches!(m, Move::Drill { .. }) && in_cycle {
                out(&mut g, "G80".into())?;
                in_cycle = false;
                motion = None;
            }
            match m {
                Move::Rapid { to } | Move::Line { to, .. } | Move::Arc { to, .. } => {
                    let (code, f, arc) = match m {
                        Move::Rapid { .. } => ("G0", None, None),
                        Move::Line { feed: f, .. } => ("G1", Some(*f), None),
                        Move::Arc { center, ccw, feed: f, .. } => (if *ccw { "G3" } else { "G2" }, Some(*f), Some(*center)),
                        Move::Drill { .. } => ("G0", None, None),
                    };
                    let mut line = String::new();
                    // an arc always says where it ends; a line only what changes
                    for (k, axis) in ["X", "Y", "Z"].into_iter().enumerate() {
                        let v = to[k];
                        let same = at[k].is_some_and(|a| n(a) == n(v));
                        if !same || (arc.is_some() && k < 2) {
                            let _ = write!(line, " {axis}{}", n(v));
                        }
                    }
                    if let Some(c) = arc {
                        let (Some(x0), Some(y0)) = (at[0], at[1]) else {
                            return Err(CamError::Invalid(format!("{}: an arc where the tool's position is not known", op.name)));
                        };
                        let _ = write!(line, " I{} J{}", n(c[0] - x0), n(c[1] - y0));
                    }
                    if line.is_empty() && arc.is_none() {
                        continue;
                    }
                    if let Some(f) = f
                        && feed.is_none_or(|old| n(old) != n(f))
                    {
                        let _ = write!(line, " F{}", n(f));
                        feed = Some(f);
                    }
                    let lead = if motion == Some(code) && arc.is_none() { String::new() } else { code.to_string() };
                    motion = Some(code);
                    out(&mut g, format!("{lead}{line}").trim_start().to_string())?;
                    at = [Some(to[0]), Some(to[1]), Some(to[2])];
                }
                Move::Drill { at: c, back, r, bottom, peck, feed: f } => {
                    // the cycle starts from `back` above the hole
                    if at[2].is_none_or(|z| n(z) != n(*back)) || at[0].is_none_or(|x| n(x) != n(c[0])) || at[1].is_none_or(|y| n(y) != n(c[1])) {
                        out(&mut g, format!("G0 X{} Y{} Z{}", n(c[0]), n(c[1]), n(*back)))?;
                    }
                    let cycle = match peck {
                        Some(q) => format!("G98 G83 X{} Y{} Z{} R{} Q{} F{}", n(c[0]), n(c[1]), n(*bottom), n(*r), n(*q), n(*f)),
                        None => format!("G98 G81 X{} Y{} Z{} R{} F{}", n(c[0]), n(c[1]), n(*bottom), n(*r), n(*f)),
                    };
                    out(&mut g, cycle)?;
                    in_cycle = true;
                    feed = Some(*f);
                    motion = None;
                    at = [Some(c[0]), Some(c[1]), Some(*back)];
                }
            }
        }
    }
    if in_cycle {
        out(&mut g, "G80".into())?;
    }
    out(&mut g, "M5 M9".into())?;
    out(&mut g, format!("G0 Z{}", n(clearance)))?;
    out(&mut g, "M30".into())?;
    out(&mut g, "%".into())?;
    Ok(g)
}
