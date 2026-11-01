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
        let fitted;
        let moves: &[Move] = if post.fit_arcs {
            fitted = fit(&op.moves, post.fit_tolerance);
            &fitted
        } else {
            &op.moves
        };
        for m in moves {
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

/// Most points one fitted move may replace.
const MAX_FIT: usize = 400;

/// `moves` with every run of straight feed moves at one height and feed (after the move that
/// brings the tool to its start) replaced by as few arcs and lines as stay within `tol` of all
/// its points and of the middle of every piece between them. Arcs end on planned points.
pub fn fit(moves: &[Move], tol: f64) -> Vec<Move> {
    let mut out: Vec<Move> = Vec::with_capacity(moves.len());
    let mut i = 0;
    let mut at: Option<[f64; 3]> = None;
    while let Some(m) = moves.get(i) {
        let (Move::Line { to, feed }, Some(start)) = (m, at) else {
            out.push(m.clone());
            at = Some(m.end());
            if let Move::Drill { at: c, back, .. } = m {
                at = Some([c[0], c[1], *back]);
            }
            i += 1;
            continue;
        };
        // the run: lines at the start's height and this feed
        let z = start[2];
        let mut pts = vec![[start[0], start[1]]];
        let mut k = i;
        while let Some(Move::Line { to: p, feed: f }) = moves.get(k) {
            if (p[2] - z).abs() > 1e-9 || (f - feed).abs() > 1e-9 {
                break;
            }
            pts.push([p[0], p[1]]);
            k += 1;
        }
        if pts.len() < 3 {
            // a lone move, or one that changes height: as planned
            out.push(m.clone());
            at = Some(*to);
            i += 1;
            continue;
        }
        let mut a = 0;
        while a + 1 < pts.len() {
            let line = longest_line(&pts, a, tol);
            let arc = longest_arc(&pts, a, tol);
            match arc {
                Some((b, centre, ccw)) if b > line => {
                    let e = pts.get(b).copied().unwrap_or([0.0, 0.0]);
                    out.push(Move::Arc { to: [e[0], e[1], z], center: centre, ccw, feed: *feed });
                    a = b;
                }
                _ => {
                    let e = pts.get(line).copied().unwrap_or([0.0, 0.0]);
                    out.push(Move::Line { to: [e[0], e[1], z], feed: *feed });
                    a = line;
                }
            }
        }
        at = pts.last().map(|p| [p[0], p[1], z]);
        i = k;
    }
    out
}

fn dist_to_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
    (a[0] + t * dx - p[0]).hypot(a[1] + t * dy - p[1])
}

/// The furthest point `b` > `a` that a straight line from `a` reaches with every point between,
/// and the middle of every piece, within `tol`.
fn longest_line(pts: &[[f64; 2]], a: usize, tol: f64) -> usize {
    let Some(&pa) = pts.get(a) else { return a + 1 };
    let mut best = a + 1;
    for b in a + 2..pts.len().min(a + MAX_FIT) {
        let Some(&pb) = pts.get(b) else { break };
        let ok = (a + 1..b).all(|k| {
            let (Some(&p), Some(&q)) = (pts.get(k), pts.get(k - 1)) else { return false };
            dist_to_segment(p, pa, pb) <= tol && dist_to_segment([(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0], pa, pb) <= tol
        }) && pts.get(b - 1).is_some_and(|q| dist_to_segment([(q[0] + pb[0]) / 2.0, (q[1] + pb[1]) / 2.0], pa, pb) <= tol);
        if !ok {
            break;
        }
        best = b;
    }
    best
}

/// The circle through three points: its centre, or `None` when they lie in a line.
fn circle(p: [f64; 2], q: [f64; 2], r: [f64; 2]) -> Option<[f64; 2]> {
    let d = 2.0 * (p[0] * (q[1] - r[1]) + q[0] * (r[1] - p[1]) + r[0] * (p[1] - q[1]));
    if d.abs() < 1e-12 {
        return None;
    }
    let (p2, q2, r2) = (p[0] * p[0] + p[1] * p[1], q[0] * q[0] + q[1] * q[1], r[0] * r[0] + r[1] * r[1]);
    Some([(p2 * (q[1] - r[1]) + q2 * (r[1] - p[1]) + r2 * (p[1] - q[1])) / d, (p2 * (r[0] - q[0]) + q2 * (p[0] - r[0]) + r2 * (q[0] - p[0])) / d])
}

/// The furthest point `b` (at least `a` + 3) an arc from `a` reaches, through the point halfway
/// between, with every point between and the middle of every piece within `tol` of it, all on
/// one turn the same way round, its radius from 0.05 to 5000 mm: `b`, the centre and whether
/// it turns anticlockwise.
fn longest_arc(pts: &[[f64; 2]], a: usize, tol: f64) -> Option<(usize, [f64; 2], bool)> {
    let pa = *pts.get(a)?;
    let mut best = None;
    for b in a + 3..pts.len().min(a + MAX_FIT) {
        let pb = *pts.get(b)?;
        let pm = *pts.get((a + b) / 2)?;
        // an arc ending back at its start is a full turn to the control
        if (pb[0] - pa[0]).hypot(pb[1] - pa[1]) < 0.01 {
            break;
        }
        let Some(c) = circle(pa, pm, pb) else { break };
        let r = (pa[0] - c[0]).hypot(pa[1] - c[1]);
        if !(0.05..=5000.0).contains(&r) {
            break;
        }
        let ccw = (pm[0] - pa[0]) * (pb[1] - pm[1]) - (pm[1] - pa[1]) * (pb[0] - pm[0]) > 0.0;
        let ang = |p: [f64; 2]| (p[1] - c[1]).atan2(p[0] - c[0]);
        let a0 = ang(pa);
        let along = |p: [f64; 2]| {
            let d = ang(p) - a0;
            if ccw { d.rem_euclid(std::f64::consts::TAU) } else { (-d).rem_euclid(std::f64::consts::TAU) }
        };
        let mut last = 0.0;
        let mut ok = true;
        for k in a + 1..=b {
            let (Some(&p), Some(&q)) = (pts.get(k), pts.get(k - 1)) else { return best };
            // furthest from the circle along the piece: at an end, or where it comes closest to the centre
            let off = ((p[0] - c[0]).hypot(p[1] - c[1]) - r).abs().max(r - dist_to_segment(c, q, p));
            let t = if k == b { along(p).max(last) } else { along(p) };
            if off > tol || t + 1e-9 < last {
                ok = false;
                break;
            }
            last = t;
        }
        // less than a full turn
        if !ok || last >= std::f64::consts::TAU - 1e-6 {
            break;
        }
        best = Some((b, c, ccw));
    }
    best
}
