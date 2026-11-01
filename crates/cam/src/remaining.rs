//! The stock left on the sheet as the operations cut it, carried from one to the next: feed
//! moves it finds cutting nothing run at the link feed, and rapids come down to just above the
//! stock that is still there rather than the top a level started from.
//!
//! The model errs on the side of material: a cell counts as cut only where the cutter covers
//! all of it, and a move is in air only when nothing stands within the cutter and a margin
//! round it (above its underside on a level, or a margin below it going down).

use crate::Result;
use crate::job::Job;
use crate::sim::{Cutter, HeightMap, cutter, pieces, sweep_in};
use crate::toolpath::{Move, Operation};

/// Longest piece a straight move is judged in, mm: a move partly in air keeps its cut at feed.
const PIECE: f64 = 1.0;
/// How far round the cutter nothing may stand for a move to be in air, mm.
const MARGIN: f64 = 0.5;
/// What may stand above a level move's underside and still count as air (a floor it rides on).
const EPS: f64 = 0.01;
/// How far above the stock a move going down stops counting as air, mm.
const DROP_MARGIN: f64 = 0.3;
/// Most cells of the model; past it the cells grow.
const MAX_CELLS: f64 = 20_000_000.0;

/// What the model changed: feed moves (or pieces) found in air, and the length of rapids
/// brought down from where the levels started, mm.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Trimmed {
    pub air_moves: usize,
    /// Length of feed moves found in air, mm.
    pub air_length: f64,
    /// Height rapids now cover instead of feed moves going down, mm.
    pub rapid_drop: f64,
}

/// The model's cell for a sheet: 0.2 mm, larger for a big sheet.
fn cell(job: &Job) -> f64 {
    let [w, d] = job.stock.size;
    (w * d / MAX_CELLS).sqrt().max(0.2)
}

/// The highest stock within `r` of the stretch `a`→`b` (in XY), measured against `under(t, d)`:
/// how far the highest cell stands above the cutter's underside there (`t` along the move, `d`
/// from its axis). Negative infinity where nothing is near.
fn stand(map: &HeightMap, a: [f64; 3], b: [f64; 3], r: f64, under: &dyn Fn(f64, f64) -> f64) -> f64 {
    let Some((i0, i1, j0, j1)) = map.span([a[0].min(b[0]) - r, a[1].min(b[1]) - r], [a[0].max(b[0]) + r, a[1].max(b[1]) + r]) else {
        return f64::NEG_INFINITY;
    };
    let (ux, uy) = (b[0] - a[0], b[1] - a[1]);
    let uu = ux * ux + uy * uy;
    let mut most = f64::NEG_INFINITY;
    for j in j0..=j1 {
        for i in i0..=i1 {
            let p = map.centre(i, j);
            let (px, py) = (p[0] - a[0], p[1] - a[1]);
            let t = if uu < 1e-18 { 0.0 } else { ((px * ux + py * uy) / uu).clamp(0.0, 1.0) };
            let d = (px - t * ux).hypot(py - t * uy);
            if d > r {
                continue;
            }
            most = most.max(map.at(i, j) - under(t, d));
        }
    }
    most
}

/// Whether the cutter `c` moving `a`→`b` cuts nothing (see the module's note); off the model,
/// the spoilboard's top is at `floor`.
fn in_air(map: &HeightMap, c: Cutter, a: [f64; 3], b: [f64; 3], floor: f64) -> bool {
    let low = a[2].min(b[2]);
    let r = c.r + MARGIN;
    let (lo, hi) = ([map.x0, map.y0], [map.x0 + map.nx as f64 * map.h, map.y0 + map.ny as f64 * map.h]);
    let inside = a[0].min(b[0]) - r >= lo[0] && a[1].min(b[1]) - r >= lo[1] && a[0].max(b[0]) + r <= hi[0] && a[1].max(b[1]) + r <= hi[1];
    if !inside && low < floor + DROP_MARGIN {
        return false;
    }
    // the underside, lowest along the move; within the margin, the cutter's edge
    let under = |_: f64, d: f64| low + c.rise((d - MARGIN).clamp(0.0, c.r));
    let s = stand(map, a, b, r, &under);
    if (a[2] - b[2]).abs() < 1e-9 { s < EPS } else { s < -DROP_MARGIN }
}

/// The highest stock under the cutter at `p` and a margin round it, at least the spoilboard's
/// top `floor`.
fn top_under(map: &HeightMap, c: Cutter, p: [f64; 3], floor: f64) -> f64 {
    stand(map, p, p, c.r + MARGIN, &|_, _| 0.0).max(floor)
}

/// Run `ops` over a model of the sheet in order; moves in air go at the link feed and rapids
/// come down to the retract height above the stock left (see the module's note).
pub fn trim_air(job: &Job, ops: &mut [Operation]) -> Result<Trimmed> {
    let [w, d] = job.stock.size;
    let zero = job.stock.work_zero();
    let h = cell(job);
    // the sheet, and round it the spoilboard's top as far as any cutter reaches past its edge
    let floor = -job.stock.thickness;
    let reach = job.tools.iter().map(|t| t.diameter).fold(0.0, f64::max) + MARGIN + 1.0;
    let (lo, hi) = ([-zero[0], -zero[1]], [w - zero[0], d - zero[1]]);
    let mut map = HeightMap::new([lo[0] - reach, lo[1] - reach], [hi[0] + reach, hi[1] + reach], 0.0, h)?;
    for j in 0..map.ny {
        for i in 0..map.nx {
            let p = map.centre(i, j);
            if (!(lo[0]..=hi[0]).contains(&p[0]) || !(lo[1]..=hi[1]).contains(&p[1]))
                && let Some(z) = map.z.get_mut(j * map.nx + i)
            {
                *z = floor as f32;
            }
        }
    }
    let pad = h * std::f64::consts::FRAC_1_SQRT_2 + 1e-6;
    let sag = h / 4.0;
    let retract = job.strategy.retract;
    let mut done = Trimmed::default();
    let mut at = [f64::NAN; 3];
    for op in ops.iter_mut() {
        let c = cutter(job, op.tool)?;
        let air_feed = job.strategy.feeds.link.min(job.strategy.rapid_feed);
        let mut out: Vec<Move> = Vec::with_capacity(op.moves.len());
        for m in &op.moves {
            let start = if at[0].is_nan() { [m.end()[0], m.end()[1], job.strategy.clearance] } else { at };
            match m {
                Move::Line { to, feed } => {
                    let mut from = start;
                    // straight down after a rapid: the rapid goes on to the retract height above
                    // the stock still there
                    let down = (to[0] - from[0]).abs() < 1e-9 && (to[1] - from[1]).abs() < 1e-9 && to[2] < from[2] - 1e-9;
                    if down && matches!(out.last(), Some(Move::Rapid { .. })) {
                        let target = (top_under(&map, c, from, floor) + retract).max(to[2]);
                        if target < from[2] - 1e-6 {
                            done.rapid_drop += from[2] - target;
                            from = [from[0], from[1], target];
                            out.push(Move::Rapid { to: from });
                        }
                    }
                    let len = (0..3).map(|k| (to[k] - from[k]).powi(2)).sum::<f64>().sqrt();
                    if len < 1e-9 {
                        at = *to;
                        continue;
                    }
                    let n = (len / PIECE).ceil().clamp(1.0, 100_000.0) as usize;
                    let lerp = |t: f64| [from[0] + (to[0] - from[0]) * t, from[1] + (to[1] - from[1]) * t, from[2] + (to[2] - from[2]) * t];
                    // pieces in air or not, joined while the same
                    let mut runs: Vec<(bool, [f64; 3])> = Vec::new();
                    for k in 1..=n {
                        let (a, b) = (lerp((k - 1) as f64 / n as f64), if k == n { *to } else { lerp(k as f64 / n as f64) });
                        let air = *feed < air_feed && in_air(&map, c, a, b, floor);
                        sweep_in(&mut map, c, a, b, pad);
                        match runs.last_mut() {
                            Some((was, end)) if *was == air => *end = b,
                            _ => runs.push((air, b)),
                        }
                    }
                    let mut prev = from;
                    for (air, end) in runs {
                        if air {
                            done.air_moves += 1;
                            done.air_length += (0..3).map(|k| (end[k] - prev[k]).powi(2)).sum::<f64>().sqrt();
                        }
                        out.push(Move::Line { to: end, feed: if air { air_feed } else { *feed } });
                        prev = end;
                    }
                }
                Move::Arc { to, center, ccw, feed } => {
                    let parts = pieces(start, m, sag);
                    let air = *feed < air_feed && parts.iter().all(|(a, b)| in_air(&map, c, *a, *b, floor));
                    for (a, b) in &parts {
                        sweep_in(&mut map, c, *a, *b, pad);
                    }
                    if air {
                        done.air_moves += 1;
                        done.air_length += parts.iter().map(|(a, b)| (0..3).map(|k| (b[k] - a[k]).powi(2)).sum::<f64>().sqrt()).sum::<f64>();
                    }
                    out.push(Move::Arc { to: *to, center: *center, ccw: *ccw, feed: if air { air_feed } else { *feed } });
                }
                Move::Rapid { .. } | Move::Drill { .. } => {
                    for (a, b) in pieces(start, m, sag) {
                        sweep_in(&mut map, c, a, b, pad);
                    }
                    out.push(m.clone());
                }
            }
            at = m.end();
        }
        op.moves = out;
    }
    Ok(done)
}
