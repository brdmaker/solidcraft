//! Laying parts out on the sheet by their bounding boxes.
//!
//! Free-rectangle packing: the sheet's usable area starts as one free rectangle; each part, the
//! largest first, goes where its box (turned 0° or 90°) leaves the least spare on its shorter
//! side; the free rectangles it overlaps are split round it. Parts keep `spacing` between their
//! boxes and stay `margin` inside the sheet's edge. The result is deterministic.

use schemars::JsonSchema;
use serde::Serialize;

use crate::job::{NestOptions, Stock};
use crate::region::Area;

/// One part to nest: its label, how many, its outline seen from above (in its own coordinates)
/// and its height.
pub struct NestItem<'a> {
    pub label: &'a str,
    pub quantity: u32,
    pub outline: &'a Area,
    pub height: f64,
}

/// Where one copy of a part goes: turned `quarter_turns` × 90° counter-clockwise about its own
/// origin, then moved by `offset`, in sheet coordinates (X 0…width, Y 0…depth).
#[derive(Clone, Debug, PartialEq, Serialize, JsonSchema)]
pub struct Placement {
    /// Index of the part in the job's `parts`.
    pub part: usize,
    pub label: String,
    /// Which copy of that part (0-based).
    pub copy: u32,
    pub quarter_turns: u8,
    pub offset: [f64; 2],
    /// The copy's bounding box on the sheet.
    pub bounds: [[f64; 2]; 2],
}

/// A copy that did not go on the sheet, and why.
#[derive(Clone, Debug, PartialEq, Serialize, JsonSchema)]
pub struct Unplaced {
    pub part: usize,
    pub label: String,
    pub copy: u32,
    pub reason: String,
}

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct Nesting {
    pub placements: Vec<Placement>,
    pub unplaced: Vec<Unplaced>,
    /// Parts' outline area over the sheet's usable area.
    pub utilisation: f64,
}

#[derive(Clone, Copy, Debug)]
struct Rect {
    x: f64,
    y: f64,
    w: f64,
    h: f64,
}

impl Rect {
    fn contains(&self, o: &Rect) -> bool {
        o.x >= self.x - EPS && o.y >= self.y - EPS && o.x + o.w <= self.x + self.w + EPS && o.y + o.h <= self.y + self.h + EPS
    }

    fn overlaps(&self, o: &Rect) -> bool {
        o.x < self.x + self.w - EPS && self.x < o.x + o.w - EPS && o.y < self.y + self.h - EPS && self.y < o.y + o.h - EPS
    }
}

const EPS: f64 = 1e-9;

/// Most free rectangles kept; past it the smallest are dropped (fewer places tried, never overlaps).
const MAX_FREE: usize = 4096;

/// Lay `items` out on `stock`.
pub fn nest(stock: &Stock, opts: &NestOptions, items: &[NestItem]) -> Nesting {
    let (w, h, m, s) = (stock.size[0], stock.size[1], opts.margin, opts.spacing);
    // Each box carries the spacing on its right and back; the usable area grows by the same so
    // a part may still touch the margin there.
    let mut free = vec![Rect { x: m, y: m, w: w - 2.0 * m + s, h: h - 2.0 * m + s }];
    let usable = (w - 2.0 * m) * (h - 2.0 * m);

    // Every copy, the largest box first (then by part and copy, so ties stay put).
    let mut copies: Vec<(usize, u32, [f64; 2], [f64; 2])> = Vec::new();
    let mut unplaced = Vec::new();
    for (i, it) in items.iter().enumerate() {
        let Some((lo, hi)) = it.outline.bounds() else {
            for c in 0..it.quantity {
                unplaced.push(Unplaced { part: i, label: it.label.into(), copy: c, reason: "the part has no outline seen from above".into() });
            }
            continue;
        };
        if it.height > stock.thickness + 1e-6 {
            for c in 0..it.quantity {
                unplaced.push(Unplaced {
                    part: i,
                    label: it.label.into(),
                    copy: c,
                    reason: format!("the part is {:.3} mm tall; the sheet is {:.3} mm thick", it.height, stock.thickness),
                });
            }
            continue;
        }
        for c in 0..it.quantity {
            copies.push((i, c, lo, hi));
        }
    }
    copies.sort_by(|a, b| {
        let area = |c: &(usize, u32, [f64; 2], [f64; 2])| (c.3[0] - c.2[0]) * (c.3[1] - c.2[1]);
        area(b).total_cmp(&area(a)).then(a.0.cmp(&b.0)).then(a.1.cmp(&b.1))
    });

    let mut placements = Vec::new();
    let mut placed_area = 0.0;
    for (i, c, lo, hi) in copies {
        let Some(it) = items.get(i) else { continue };
        let (bw, bh) = (hi[0] - lo[0], hi[1] - lo[1]);
        let turns: &[u8] = if opts.rotate { &[0, 1] } else { &[0] };
        // Best short side fit, then long side, then lowest, then leftmost.
        let mut best: Option<((f64, f64, f64, f64), Rect, u8)> = None;
        for &q in turns {
            let (iw, ih) = if q % 2 == 0 { (bw + s, bh + s) } else { (bh + s, bw + s) };
            for f in &free {
                if iw <= f.w + EPS && ih <= f.h + EPS {
                    let (dw, dh) = (f.w - iw, f.h - ih);
                    let key = (dw.min(dh), dw.max(dh), f.y, f.x);
                    if best.as_ref().is_none_or(|(k, ..)| key < *k) {
                        best = Some((key, Rect { x: f.x, y: f.y, w: iw, h: ih }, q));
                    }
                }
            }
        }
        let Some((_, r, q)) = best else {
            unplaced.push(Unplaced { part: i, label: it.label.into(), copy: c, reason: "no room left on the sheet".into() });
            continue;
        };
        // The turned outline's box starts at the free rectangle's corner.
        let turned = it.outline.placed(q, 0.0, 0.0);
        let (tlo, thi) = turned.bounds().unwrap_or((lo, hi));
        let offset = [r.x - tlo[0], r.y - tlo[1]];
        placements.push(Placement {
            part: i,
            label: it.label.into(),
            copy: c,
            quarter_turns: q,
            offset,
            bounds: [[r.x, r.y], [r.x + (thi[0] - tlo[0]), r.y + (thi[1] - tlo[1])]],
        });
        placed_area += it.outline.area();
        split(&mut free, &r);
    }
    placements.sort_by(|a, b| a.part.cmp(&b.part).then(a.copy.cmp(&b.copy)));
    unplaced.sort_by(|a, b| a.part.cmp(&b.part).then(a.copy.cmp(&b.copy)));
    Nesting { placements, unplaced, utilisation: if usable > 0.0 { placed_area / usable } else { 0.0 } }
}

/// Take `used` out of the free rectangles: each one it overlaps becomes the (up to four) parts
/// of it beside `used`; then drop the rectangles inside others.
fn split(free: &mut Vec<Rect>, used: &Rect) {
    let mut next = Vec::with_capacity(free.len() + 4);
    for f in free.drain(..) {
        if !f.overlaps(used) {
            next.push(f);
            continue;
        }
        if used.x > f.x + EPS {
            next.push(Rect { x: f.x, y: f.y, w: used.x - f.x, h: f.h });
        }
        if used.x + used.w < f.x + f.w - EPS {
            next.push(Rect { x: used.x + used.w, y: f.y, w: f.x + f.w - used.x - used.w, h: f.h });
        }
        if used.y > f.y + EPS {
            next.push(Rect { x: f.x, y: f.y, w: f.w, h: used.y - f.y });
        }
        if used.y + used.h < f.y + f.h - EPS {
            next.push(Rect { x: f.x, y: used.y + used.h, w: f.w, h: f.y + f.h - used.y - used.h });
        }
    }
    let mut kept: Vec<Rect> = Vec::with_capacity(next.len());
    for (i, r) in next.iter().enumerate() {
        let inside = next.iter().enumerate().any(|(j, o)| j != i && o.contains(r) && (!r.contains(o) || j < i));
        if !inside {
            kept.push(*r);
        }
    }
    if kept.len() > MAX_FREE {
        kept.sort_by(|a, b| (b.w * b.h).total_cmp(&(a.w * a.h)));
        kept.truncate(MAX_FREE);
    }
    *free = kept;
}
