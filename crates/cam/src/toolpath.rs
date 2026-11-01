//! Toolpaths: the moves of one tool, as the post-processor writes them.
//!
//! Coordinates are millimetres. Arcs are exact (`G2`/`G3`, with a Z change for a helix); paths
//! that follow offset areas are polylines within the area tolerance. The spindle turns clockwise
//! (`M3`), so every cutting path keeps the finished wall on its left: climb milling.

use schemars::JsonSchema;
use serde::Serialize;

use crate::region::{Ring, ring_length};

/// One move of the tool tip.
#[derive(Clone, Debug, PartialEq, Serialize, JsonSchema)]
#[serde(tag = "move", rename_all = "snake_case")]
pub enum Move {
    /// At rapid speed (`G0`); only above the material or along a cleared path at a safe height.
    Rapid { to: [f64; 3] },
    /// A straight cut (`G1`).
    Line { to: [f64; 3], feed: f64 },
    /// An arc in XY about `center`, ending at `to` (whose Z may differ: a helix). `ccw`: `G3`,
    /// else `G2`. Start and end at the same XY is a full turn.
    Arc { to: [f64; 3], center: [f64; 2], ccw: bool, feed: f64 },
    /// A drilling cycle at `at`, entered from height `back` (the tool is there, above `at`):
    /// rapid to the retract height `r`, drill to `bottom` (`G81`, or `G83` with `peck`), and
    /// back up to `back` (`G98`), so moves between holes never run at the low retract height.
    Drill { at: [f64; 2], back: f64, r: f64, bottom: f64, peck: Option<f64>, feed: f64 },
}

impl Move {
    pub fn end(&self) -> [f64; 3] {
        match self {
            Move::Rapid { to } | Move::Line { to, .. } | Move::Arc { to, .. } => *to,
            Move::Drill { at, back, .. } => [at[0], at[1], *back],
        }
    }
}

/// What an operation does.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum OpKind {
    Facing,
    /// Countersinks on the part's top face bored with an end mill, first of all while the sheet
    /// is whole.
    Countersink,
    Roughing,
    /// Through holes drilled: near the end, just before the separation (a hole through the
    /// sheet lets the vacuum leak).
    Drilling,
    /// Blind round steps (counterbores, recesses) milled.
    HoleMilling,
    FloorFinish,
    WallFinish,
    /// Countersinks and chamfers in hole stacks cut after the walls: with the chamfer mill, or
    /// bored with an end mill on a floor below the top face.
    Chamfer,
    Profile,
    /// A chamfer round the part's outline, cut after the profile so the cutter's tip runs in
    /// the profile's slot.
    EdgeChamfer,
    /// Through holes milled: near the end, with the drilled ones, just before the separation.
    ThroughHole,
    /// Cutting through the skin left under and round the parts (the `skin` cut-out), level by
    /// level over the whole sheet: last of all, while the vacuum still holds every part.
    Separation,
}

impl OpKind {
    /// The order operations run in on a sheet: material comes off before parts are cut free.
    pub fn rank(self) -> u8 {
        match self {
            OpKind::Facing => 0,
            OpKind::Countersink => 1,
            OpKind::Roughing => 2,
            OpKind::HoleMilling => 3,
            // the wall passes first, level by level: they clear what roughing could not reach
            // (corners tighter than the roughing tool), so the floor pass only skims the floor
            OpKind::WallFinish => 4,
            OpKind::FloorFinish => 5,
            OpKind::Chamfer => 6,
            OpKind::Profile => 7,
            OpKind::EdgeChamfer => 8,
            OpKind::Drilling => 9,
            OpKind::ThroughHole => 10,
            OpKind::Separation => 11,
        }
    }
}

/// The moves of one tool doing one job on one part.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct Operation {
    pub kind: OpKind,
    /// What it cuts, for the G-code's comments: `Flange #2: roughing`.
    pub name: String,
    /// LinuxCNC tool number.
    pub tool: u32,
    pub rpm: f64,
    /// Every move starts and ends at `clearance`.
    pub moves: Vec<Move>,
}

impl Operation {
    /// The lowest height it cuts at: its moves' ends (rapids aside) and drilling depths;
    /// infinity when it cuts nothing.
    pub fn deepest(&self) -> f64 {
        self.moves
            .iter()
            .filter_map(|m| match m {
                Move::Rapid { .. } => None,
                Move::Line { to, .. } | Move::Arc { to, .. } => Some(to[2]),
                Move::Drill { bottom, .. } => Some(*bottom),
            })
            .fold(f64::INFINITY, f64::min)
    }
}

/// Heights an operation moves at.
#[derive(Clone, Copy, Debug)]
pub struct Heights {
    /// Rapid moves between features happen here (above everything).
    pub clearance: f64,
    /// Feeds start this far above the surface they enter.
    pub retract: f64,
}

/// Builds the moves of one operation, tracking where the tool is.
pub struct PathBuilder {
    pub moves: Vec<Move>,
    pub at: [f64; 3],
    pub h: Heights,
}

fn dist2(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

impl PathBuilder {
    /// Starts at clearance height, somewhere: the operation before left the tool there. The
    /// first move goes across at clearance height to wherever the work starts.
    pub fn new(h: Heights, _hint: [f64; 2]) -> PathBuilder {
        PathBuilder { moves: Vec::new(), at: [f64::NAN, f64::NAN, h.clearance], h }
    }

    /// Has the tool been anywhere yet?
    fn placed(&self) -> bool {
        self.at[0].is_finite() && self.at[1].is_finite()
    }

    pub fn rapid(&mut self, to: [f64; 3]) {
        if to != self.at {
            self.moves.push(Move::Rapid { to });
            self.at = to;
        }
    }

    pub fn line(&mut self, to: [f64; 3], feed: f64) {
        if to != self.at {
            self.moves.push(Move::Line { to, feed });
            self.at = to;
        }
    }

    pub fn arc(&mut self, to: [f64; 3], center: [f64; 2], ccw: bool, feed: f64) {
        self.moves.push(Move::Arc { to, center, ccw, feed });
        self.at = to;
    }

    /// Up to clearance, across, and down at rapid to the retract height above `z_surface`.
    pub fn travel_to(&mut self, xy: [f64; 2], z_surface: f64) {
        let c = self.h.clearance;
        if !self.placed() {
            self.moves.push(Move::Rapid { to: [xy[0], xy[1], c] });
            self.at = [xy[0], xy[1], c];
        } else if [self.at[0], self.at[1]] != xy {
            self.rapid([self.at[0], self.at[1], c]);
            self.rapid([xy[0], xy[1], c]);
        }
        self.rapid([xy[0], xy[1], (z_surface + self.h.retract).min(c)]);
    }

    /// Back up to clearance height.
    pub fn up(&mut self) {
        let c = self.h.clearance;
        if self.placed() {
            self.rapid([self.at[0], self.at[1], c]);
        }
    }

    /// Follow `ring` (closed) once at height `z`, starting at its point `start`.
    pub fn ring(&mut self, ring: &Ring, start: usize, z: f64, feed: f64) {
        let n = ring.len();
        for i in 1..=n {
            if let Some(p) = ring.get((start + i) % n) {
                self.line([p[0], p[1], z], feed);
            }
        }
    }

    /// Ramp down along `ring` from the tool's height to `z`, at most `angle` (radians) from
    /// horizontal, going round as often as it takes; then once round at `z`. A ring too short
    /// to ramp on is entered by plunging at `plunge` feed.
    pub fn ramp_ring(&mut self, ring: &Ring, start: usize, z: f64, angle: f64, feed: f64, plunge: f64) {
        let len = ring_length(ring);
        let drop = self.at[2] - z;
        if drop > 1e-9 {
            if len < 1e-3 {
                self.line([self.at[0], self.at[1], z], plunge);
            } else {
                let run = drop / angle.tan().max(1e-3);
                let n = ring.len();
                let mut gone = 0.0;
                let mut i = start;
                let z0 = self.at[2];
                // At most 10 000 vertices of ramp: a ramp longer than that plunges the rest.
                let mut guard = 0usize;
                while gone < run && guard < 10_000 {
                    let (Some(a), Some(b)) = (ring.get(i % n), ring.get((i + 1) % n)) else { break };
                    let seg = dist2(*a, *b);
                    gone += seg;
                    let zz = (z0 - drop * (gone / run).min(1.0)).max(z);
                    self.line([b[0], b[1], zz], plunge.max(feed.min(feed * 0.5 + plunge)));
                    i += 1;
                    guard += 1;
                }
                self.line([self.at[0], self.at[1], z], plunge);
                let i = i % n;
                self.ring(ring, i, z, feed);
                return;
            }
        }
        self.ring(ring, start, z, feed);
    }

    /// A circle of radius `r` about `c` at height `z`, clockwise (wall on the left of a hole),
    /// starting and ending at its +X point.
    pub fn circle(&mut self, c: [f64; 2], r: f64, z: f64, feed: f64) {
        let p = [c[0] + r, c[1], z];
        self.line(p, feed);
        self.arc(p, c, false, feed);
    }

    /// A clockwise helix of radius `r` about `c` from the tool's height down to `z`, at most
    /// `pitch` per turn, then a full turn at `z` to flatten the bottom.
    pub fn helix(&mut self, c: [f64; 2], r: f64, z: f64, pitch: f64, feed: f64) {
        let p = [c[0] + r, c[1]];
        self.line([p[0], p[1], self.at[2]], feed);
        let drop = self.at[2] - z;
        let turns = (drop / pitch.max(1e-3)).ceil().clamp(1.0, 10_000.0) as usize;
        let z0 = self.at[2];
        for t in 1..=turns {
            let zz = z0 - drop * t as f64 / turns as f64;
            self.arc([p[0], p[1], zz], c, false, feed);
        }
        self.arc([p[0], p[1], z], c, false, feed);
    }
}

/// The vertex of `ring` nearest `p`.
pub fn nearest_vertex(ring: &Ring, p: [f64; 2]) -> usize {
    ring.iter().enumerate().min_by(|a, b| dist2(*a.1, p).total_cmp(&dist2(*b.1, p))).map(|(i, _)| i).unwrap_or(0)
}

/// Splits a ring into tab and cut stretches: `count` tabs of `width` (along the path) spaced
/// evenly, the first centred a half-spacing from the start. Returns, for each vertex of the
/// resampled ring (closed, first point repeated last), whether it lies on a tab.
pub fn with_tabs(ring: &Ring, count: usize, width: f64) -> Vec<([f64; 2], bool)> {
    let len = ring_length(ring);
    let mut out = Vec::new();
    let Some(first) = ring.first() else { return out };
    if count == 0 || width <= 0.0 || len <= count as f64 * width {
        let mut pts: Vec<([f64; 2], bool)> = ring.iter().map(|p| (*p, false)).collect();
        pts.push((*first, false));
        return pts;
    }
    let spacing = len / count as f64;
    let mut cuts: Vec<f64> = Vec::new(); // arc-lengths where tabs start and end
    for k in 0..count {
        let c = spacing * (k as f64 + 0.5);
        cuts.push(c - width / 2.0);
        cuts.push(c + width / 2.0);
    }
    let on_tab = |s: f64| (0..count).any(|k| (s - spacing * (k as f64 + 0.5)).abs() < width / 2.0);
    let n = ring.len();
    let mut s = 0.0;
    out.push((*first, on_tab(0.0)));
    for i in 0..n {
        let (Some(a), Some(b)) = (ring.get(i), ring.get((i + 1) % n)) else { continue };
        let seg = dist2(*a, *b);
        for c in cuts.iter().filter(|c| **c > s && **c < s + seg) {
            let t = (c - s) / seg;
            let p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
            // the cut point itself belongs to the stretch it starts
            out.push((p, on_tab(*c + 1e-9)));
        }
        s += seg;
        out.push((*b, on_tab(s - 1e-9)));
    }
    out
}

/// How the tool moved along a stroke.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StrokeKind {
    Rapid,
    Cut,
}

/// An operation as polylines: lines as they are, arcs every `arc_step` radians, drilling cycles
/// as their vertical plunge (from the retract height to the bottom).
#[derive(Clone, Debug)]
pub struct Stroke {
    pub kind: StrokeKind,
    pub pts: Vec<[f64; 3]>,
}

/// The strokes of `op`, starting from `start` (the tool's position before it). Consecutive moves
/// of one kind join into one stroke.
pub fn trace(op: &Operation, start: [f64; 3], arc_step: f64) -> Vec<Stroke> {
    let mut out: Vec<Stroke> = Vec::new();
    let mut cur = start;
    let step = arc_step.clamp(1e-3, 1.0);
    let mut push = |kind: StrokeKind, from: [f64; 3], pts: Vec<[f64; 3]>| match out.last_mut() {
        Some(s) if s.kind == kind && s.pts.last() == Some(&from) => s.pts.extend(pts),
        _ => {
            let mut all = vec![from];
            all.extend(pts);
            out.push(Stroke { kind, pts: all });
        }
    };
    for m in &op.moves {
        match m {
            Move::Rapid { to } => {
                if to[0].is_finite() && cur[0].is_finite() {
                    push(StrokeKind::Rapid, cur, vec![*to]);
                }
                cur = *to;
            }
            Move::Line { to, .. } => {
                push(StrokeKind::Cut, cur, vec![*to]);
                cur = *to;
            }
            Move::Arc { to, center, ccw, .. } => {
                let r = (cur[0] - center[0]).hypot(cur[1] - center[1]);
                let a0 = (cur[1] - center[1]).atan2(cur[0] - center[0]);
                let a1 = (to[1] - center[1]).atan2(to[0] - center[0]);
                let tau = std::f64::consts::TAU;
                let mut sweep = if *ccw { (a1 - a0).rem_euclid(tau) } else { -(a0 - a1).rem_euclid(tau) };
                if sweep.abs() < 1e-9 {
                    sweep = if *ccw { tau } else { -tau };
                }
                let n = ((sweep.abs() / step).ceil() as usize).clamp(1, 100_000);
                let pts = (1..=n)
                    .map(|i| {
                        let t = i as f64 / n as f64;
                        let a = a0 + sweep * t;
                        [center[0] + r * a.cos(), center[1] + r * a.sin(), cur[2] + (to[2] - cur[2]) * t]
                    })
                    .collect();
                push(StrokeKind::Cut, cur, pts);
                cur = *to;
            }
            Move::Drill { at, back, r, bottom, .. } => {
                push(StrokeKind::Rapid, cur, vec![[at[0], at[1], *r]]);
                push(StrokeKind::Cut, [at[0], at[1], *r], vec![[at[0], at[1], *bottom]]);
                push(StrokeKind::Rapid, [at[0], at[1], *bottom], vec![[at[0], at[1], *back]]);
                cur = [at[0], at[1], *back];
            }
        }
    }
    out
}
