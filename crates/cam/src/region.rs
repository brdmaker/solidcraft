//! Planar areas for toolpaths: unions, differences and offsets of polygons with holes.
//!
//! An [`Area`] is a set of shapes, each an outer ring (counter-clockwise) and its holes
//! (clockwise), in millimetres. Curved boundaries arrive as polylines within a chord tolerance;
//! round offset corners are drawn the same way. The arithmetic runs on a fixed integer grid of
//! [`GRID`] mm, so results are exact on that grid and never depend on the input's scale.

use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::scale::FixedScaleFloatOverlay;
use i_overlay::mesh::float::outline::offset::OutlineOffset;
use i_overlay::mesh::float::style::{LineJoin, OutlineStyle};
use serde::Serialize;
use solidcraft_geom::{Region2, Vec2};

use crate::{CamError, Result};

/// Grid of the integer arithmetic (mm): 0.1 µm, far below any machine's resolution.
pub const GRID: f64 = 1e-4;

/// Largest coordinate an area may have (mm): 10 m, well inside the grid's integer range.
pub const MAX_COORD: f64 = 10_000.0;

/// A closed polyline, without the closing repeat of its first point.
pub type Ring = Vec<[f64; 2]>;

/// Shapes, each `[outer, hole, hole, ...]`.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct Area {
    pub shapes: Vec<Vec<Ring>>,
}

impl Area {
    pub fn empty() -> Area {
        Area::default()
    }

    pub fn is_empty(&self) -> bool {
        self.shapes.is_empty()
    }

    /// An axis-aligned rectangle.
    pub fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Result<Area> {
        let (x0, x1) = (x0.min(x1), x0.max(x1));
        let (y0, y1) = (y0.min(y1), y0.max(y1));
        Area::from_rings(&[vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]])
    }

    /// A circle drawn within chord tolerance `tol`.
    pub fn circle(cx: f64, cy: f64, r: f64, tol: f64) -> Result<Area> {
        if !(r.is_finite() && r > 0.0) {
            return Err(CamError::Geometry(format!("circle radius {r} is not positive")));
        }
        Area::from_rings(&[circle_ring(cx, cy, r, tol)])
    }

    /// The area the rings enclose under the even-odd rule: rings inside rings are holes,
    /// whatever their direction.
    pub fn from_rings(rings: &[Ring]) -> Result<Area> {
        let rings: Vec<Ring> = rings.iter().filter(|r| r.len() >= 3).cloned().collect();
        if rings.is_empty() {
            return Ok(Area::empty());
        }
        check_rings(&rings)?;
        let shapes = rings
            .overlay_with_fixed_scale_as::<i64>(&Vec::<Ring>::new(), OverlayRule::Subject, FillRule::EvenOdd, 1.0 / GRID)
            .map_err(|e| CamError::Geometry(format!("area from rings: {e:?}")))?;
        Ok(Area { shapes })
    }

    /// The union of many polygons, each taken as filled whatever its direction (overlaps count
    /// once): the shadow of a set of triangles, say.
    pub fn union_of(polygons: &[Ring]) -> Result<Area> {
        let rings: Vec<Ring> = polygons
            .iter()
            .filter(|r| r.len() >= 3 && signed_area(r).abs() > GRID * GRID)
            .map(|r| if signed_area(r) < 0.0 { r.iter().rev().copied().collect() } else { r.clone() })
            .collect();
        if rings.is_empty() {
            return Ok(Area::empty());
        }
        check_rings(&rings)?;
        let shapes = rings
            .overlay_with_fixed_scale_as::<i64>(&Vec::<Ring>::new(), OverlayRule::Subject, FillRule::NonZero, 1.0 / GRID)
            .map_err(|e| CamError::Geometry(format!("union of polygons: {e:?}")))?;
        Ok(Area { shapes })
    }

    /// A sketch or face region, its curves drawn within chord tolerance `tol`.
    pub fn from_region(region: &Region2, tol: f64) -> Result<Area> {
        let mut rings = vec![ring_of(&region.outer.polyline(tol).into_vec())];
        rings.extend(region.holes.iter().map(|h| ring_of(&h.polyline(tol).into_vec())));
        Area::from_rings(&rings)
    }

    pub fn union(&self, other: &Area) -> Result<Area> {
        self.boolean(other, OverlayRule::Union)
    }

    pub fn difference(&self, other: &Area) -> Result<Area> {
        self.boolean(other, OverlayRule::Difference)
    }

    pub fn intersection(&self, other: &Area) -> Result<Area> {
        self.boolean(other, OverlayRule::Intersect)
    }

    fn boolean(&self, other: &Area, rule: OverlayRule) -> Result<Area> {
        if self.is_empty() && other.is_empty() {
            return Ok(Area::empty());
        }
        let shapes = self
            .shapes
            .overlay_with_fixed_scale_as::<i64>(&other.shapes, rule, FillRule::NonZero, 1.0 / GRID)
            .map_err(|e| CamError::Geometry(format!("area {rule:?}: {e:?}")))?;
        Ok(Area { shapes })
    }

    /// The area grown by `d` (shrunk when `d` is negative), corners rounded with chords within
    /// `tol`: every point within `d` of the area, or every point at least `-d` inside it.
    pub fn offset(&self, d: f64, tol: f64) -> Result<Area> {
        if !d.is_finite() || d.abs() > MAX_COORD {
            return Err(CamError::Geometry(format!("offset distance {d} is out of range")));
        }
        if self.is_empty() || d.abs() < GRID {
            return Ok(self.clone());
        }
        // `Round(a)`: a is the longest chord over the radius; a chord of length L on radius R
        // sags L²/(8R), so a sag of `tol` needs a = sqrt(8 tol / R).
        let tol = tol.clamp(GRID * 10.0, d.abs());
        let angle = (8.0 * tol / d.abs()).sqrt().clamp(0.01, 1.0);
        let style = OutlineStyle::new(d).line_join(LineJoin::Round(angle));
        let shapes = self.shapes.outline_fixed_scale_as::<i64>(&style, 1.0 / GRID).map_err(|e| CamError::Geometry(format!("offset {d}: {e:?}")))?;
        Ok(Area { shapes })
    }

    /// Enclosed area (mm²), holes subtracted.
    pub fn area(&self) -> f64 {
        self.shapes.iter().flatten().map(signed_area).sum()
    }

    /// Is the point inside (on a boundary counts as inside within the grid)?
    pub fn contains(&self, p: [f64; 2]) -> bool {
        self.shapes.iter().any(|shape| {
            let mut rings = shape.iter();
            rings.next().is_some_and(|outer| ring_contains(outer, p)) && !rings.any(|h| ring_contains(h, p))
        })
    }

    /// Every ring, outer ones counter-clockwise and holes clockwise.
    pub fn rings(&self) -> impl Iterator<Item = &Ring> {
        self.shapes.iter().flatten()
    }

    /// Bounding box `[min, max]`, `None` when empty.
    pub fn bounds(&self) -> Option<([f64; 2], [f64; 2])> {
        let mut pts = self.rings().flatten();
        let first = *pts.next()?;
        Some(pts.fold((first, first), |(lo, hi), p| ([lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])])))
    }

    /// The area moved by `(dx, dy)` and turned `quarter_turns` × 90° counter-clockwise about
    /// the origin first.
    pub fn placed(&self, quarter_turns: u8, dx: f64, dy: f64) -> Area {
        let turn = |p: [f64; 2]| match quarter_turns % 4 {
            1 => [-p[1], p[0]],
            2 => [-p[0], -p[1]],
            3 => [p[1], -p[0]],
            _ => p,
        };
        Area {
            shapes: self
                .shapes
                .iter()
                .map(|shape| shape.iter().map(|ring| ring.iter().map(|p| turn(*p)).map(|p| [p[0] + dx, p[1] + dy]).collect()).collect())
                .collect(),
        }
    }
}

/// Twice-checked input: finite and within [`MAX_COORD`].
fn check_rings(rings: &[Ring]) -> Result<()> {
    for p in rings.iter().flatten() {
        if !(p[0].is_finite() && p[1].is_finite()) || p[0].abs() > MAX_COORD || p[1].abs() > MAX_COORD {
            return Err(CamError::Geometry(format!("point {p:?} is not finite or beyond ±{MAX_COORD} mm")));
        }
    }
    Ok(())
}

fn ring_of(points: &[Vec2]) -> Ring {
    let mut ring: Ring = points.iter().map(|p| [p.x, p.y]).collect();
    if ring.len() > 1 && ring.first() == ring.last() {
        ring.pop();
    }
    ring
}

/// A circle as a ring of chords within `tol` of the true circle, counter-clockwise.
pub fn circle_ring(cx: f64, cy: f64, r: f64, tol: f64) -> Ring {
    let n = segments_for(r, tol, std::f64::consts::TAU);
    (0..n).map(|i| i as f64 * std::f64::consts::TAU / n as f64).map(|a| [cx + r * a.cos(), cy + r * a.sin()]).collect()
}

/// Chords for an arc of radius `r` sweeping `sweep` radians with sag at most `tol`.
pub fn segments_for(r: f64, tol: f64, sweep: f64) -> usize {
    let tol = tol.max(GRID);
    let step = if r <= tol { std::f64::consts::FRAC_PI_2 } else { 2.0 * (1.0 - tol / r).clamp(-1.0, 1.0).acos() };
    ((sweep.abs() / step.max(1e-3)).ceil() as usize).clamp(4, 100_000)
}

/// Each edge of a ring, closing edge included.
pub fn ring_edges(r: &Ring) -> impl Iterator<Item = ([f64; 2], [f64; 2])> + '_ {
    r.iter().zip(r.iter().skip(1).chain(r.first())).map(|(a, b)| (*a, *b))
}

pub fn signed_area(r: &Ring) -> f64 {
    0.5 * ring_edges(r).map(|(a, b)| a[0] * b[1] - b[0] * a[1]).sum::<f64>()
}

fn ring_contains(r: &Ring, p: [f64; 2]) -> bool {
    ring_edges(r).filter(|(a, b)| (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]).count() % 2 == 1
}

/// Length of a ring, closing segment included.
pub fn ring_length(r: &Ring) -> f64 {
    ring_edges(r).map(|(a, b)| (b[0] - a[0]).hypot(b[1] - a[1])).sum()
}
