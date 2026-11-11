//! The turned part's profile: its outer and inner radius along its axis, sampled finely from the
//! solid.
//!
//! From the exact B-rep where every face is one of revolution about Z (a plane square to Z, a
//! cylinder, cone, sphere or torus on the axis): each face spans the heights of its vertices and
//! has its radius in closed form there. At each sample the farthest face from the axis is the
//! outer radius; the faces a ray out from the axis crosses just off the sample say whether the
//! axis is in the material (an odd count) or a bore is, as wide as the nearest of them.
//!
//! A part with any other face (a flat, a cross hole, a free-form surface) is read from its mesh
//! instead, within [`MESH_TOL`]: cut by a plane square to Z at every sample, the farthest point
//! of the cut from the axis is the outer radius there, the nearest the inner.

use solidcore_record::brep::Loop;
use solidcraft_geom::brep::eval::CurveEval;
use solidcraft_kernel::{Body, FaceSurface, face_surface};

use super::job::Front;
use crate::{CamError, Result};

/// The mesh's chord tolerance, mm: the profile lies within it of the true surfaces.
pub const MESH_TOL: f64 = 0.001;
/// The finest step between samples along the axis, mm.
pub const STEP: f64 = 0.005;
/// Direction of the ray testing whether the axis is in the material, radians from +X.
const RAY: f64 = 0.618_033_988_7;
/// The ray's plane lies this far in front of each sample's, mm.
const RAY_SHIFT: f64 = 1.618_034e-7;
/// Corners this close to a sample's plane are on it, mm.
const EPS: f64 = 1e-9;
/// Most samples along the axis; a longer part is sampled more coarsely.
const MAX_SAMPLES: usize = 200_000;

/// The part's radii along its axis, z from 0 (the end toward the chuck) to `length` (the front).
#[derive(Clone, Debug)]
pub struct Profile {
    pub length: f64,
    /// Step between samples, mm: sample `i` is at z = i · dz.
    pub dz: f64,
    /// Outer radius at each sample.
    pub od: Vec<f64>,
    /// Inner radius at each sample: a bore's (0: solid to the axis).
    pub id: Vec<f64>,
    /// Read from the exact faces (else from the mesh).
    pub exact: bool,
}

impl Profile {
    /// The sample nearest `z` (clamped to the part).
    pub fn index(&self, z: f64) -> usize {
        let i = (z / self.dz).round();
        if i.is_finite() && i > 0.0 { (i as usize).min(self.od.len().saturating_sub(1)) } else { 0 }
    }

    /// Outer radius at `z`.
    pub fn od_at(&self, z: f64) -> f64 {
        self.od.get(self.index(z)).copied().unwrap_or(0.0)
    }

    /// Largest outer radius.
    pub fn max_od(&self) -> f64 {
        self.od.iter().copied().fold(0.0, f64::max)
    }
}

/// The profile of `body`, turned about its Z axis with its `front` end away from the chuck: from
/// its exact faces where they are all of revolution about Z, else from its mesh.
pub fn read(body: &Body, front: Front) -> Result<Profile> {
    match exact(body, front)? {
        Some(p) => Ok(p),
        None => from_mesh(body, front),
    }
}

/// The profile a turn-mill part is turned to before its milled features are cut (flats, slots,
/// cross holes, holes off the axis): from its exact faces where they are all of revolution about
/// Z; else its outer radius from the mesh (the farthest point at each sample: what the milling
/// leaves standing), and a bore only where a face of revolution about Z bounds it (the nearest
/// point of the cut on such a face): a slot or cross hole through the axis is milled, not bored.
pub fn read_envelope(body: &Body, front: Front) -> Result<Profile> {
    if let Some(p) = exact(body, front)? {
        return Ok(p);
    }
    let mut p = from_mesh(body, front)?;
    let faces = turned_faces(body, front, true)?.map(|f| f.0).unwrap_or_default();
    let tol = 2.0 * MESH_TOL + 1e-6;
    let n = p.id.len();
    // on a face of revolution: a turned bore
    let turned: Vec<bool> = (0..n)
        .map(|i| {
            let (z, d) = (i as f64 * p.dz, p.id.get(i).copied().unwrap_or(0.0));
            d > 0.0
                && faces.iter().any(|t| match *t {
                    Turned::Round { z0, z1, shape } => z >= z0 - 1e-9 && z <= z1 + 1e-9 && (shape.radius(z) - d).abs() <= tol,
                    Turned::Face { .. } => false,
                })
        })
        .collect();
    // a stretch where the axis is open but no turned face bounds it: milled. A bore goes on
    // through it where it is at least as wide as the bore at its ends (a pocket opening the bore
    // to the face); else it is solid as turned.
    let mut i = 0;
    while i < n {
        if turned.get(i).copied().unwrap_or(false) || p.id.get(i).copied().unwrap_or(0.0) <= 0.0 {
            i += 1;
            continue;
        }
        let a = i;
        while i < n && !turned.get(i).copied().unwrap_or(false) && p.id.get(i).copied().unwrap_or(0.0) > 0.0 {
            i += 1;
        }
        let ends = [a.checked_sub(1), Some(i).filter(|j| *j < n)];
        let bore =
            ends.iter().flatten().filter(|j| turned.get(**j).copied().unwrap_or(false)).filter_map(|j| p.id.get(*j)).copied().fold(0.0, f64::max);
        for k in a..i {
            if let Some(d) = p.id.get_mut(k) {
                *d = if bore > 0.0 && *d >= bore - tol { bore } else { 0.0 };
            }
        }
    }
    Ok(p)
}

/// How far off the axis a face's axis or centre may lie, and how far from square to Z a plane may
/// turn (as a sine), and still count as turned about Z.
const AXIS_TOL: f64 = 1e-6;

/// Points taken along each edge of a face (besides its ends), to find the face's extent.
const EDGE_SAMPLES: usize = 16;

/// One face of revolution about Z, in the part's own z (from its chuck end).
#[derive(Clone, Copy, Debug)]
enum Turned {
    /// A plane square to the axis at `z`, out to `r`.
    Face { z: f64, r: f64 },
    /// A face of revolution from `z0` to `z1` with its radius at each z.
    Round { z0: f64, z1: f64, shape: Shape },
}

#[derive(Clone, Copy, Debug)]
enum Shape {
    Cylinder {
        r: f64,
    },
    /// The radius grows by `slope` per unit of z from `r0` at z 0.
    Cone {
        r0: f64,
        slope: f64,
    },
    /// About the axis point at `zc`: radius `r` (a sphere); a torus's tube of radius `r` round a
    /// ring of radius `ring`, on its `outer` side or its inner.
    Ring {
        zc: f64,
        ring: f64,
        r: f64,
        outer: bool,
    },
}

impl Shape {
    fn radius(self, z: f64) -> f64 {
        match self {
            Shape::Cylinder { r } => r,
            Shape::Cone { r0, slope } => (r0 + slope * z).max(0.0),
            Shape::Ring { zc, ring, r, outer } => {
                let h = (r * r - (z - zc) * (z - zc)).max(0.0).sqrt();
                (if outer { ring + h } else { ring - h }).max(0.0)
            }
        }
    }
}

/// The part's faces of revolution about Z in its own z, and its length; `None` where a face is
/// not one (with `skip_others`: such faces are left out instead).
fn turned_faces(body: &Body, front: Front, skip_others: bool) -> Result<Option<(Vec<Turned>, f64)>> {
    if body.is_mesh() {
        return Ok(None);
    }
    let rec = body.record();
    // each face's vertices (with its poles) and points along its edges: a meridian arc shows
    // which side of a torus's tube the face is on, and how far along the axis it reaches, where its
    // ends lie on the tube's ring
    let corners = |f: usize| -> Vec<[f64; 3]> {
        let mut out = Vec::new();
        for lp in rec.faces.get(f).map(|x| x.loops.as_slice()).unwrap_or_default() {
            let mut vs: Vec<u32> = Vec::new();
            match lp {
                Loop::Edges { edges } => {
                    for oe in edges {
                        let Some(e) = rec.edges.get(oe.edge as usize) else { continue };
                        vs.extend(e.vertices);
                        if let Ok(c) = CurveEval::new(e.parameters()) {
                            let [t0, t1] = e.interval;
                            out.extend(
                                (1..EDGE_SAMPLES).filter_map(|i| c.point(t0 + (t1 - t0) * i as f64 / EDGE_SAMPLES as f64)).map(|p| [p.x, p.y, p.z]),
                            );
                        }
                    }
                }
                Loop::Vertex { vertex } => vs.push(*vertex),
            }
            out.extend(vs.iter().filter_map(|v| rec.vertices.get(*v as usize)).map(|v| v.point));
        }
        out
    };
    let all: Vec<[f64; 3]> = (0..rec.faces.len()).flat_map(corners).collect();
    let lo = all.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
    let hi = all.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
    if !(lo.is_finite() && hi.is_finite() && hi > lo) {
        return Ok(None);
    }
    let (along, sign) = match front {
        Front::PlusZ => (lo, 1.0),
        Front::MinusZ => (hi, -1.0),
    };
    let zp = |z: f64| sign * (z - along);
    let on_axis = |x: f64, y: f64| x.hypot(y) < AXIS_TOL.max(1e-9 * (hi - lo));
    let mut faces: Vec<Turned> = Vec::with_capacity(rec.faces.len());
    for f in 0..rec.faces.len() {
        let pts = corners(f);
        let zs: Vec<f64> = pts.iter().map(|p| zp(p[2])).collect();
        let z0 = zs.iter().copied().fold(f64::INFINITY, f64::min);
        let z1 = zs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let r_of = |p: &[f64; 3]| p[0].hypot(p[1]);
        let Some(surface) = face_surface(body, f) else {
            if skip_others {
                continue;
            }
            return Ok(None);
        };
        let round = |shape: Shape| (z0.is_finite() && z1 >= z0).then_some(Turned::Round { z0, z1, shape });
        let t = match surface {
            FaceSurface::Plane { normal, point } => {
                let n = normal.vec();
                if n.z.abs() < 1.0 - AXIS_TOL {
                    if skip_others {
                        continue;
                    }
                    return Ok(None);
                }
                let r = pts.iter().map(r_of).fold(0.0, f64::max);
                Some(Turned::Face { z: zp(point.z), r })
            }
            FaceSurface::Cylinder { origin, axis, radius } => {
                if axis.vec().z.abs() < 1.0 - AXIS_TOL || !on_axis(origin.x, origin.y) {
                    if skip_others {
                        continue;
                    }
                    return Ok(None);
                }
                round(Shape::Cylinder { r: radius })
            }
            FaceSurface::Cone { apex, axis, half_angle } => {
                let a = axis.vec();
                if a.z.abs() < 1.0 - AXIS_TOL || !on_axis(apex.x, apex.y) {
                    if skip_others {
                        continue;
                    }
                    return Ok(None);
                }
                // r = (z − apex) along the axis · tan(half angle), in the part's z
                let slope = half_angle.tan() * a.z.signum() * sign;
                round(Shape::Cone { r0: -slope * zp(apex.z), slope })
            }
            FaceSurface::Sphere { center, radius } => {
                if !on_axis(center.x, center.y) {
                    if skip_others {
                        continue;
                    }
                    return Ok(None);
                }
                round(Shape::Ring { zc: zp(center.z), ring: 0.0, r: radius, outer: true })
            }
            FaceSurface::Torus { center, axis, major, minor } => {
                if axis.vec().z.abs() < 1.0 - AXIS_TOL || !on_axis(center.x, center.y) {
                    if skip_others {
                        continue;
                    }
                    return Ok(None);
                }
                // its side of the tube: where its corners lie off the ring
                let off = pts.iter().map(|p| r_of(p) - major).fold(0.0, |a: f64, d| if d.abs() > a.abs() { d } else { a });
                round(Shape::Ring { zc: zp(center.z), ring: major, r: minor, outer: off >= 0.0 })
            }
        };
        match t {
            Some(t) => faces.push(t),
            None => return Ok(None),
        }
    }
    Ok(Some((faces, hi - lo)))
}

/// The profile from the exact faces, or `None` where a face is not of revolution about Z.
fn exact(body: &Body, front: Front) -> Result<Option<Profile>> {
    let Some((faces, length)) = turned_faces(body, front, false)? else { return Ok(None) };
    let n = ((length / STEP).ceil() as usize + 1).clamp(2, MAX_SAMPLES);
    let dz = length / (n - 1) as f64;
    let tol = EPS.max(1e-9 * length);
    let mut od = vec![0.0f64; n];
    let mut id = vec![0.0f64; n];
    let shift = RAY_SHIFT.max(1e-7 * length);
    for i in 0..n {
        let z = i as f64 * dz;
        let mut o = 0.0f64;
        for t in &faces {
            match *t {
                Turned::Face { z: zf, r } if (zf - z).abs() <= tol => o = o.max(r),
                Turned::Round { z0, z1, shape } if z >= z0 - tol && z <= z1 + tol => o = o.max(shape.radius(z.clamp(z0, z1))),
                _ => {}
            }
        }
        // the bore just either side of the sample (inside the part): where a face square to the
        // axis stands at it (a bore's floor, a counterbore's step), the narrower
        let bore = |zq: f64| {
            let mut radii: Vec<f64> = faces
                .iter()
                .filter_map(|t| match *t {
                    Turned::Round { z0, z1, shape } if zq > z0 && zq < z1 => Some(shape.radius(zq)),
                    _ => None,
                })
                .filter(|r| *r > 1e-9)
                .collect();
            radii.sort_by(f64::total_cmp);
            radii.dedup_by(|a, b| (*a - *b).abs() < 1e-7);
            // an even count: the axis is outside the material, the nearest face is a bore's
            match radii.first() {
                Some(r) if radii.len().is_multiple_of(2) && *r < o - 0.01 => *r,
                _ => 0.0,
            }
        };
        let d = [z - shift, z + shift].into_iter().filter(|zq| *zq > 0.0 && *zq < length).map(bore).fold(f64::INFINITY, f64::min);
        if let (Some(slot), Some(di)) = (od.get_mut(i), id.get_mut(i)) {
            *slot = o;
            *di = if d.is_finite() { d } else { 0.0 };
        }
    }
    if od.iter().any(|v| !v.is_finite()) || od.iter().all(|v| *v <= 0.0) {
        return Err(CamError::Geometry("the part's profile could not be read".into()));
    }
    Ok(Some(Profile { length, dz, od, id, exact: true }))
}

/// The profile from the mesh: each sample's plane cut through it.
fn from_mesh(body: &Body, front: Front) -> Result<Profile> {
    let mesh = body.mesh(&solidcraft_kernel::MeshTolerance::chord(MESH_TOL))?;
    let b = mesh.bounds();
    let (lo, hi) = (b.min, b.max);
    if !(lo.x.is_finite() && hi.x.is_finite() && hi.z > lo.z) {
        return Err(CamError::Geometry("the part has no size".into()));
    }
    // turned about Z: as wide in X as in Y, centred on the axis
    let tol = 0.01;
    if (lo.x + hi.x).abs() > tol || (lo.y + hi.y).abs() > tol || ((hi.x - lo.x) - (hi.y - lo.y)).abs() > tol {
        return Err(CamError::Geometry(format!(
            "the part is not turned about its Z axis: it spans x {:.3}…{:.3}, y {:.3}…{:.3}",
            lo.x, hi.x, lo.y, hi.y
        )));
    }
    let length = hi.z - lo.z;
    let n = ((length / STEP).ceil() as usize + 1).clamp(2, MAX_SAMPLES);
    let dz = length / (n - 1) as f64;
    let mut od = vec![f64::NEG_INFINITY; n];
    let mut id = vec![f64::INFINITY; n];
    // how often a ray along +X from the axis crosses the cut at each sample: odd, the axis is in
    // the material there
    let mut crossings = vec![0u32; n];
    // a point's z along the part, from its chuck end
    let along = |z: f64| match front {
        Front::PlusZ => z - lo.z,
        Front::MinusZ => hi.z - z,
    };
    for t in &mesh.triangles {
        let (Some(a), Some(bb), Some(c)) = (mesh.positions.get(t[0] as usize), mesh.positions.get(t[1] as usize), mesh.positions.get(t[2] as usize))
        else {
            continue;
        };
        let p = [(along(a.z), a.x.hypot(a.y)), (along(bb.z), bb.x.hypot(bb.y)), (along(c.z), c.x.hypot(c.y))];
        // with the (z, x, y) of each corner for the cut
        let q = [(along(a.z), a.x, a.y), (along(bb.z), bb.x, bb.y), (along(c.z), c.x, c.y)];
        let zlo = p.iter().map(|v| v.0).fold(f64::INFINITY, f64::min);
        let zhi = p.iter().map(|v| v.0).fold(f64::NEG_INFINITY, f64::max);
        // a corner a rounding error off a sample (a face at z 0 meshed at 1e-17) is on it
        let (zlo, zhi) = (zlo - EPS, zhi + EPS);
        let i0 = ((zlo / dz).ceil().max(0.0) as usize).min(n - 1);
        let i1 = ((zhi / dz).floor().max(0.0) as usize).min(n - 1);
        if (zlo / dz).ceil() > (zhi / dz).floor() {
            continue;
        }
        for i in i0..=i1 {
            let z = i as f64 * dz;
            let mut hit = |r: f64, inner: bool| {
                if let (Some(o), Some(d)) = (od.get_mut(i), id.get_mut(i)) {
                    *o = o.max(r);
                    if inner {
                        *d = d.min(r);
                    }
                }
            };
            if zhi - zlo < 3.0 * EPS {
                // a triangle in the plane (a face square to the axis): it reaches as far out as its
                // corners; it says nothing of a bore
                for v in &p {
                    hit(v.1, false);
                }
                continue;
            }
            for (k, l) in [(0, 1), (1, 2), (2, 0)] {
                let (Some(u), Some(v)) = (q.get(k), q.get(l)) else { continue };
                let (za, zb) = (u.0, v.0);
                if ((za - z) * (zb - z) > 0.0 && (za - z).abs() > EPS && (zb - z).abs() > EPS) || (za - zb).abs() < 1e-15 {
                    if (za - z).abs() <= EPS {
                        hit(u.1.hypot(u.2), true);
                    }
                    continue;
                }
                let s = ((z - za) / (zb - za)).clamp(0.0, 1.0);
                hit((u.1 + (v.1 - u.1) * s).hypot(u.2 + (v.2 - u.2) * s), true);
            }
            // the ray in a plane a hair off the sample's, so no ring of mesh vertices lies in it,
            // at an angle no mesh puts its vertices on (they sit at 0° on every circle)
            let zr = z + RAY_SHIFT;
            let mut ray: Vec<[f64; 2]> = Vec::with_capacity(2);
            for (k, l) in [(0, 1), (1, 2), (2, 0)] {
                let (Some(u), Some(v)) = (q.get(k), q.get(l)) else { continue };
                if (u.0 - zr) * (v.0 - zr) < 0.0 {
                    let s = (zr - u.0) / (v.0 - u.0);
                    ray.push([u.1 + (v.1 - u.1) * s, u.2 + (v.2 - u.2) * s]);
                }
            }
            let (sn, cs) = RAY.sin_cos();
            let turn = |p: &[f64; 2]| [cs * p[0] + sn * p[1], cs * p[1] - sn * p[0]];
            if let (2, Some(a), Some(b)) = (ray.len(), ray.first().map(turn), ray.get(1).map(turn))
                && (a[1] > 0.0) != (b[1] > 0.0)
                && a[0] + (0.0 - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) > 0.0
                && let Some(c) = crossings.get_mut(i)
            {
                *c += 1;
            }
        }
    }
    // a sample no triangle reached (between two of a coarse mesh): its neighbours'
    for i in 0..n {
        let Some(o) = od.get(i).copied() else { continue };
        if o.is_finite() {
            continue;
        }
        let near = (1..n).find_map(|k| {
            let a = i.checked_sub(k).and_then(|j| od.get(j)).copied().filter(|v| v.is_finite());
            let b = od.get(i + k).copied().filter(|v| v.is_finite());
            a.or(b).map(|v| (v, k))
        });
        if let (Some((v, k)), Some(slot)) = (near, od.get_mut(i)) {
            *slot = v;
            let iv = i.checked_sub(k).and_then(|j| id.get(j)).or_else(|| id.get(i + k)).copied().unwrap_or(0.0);
            if let Some(d) = id.get_mut(i) {
                *d = iv;
            }
        }
    }
    // solid to the axis where the axis lies in the material; else a bore, as wide as the cut's
    // nearest point
    for ((d, o), c) in id.iter_mut().zip(&od).zip(&crossings) {
        if !d.is_finite() || c % 2 == 1 || *d < 1e-6 || *d >= o - 0.01 {
            *d = 0.0;
        }
    }
    if od.iter().any(|v| !v.is_finite()) {
        return Err(CamError::Geometry("the part's profile could not be read".into()));
    }
    Ok(Profile { length, dz, od, id, exact: false })
}
