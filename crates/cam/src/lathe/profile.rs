//! The turned part's profile: its outer and inner radius along its axis, sampled finely from the
//! solid. The part is meshed within [`MESH_TOL`] and cut by a plane square to Z at every sample:
//! the farthest point of the cut from the axis is the outer radius there, the nearest the inner
//! (zero where the part is solid to the axis).

use solidcraft_kernel::Body;

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

/// The profile of `body`, turned about its Z axis with its `front` end away from the chuck.
pub fn read(body: &Body, front: Front) -> Result<Profile> {
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
    Ok(Profile { length, dz, od, id })
}
