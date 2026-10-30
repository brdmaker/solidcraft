//! Machining features read from an exact solid, for a 3-axis router cutting from the top (+Z).
//!
//! Every face is classified by its own surface: upward flat faces are floors (their outlines
//! read exactly from the B-rep edges), vertical cylinders and cones on one axis make hole stacks
//! (through holes, counterbores, countersinks, stepped recesses), sloped bands along an outline
//! are chamfers. The tessellation is used only for topology (which edges bound a face) and to
//! sample each face's extent and side; no toolpath geometry comes from triangles.

use serde::Serialize;
use solidcraft_geom::{Mesh, Plane, Vec2, Vec3};
use solidcraft_kernel::{Body, FaceSurface, face_surface};

use crate::region::{Area, Ring, signed_area};
use crate::{CamError, Result};

/// How closely curved outlines are drawn when an area is built from them (mm).
pub const AREA_TOL: f64 = 0.002;

/// A face this far from vertical or horizontal (radians) is sloped.
const ANGLE_TOL: f64 = 1e-4;

/// The most faces a part may have.
pub const MAX_FACES: usize = 20_000;

/// One step of a hole stack, from its top down.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct HoleStep {
    /// `cylinder` or `cone`.
    pub kind: &'static str,
    pub z_top: f64,
    pub z_bottom: f64,
    pub r_top: f64,
    pub r_bottom: f64,
}

/// Concave round faces on one vertical axis, top down.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct HoleStack {
    pub center: [f64; 2],
    /// `hole`, `counterbore`, `countersink`, `stepped` (more than one step of another shape).
    pub kind: &'static str,
    pub through: bool,
    pub z_top: f64,
    pub z_bottom: f64,
    /// Diameter of the narrowest step: the drill or bore size.
    pub diameter: f64,
    pub steps: Vec<HoleStep>,
}

/// An upward flat face.
#[derive(Clone, Debug, Serialize)]
pub struct Floor {
    pub face: usize,
    pub z: f64,
    pub area: f64,
    pub bounds: [[f64; 2]; 2],
    /// The top face of the stock.
    pub top: bool,
    /// The floor of a step in a hole stack (a counterbore's seat).
    pub in_hole: bool,
    #[serde(skip)]
    pub region: Area,
}

/// A sloped band along an edge, cut with a chamfer mill.
#[derive(Clone, Debug, Serialize)]
pub struct Chamfer {
    pub z_top: f64,
    pub z_bottom: f64,
    /// Horizontal width of the band.
    pub width: f64,
    /// Angle of the band from horizontal (degrees).
    pub angle_deg: f64,
    /// `outline` (round the outside of the part) or `opening` (round a pocket, cutout or hole).
    pub on: &'static str,
    pub faces: Vec<usize>,
    #[serde(skip)]
    pub region: Area,
}

/// A through opening that isn't a round hole: a cutout, cut round its outline.
#[derive(Clone, Debug, Serialize)]
pub struct Cutout {
    pub area: f64,
    pub bounds: [[f64; 2]; 2],
    #[serde(skip)]
    pub ring: Ring,
}

/// A face the job cannot machine (undercut, horizontal hole) or this version cannot yet.
#[derive(Clone, Debug, Serialize)]
pub struct FaceNote {
    pub face: usize,
    pub surface: &'static str,
    pub reason: String,
}

/// Everything read from one part.
#[derive(Clone, Debug, Serialize)]
pub struct PartFeatures {
    pub top_z: f64,
    pub bottom_z: f64,
    pub bounds: [[f64; 2]; 2],
    /// Area of the part seen from above (through openings excluded).
    pub silhouette_area: f64,
    /// Area inside the outer outline (openings included).
    pub outline_area: f64,
    pub floors: Vec<Floor>,
    pub holes: Vec<HoleStack>,
    pub chamfers: Vec<Chamfer>,
    pub cutouts: Vec<Cutout>,
    pub unsupported: Vec<FaceNote>,
    #[serde(skip)]
    pub silhouette: Area,
    #[serde(skip)]
    pub outline: Area,
}

/// A concave vertical round face: face, `cylinder` or `cone`, centre, z top and bottom, radius at
/// the top and bottom, and whether its surface goes all the way round.
type Round = (usize, &'static str, [f64; 2], f64, f64, f64, f64, bool);

/// A face as the mesh samples it.
struct Sample {
    points: Vec<Vec3>,
    /// Area-weighted normal sum and the triangles' centroids with their normals.
    normal: Vec3,
    tris: Vec<(Vec3, Vec3, [Vec3; 3])>,
    zmin: f64,
    zmax: f64,
}

fn samples(mesh: &Mesh, faces: usize) -> Vec<Sample> {
    let mut s: Vec<Sample> = (0..faces)
        .map(|_| Sample { points: Vec::new(), normal: Vec3::ZERO, tris: Vec::new(), zmin: f64::INFINITY, zmax: f64::NEG_INFINITY })
        .collect();
    for (t, f) in mesh.triangles.iter().zip(&mesh.tri_face) {
        let (Some(tri), Some(slot)) = (mesh.tri(t), s.get_mut(*f as usize)) else { continue };
        let [a, b, c] = tri;
        let n = (b - a).cross(c - a);
        slot.normal += n;
        slot.tris.push(((a + b + c) / 3.0, n, tri));
        for p in tri {
            slot.zmin = slot.zmin.min(p.z);
            slot.zmax = slot.zmax.max(p.z);
            slot.points.push(p);
        }
    }
    s
}

/// Is the round face concave (material outside the circle, as in a hole)? Most of its area's
/// normals point at the axis.
fn concave(s: &Sample, c: [f64; 2]) -> bool {
    let toward: f64 = s.tris.iter().map(|(p, n, _)| -(n.x * (p.x - c[0]) + n.y * (p.y - c[1]))).sum();
    toward > 0.0
}

/// Does the round face go all the way round its axis? No gap between the angles of its points
/// is wider than a quarter turn.
fn full_turn<'a>(points: impl Iterator<Item = &'a Vec3>, c: [f64; 2]) -> bool {
    let mut a: Vec<f64> = points.map(|p| (p.y - c[1]).atan2(p.x - c[0])).filter(|a| a.is_finite()).collect();
    if a.len() < 3 {
        return false;
    }
    a.sort_by(f64::total_cmp);
    let wrap = a.first().copied().unwrap_or(0.0) + std::f64::consts::TAU - a.last().copied().unwrap_or(0.0);
    a.windows(2).filter_map(|w| Some(w.get(1)? - w.first()?)).fold(wrap, f64::max) < std::f64::consts::FRAC_PI_2
}

fn round_to(v: f64, step: f64) -> f64 {
    (v / step).round() * step
}

/// The projection of a face seen from above: the union of its triangles' shadows.
fn shadow(s: &Sample) -> Result<Area> {
    let rings: Vec<Ring> = s
        .tris
        .iter()
        .map(|(_, _, [a, b, c])| {
            let r = vec![[a.x, a.y], [b.x, b.y], [c.x, c.y]];
            if signed_area(&r) < 0.0 { vec![[a.x, a.y], [c.x, c.y], [b.x, b.y]] } else { r }
        })
        .filter(|r| signed_area(r).abs() > 1e-12)
        .collect();
    Area::union_of(&rings)
}

/// Read the machining features of `body`, cut from the top (+Z).
pub fn read_part(body: &Body) -> Result<PartFeatures> {
    let nf = body.face_count();
    if nf == 0 || nf > MAX_FACES {
        return Err(CamError::Invalid(format!("a part needs 1 to {MAX_FACES} faces; this one has {nf}")));
    }
    if body.is_mesh() {
        return Err(CamError::Unsupported("toolpaths from a mesh body: convert it to a solid first (mesh.convert)".into()));
    }
    let size = body.size();
    let tol = (size * 1e-6).max(1e-6);
    let mesh = body.tessellate((size * 2e-4).clamp(0.002, 0.05))?;
    let samples = samples(&mesh, nf);
    let top_z = samples.iter().map(|s| s.zmax).fold(f64::NEG_INFINITY, f64::max);
    let bottom_z = samples.iter().map(|s| s.zmin).fold(f64::INFINITY, f64::min);
    if !(top_z.is_finite() && bottom_z.is_finite() && top_z - bottom_z > tol) {
        return Err(CamError::Invalid("the part has no height".into()));
    }

    let mut floors = Vec::new();
    let mut rounds: Vec<Round> = Vec::new();
    let mut sloped: Vec<(usize, f64)> = Vec::new(); // face, angle from horizontal (rad)
    let mut unsupported = Vec::new();
    let note = |face: usize, surface: &'static str, reason: &str| FaceNote { face, surface, reason: reason.into() };

    for (face, s) in samples.iter().enumerate() {
        if s.tris.is_empty() {
            continue;
        }
        let Some(surface) = face_surface(body, face) else {
            unsupported.push(note(face, "freeform", "not a plane, cylinder or cone: needs 3D finishing (not supported yet)"));
            continue;
        };
        match surface {
            FaceSurface::Plane { normal, .. } => {
                let n = normal.vec();
                if n.z > 1.0 - ANGLE_TOL {
                    floors.push(floor(body, &mesh, face, s, top_z, tol)?);
                } else if n.z < -1.0 + ANGLE_TOL {
                    if (s.zmax - bottom_z).abs() > tol {
                        unsupported.push(note(face, "plane", "faces down above the bottom: an undercut a router cannot reach from the top"));
                    }
                } else if n.z.abs() <= ANGLE_TOL {
                    // a vertical wall: bounds the floors beside it
                } else if n.z > 0.0 {
                    sloped.push((face, n.z.clamp(-1.0, 1.0).acos()));
                } else {
                    unsupported.push(note(face, "plane", "slopes outward under the part: an undercut"));
                }
            }
            FaceSurface::Cylinder { origin, axis, radius } => {
                let a = axis.vec();
                if a.z.abs() < 1.0 - ANGLE_TOL {
                    unsupported.push(note(face, "cylinder", "a round face whose axis is not vertical: needs 3D finishing (not supported yet)"));
                    continue;
                }
                let c = [origin.x, origin.y];
                if concave(s, c) {
                    rounds.push((face, "cylinder", c, s.zmax, s.zmin, radius, radius, false));
                }
                // convex vertical cylinders are rounded walls: bounds of the floors
            }
            FaceSurface::Cone { apex, axis, half_angle } => {
                let a = axis.vec();
                if a.z.abs() < 1.0 - ANGLE_TOL {
                    unsupported.push(note(face, "cone", "a cone whose axis is not vertical: needs 3D finishing (not supported yet)"));
                    continue;
                }
                let c = [apex.x, apex.y];
                let r = |z: f64| (z - apex.z).abs() * half_angle.tan();
                let (rt, rb) = (r(s.zmax), r(s.zmin));
                if rt < rb {
                    unsupported.push(note(face, "cone", "narrows upward: an undercut"));
                } else if concave(s, c) {
                    rounds.push((face, "cone", c, s.zmax, s.zmin, rt, rb, false));
                } else {
                    // a chamfer round a corner of an outline or opening
                    sloped.push((face, std::f64::consts::FRAC_PI_2 - half_angle));
                }
            }
            FaceSurface::Sphere { .. } => unsupported.push(note(face, "sphere", "needs 3D finishing with a ball mill (not supported yet)")),
            FaceSurface::Torus { .. } => unsupported.push(note(face, "torus", "a rounded edge: needs 3D finishing (not supported yet)")),
        }
    }

    // The kernel may split one round surface into several faces (a cylinder into two halves):
    // join faces of one kind, axis, radii and height range, then ask whether they go all the
    // way round. A round surface that doesn't is a corner of a pocket or opening (a cylinder,
    // bounding the floors beside it) or a chamfer round such a corner (a cone).
    let same = |a: &Round, b: &Round| {
        a.1 == b.1
            && (a.2[0] - b.2[0]).hypot(a.2[1] - b.2[1]) <= tol * 100.0
            && (a.3 - b.3).abs() <= tol * 10.0
            && (a.4 - b.4).abs() <= tol * 10.0
            && (a.5 - b.5).abs() <= tol * 10.0
            && (a.6 - b.6).abs() <= tol * 10.0
    };
    let mut groups: Vec<Vec<Round>> = Vec::new();
    for r in rounds {
        match groups.iter_mut().find(|g| g.first().is_some_and(|f| same(f, &r))) {
            Some(g) => g.push(r),
            None => groups.push(vec![r]),
        }
    }
    let mut rounds: Vec<Round> = Vec::new();
    for g in groups {
        let Some(first) = g.first().copied() else { continue };
        let points = g.iter().filter_map(|r| samples.get(r.0)).flat_map(|s| s.points.iter());
        if full_turn(points, first.2) {
            rounds.extend(g.into_iter().map(|mut r| {
                r.7 = true;
                r
            }));
        } else if first.1 == "cone" {
            for r in g {
                if let Some(FaceSurface::Cone { half_angle, .. }) = face_surface(body, r.0) {
                    sloped.push((r.0, std::f64::consts::FRAC_PI_2 - half_angle));
                }
            }
        }
    }
    let holes = hole_stacks(&rounds, bottom_z, tol);
    for f in &mut floors {
        f.in_hole = !f.top && holes.iter().any(|h| floor_in_hole(f, h, tol));
    }

    // The part seen from above: its floors and the shadows of its sloped faces and cones.
    let mut silhouette = Area::empty();
    for f in &floors {
        silhouette = silhouette.union(&f.region)?;
    }
    let mut chamfer_faces = Vec::new();
    for (face, angle) in &sloped {
        if let Some(s) = samples.get(*face) {
            let sh = shadow(s)?;
            silhouette = silhouette.union(&sh)?;
            chamfer_faces.push((*face, *angle, s.zmax, s.zmin, sh));
        }
    }
    for (face, ..) in &rounds {
        if let Some(s) = samples.get(*face).filter(|_| matches!(face_surface(body, *face), Some(FaceSurface::Cone { .. }))) {
            silhouette = silhouette.union(&shadow(s)?)?;
        }
    }
    let outline = Area::union_of(&silhouette.shapes.iter().filter_map(|s| s.first().cloned()).collect::<Vec<_>>())?;
    let chamfers = chamfer_bands(chamfer_faces, &outline, tol)?;

    // Through openings that are not round holes.
    let mut cutouts = Vec::new();
    for ring in silhouette.shapes.iter().flat_map(|s| s.iter().skip(1)) {
        let area = -signed_area(ring);
        let b = ring_bounds(ring);
        let c = [(b[0][0] + b[1][0]) / 2.0, (b[0][1] + b[1][1]) / 2.0];
        let round = holes.iter().any(|h| h.through && (h.center[0] - c[0]).hypot(h.center[1] - c[1]) < (h.diameter * 1e-3).max(tol * 10.0));
        if !round {
            cutouts.push(Cutout { area, bounds: b, ring: ring.clone() });
        }
    }

    let bounds = silhouette.bounds().map(|(lo, hi)| [lo, hi]).unwrap_or([[0.0; 2]; 2]);
    Ok(PartFeatures {
        top_z,
        bottom_z,
        bounds,
        silhouette_area: silhouette.area(),
        outline_area: outline.area(),
        floors,
        holes,
        chamfers,
        cutouts,
        unsupported,
        silhouette,
        outline,
    })
}

fn ring_bounds(r: &Ring) -> [[f64; 2]; 2] {
    r.iter()
        .fold([[f64::INFINITY; 2], [f64::NEG_INFINITY; 2]], |[lo, hi], p| [[lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])]])
}

/// An upward flat face and its exact outline.
fn floor(body: &Body, mesh: &Mesh, face: usize, s: &Sample, top_z: f64, tol: f64) -> Result<Floor> {
    let z = (s.zmin + s.zmax) / 2.0;
    let plane = Plane::new(Vec3::new(0.0, 0.0, z), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0))
        .ok_or_else(|| CamError::Geometry("floor plane".into()))?;
    let f = u32::try_from(face).map_err(|_| CamError::Invalid("face index".into()))?;
    let idx: Vec<usize> = mesh.edge_faces.iter().enumerate().filter(|(_, fs)| fs.contains(&f)).map(|(i, _)| i).collect();
    let exact = body.planar_edges(&plane, &idx);
    // Each edge as a polyline: exact where the kernel reads it, else the mesh's own polyline.
    let mut pieces: Vec<Vec<[f64; 2]>> = Vec::new();
    for (i, segs) in idx.iter().zip(exact) {
        let pts: Vec<[f64; 2]> = match segs {
            Some(segs) => segs.iter().flat_map(|seg| seg.polyline(AREA_TOL)).map(|p: Vec2| [p.x, p.y]).collect(),
            None => mesh.edges.get(*i).map(|e| e.iter().map(|p| [p.x, p.y]).collect()).unwrap_or_default(),
        };
        if pts.len() >= 2 {
            pieces.push(pts);
        }
    }
    let rings = chain(pieces, tol.max(1e-5));
    let region = Area::from_rings(&rings)?;
    let bounds = region.bounds().map(|(lo, hi)| [lo, hi]).unwrap_or([[0.0; 2]; 2]);
    Ok(Floor { face, z, area: region.area(), bounds, top: (z - top_z).abs() <= tol, in_hole: false, region })
}

/// Joins polylines end to end into closed rings.
fn chain(mut pieces: Vec<Vec<[f64; 2]>>, tol: f64) -> Vec<Ring> {
    let near = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]) <= tol;
    let mut rings = Vec::new();
    let mut guard = 0usize;
    while let Some(mut ring) = pieces.pop() {
        loop {
            guard += 1;
            if guard > 1_000_000 {
                return rings;
            }
            let (Some(&first), Some(&last)) = (ring.first(), ring.last()) else { break };
            if ring.len() > 2 && near(first, last) {
                ring.pop();
                rings.push(ring);
                break;
            }
            let next = pieces.iter().position(|p| p.first().is_some_and(|q| near(*q, last)) || p.last().is_some_and(|q| near(*q, last)));
            let Some(i) = next else { break };
            let mut p = pieces.swap_remove(i);
            if !p.first().is_some_and(|q| near(*q, last)) {
                p.reverse();
            }
            ring.extend(p.into_iter().skip(1));
        }
    }
    rings
}

/// Concave full round faces grouped by axis into stacks, top down.
fn hole_stacks(rounds: &[Round], bottom_z: f64, tol: f64) -> Vec<HoleStack> {
    let mut stacks: Vec<HoleStack> = Vec::new();
    for &(_, kind, c, zt, zb, rt, rb, _) in rounds.iter().filter(|r| r.7) {
        let step = HoleStep { kind, z_top: zt, z_bottom: zb, r_top: rt, r_bottom: rb };
        let same_step = |t: &HoleStep| {
            t.kind == step.kind
                && (t.z_top - step.z_top).abs() <= tol * 10.0
                && (t.z_bottom - step.z_bottom).abs() <= tol * 10.0
                && (t.r_top - step.r_top).abs() <= tol * 10.0
        };
        match stacks.iter_mut().find(|s| (s.center[0] - c[0]).hypot(s.center[1] - c[1]) <= tol * 100.0) {
            Some(s) if s.steps.iter().any(same_step) => {}
            Some(s) => s.steps.push(step),
            None => stacks.push(HoleStack { center: c, kind: "hole", through: false, z_top: zt, z_bottom: zb, diameter: 0.0, steps: vec![step] }),
        }
    }
    for s in &mut stacks {
        s.steps.sort_by(|a, b| b.z_top.total_cmp(&a.z_top));
        s.z_top = s.steps.iter().map(|t| t.z_top).fold(f64::NEG_INFINITY, f64::max);
        s.z_bottom = s.steps.iter().map(|t| t.z_bottom).fold(f64::INFINITY, f64::min);
        s.through = (s.z_bottom - bottom_z).abs() <= tol * 10.0;
        s.diameter = 2.0 * s.steps.iter().map(|t| t.r_top.min(t.r_bottom)).fold(f64::INFINITY, f64::min);
        let cyl: Vec<&HoleStep> = s.steps.iter().filter(|t| t.kind == "cylinder").collect();
        let cone_on_top = s.steps.first().is_some_and(|t| t.kind == "cone");
        s.kind = match (cyl.len(), cone_on_top, s.steps.len()) {
            (1, false, 1) => "hole",
            (1, true, 2) => "countersink",
            (2, false, 2) if cyl.first().zip(cyl.get(1)).is_some_and(|(a, b)| a.r_top > b.r_top) => "counterbore",
            _ => "stepped",
        };
    }
    stacks.sort_by(|a, b| a.center[0].total_cmp(&b.center[0]).then(a.center[1].total_cmp(&b.center[1])));
    stacks
}

/// Is the floor the seat of a step in the stack: inside the stack's widest circle?
fn floor_in_hole(f: &Floor, h: &HoleStack, tol: f64) -> bool {
    let r = h.steps.iter().map(|t| t.r_top.max(t.r_bottom)).fold(0.0, f64::max);
    f.z < h.z_top + tol && f.z > h.z_bottom - tol && {
        let [lo, hi] = f.bounds;
        [lo, hi, [lo[0], hi[1]], [hi[0], lo[1]]].iter().all(|p| (p[0] - h.center[0]).hypot(p[1] - h.center[1]) <= r * std::f64::consts::SQRT_2 + tol)
    }
}

/// Sloped faces joined into bands (connected shadows of one height range and angle).
fn chamfer_bands(faces: Vec<(usize, f64, f64, f64, Area)>, outline: &Area, tol: f64) -> Result<Vec<Chamfer>> {
    let mut bands: Vec<Chamfer> = Vec::new();
    for (face, angle, zt, zb, sh) in faces {
        let key = |c: &Chamfer| {
            (c.z_top - zt).abs() <= tol * 10.0 && (c.z_bottom - zb).abs() <= tol * 10.0 && (c.angle_deg - angle.to_degrees()).abs() < 0.01
        };
        let touching = |c: &Chamfer| {
            key(c) && !c.region.offset(tol.max(1e-3) * 10.0, 0.01).and_then(|g| g.intersection(&sh)).map(|a| a.is_empty()).unwrap_or(true)
        };
        match bands.iter_mut().find(|c| touching(c)) {
            Some(c) => {
                c.region = c.region.union(&sh)?;
                c.faces.push(face);
            }
            None => bands.push(Chamfer {
                z_top: zt,
                z_bottom: zb,
                width: (zt - zb) / angle.tan().max(1e-9),
                angle_deg: round_to(angle.to_degrees(), 1e-6),
                on: "opening",
                faces: vec![face],
                region: sh,
            }),
        }
    }
    // A band whose outer edge is the part's outline runs round the outside.
    let outer = outline.area();
    for c in &mut bands {
        let filled = Area::union_of(&c.region.shapes.iter().filter_map(|s| s.first().cloned()).collect::<Vec<_>>())?;
        if (filled.area() - outer).abs() <= outer * 1e-6 + 1e-3 {
            c.on = "outline";
        }
    }
    Ok(bands)
}
