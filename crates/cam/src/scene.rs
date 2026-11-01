//! The sheet and its placed parts as models, for a machine's own viewer (a LinuxCNC screen, say):
//! binary STL files and a JSON file saying where everything is.
//!
//! Every model is ready placed, in work coordinates (mm): Z0 the sheet's top, X right, Y back,
//! work zero where `stock.origin` puts it. Each copy of each part has its own STL, turned as it
//! is machined and moved to where it lies on the sheet, so a viewer just loads the files;
//! `placed.stl` holds every copy in one, `job.ngc` the LinuxCNC program. [`zip`] packs the lot
//! into one archive.

use schemars::JsonSchema;
use serde::Serialize;
use solidcraft_geom::Vec3;
use solidcraft_kernel::Body;

use crate::job::{Job, Origin, Up};
use crate::plan::JobPlan;
use crate::{CamError, Result};

/// Chord tolerance of the part models (mm).
pub const MODEL_TOL: f64 = 0.02;

/// What the scene file says.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct SceneDoc {
    /// The format's version: 1.
    pub version: u32,
    /// Always `mm`.
    pub units: String,
    /// Where work zero is on the sheet's top.
    pub work_zero: Origin,
    pub stock: SceneStock,
    pub parts: Vec<ScenePart>,
    /// Every copy placed, in one model (work coordinates).
    pub placed_model: String,
    /// The LinuxCNC program for the job (same work zero).
    pub program: String,
    /// Copies that did not fit on the sheet.
    pub unplaced: Vec<String>,
}

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct SceneStock {
    /// Width (X), depth (Y) and thickness (Z).
    pub size: [f64; 3],
    /// Its box in work coordinates.
    pub min: [f64; 3],
    pub max: [f64; 3],
    pub material: String,
    pub model: String,
}

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct ScenePart {
    pub name: String,
    pub design: String,
    pub body: String,
    /// Which of the design's axes is up (the model is turned so).
    pub up: Up,
    pub copies: Vec<SceneCopy>,
}

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct SceneCopy {
    /// 1-based, as in the operations' names (`Bracket #2`).
    pub copy: u32,
    /// The copy's model, already turned and placed (work coordinates).
    pub model: String,
    /// How the nesting turned it about Z, counter-clockwise, degrees (0 or 90); for reference,
    /// the model already is.
    pub rotation_deg: f64,
    /// The copy's box in work coordinates.
    pub min: [f64; 3],
    pub max: [f64; 3],
}

/// The scene's files: `scene.json` and the models, each with its path inside the folder.
pub struct SceneFiles {
    pub doc: SceneDoc,
    pub json: String,
    pub files: Vec<(String, Vec<u8>)>,
}

/// Triangles as corner triples.
type Tris = Vec<[Vec3; 3]>;

/// Binary STL of `tris`.
pub fn stl(name: &str, tris: &[[Vec3; 3]]) -> Vec<u8> {
    let mut out = Vec::with_capacity(84 + 50 * tris.len());
    let mut header = [b' '; 80];
    for (h, c) in header.iter_mut().zip(format!("SolidCraft CAM {name}").bytes()) {
        *h = c;
    }
    out.extend_from_slice(&header);
    out.extend_from_slice(&u32::try_from(tris.len()).unwrap_or(u32::MAX).to_le_bytes());
    for t in tris.iter().take(u32::MAX as usize) {
        let n = (t[1] - t[0]).cross(t[2] - t[0]);
        let len = n.len();
        let n = if len > 0.0 { n * (1.0 / len) } else { Vec3::new(0.0, 0.0, 0.0) };
        for v in std::iter::once(n).chain(t.iter().copied()) {
            for c in [v.x, v.y, v.z] {
                out.extend_from_slice(&(c as f32).to_le_bytes());
            }
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

/// A box's twelve triangles, facing out.
pub fn stock_tris(lo: [f64; 3], hi: [f64; 3]) -> Tris {
    let p =
        |i: usize| Vec3::new(if i & 1 == 0 { lo[0] } else { hi[0] }, if i & 2 == 0 { lo[1] } else { hi[1] }, if i & 4 == 0 { lo[2] } else { hi[2] });
    // each face as a quad, corners anticlockwise seen from outside
    let quads = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];
    quads.iter().flat_map(|q| [[p(q[0]), p(q[1]), p(q[2])], [p(q[0]), p(q[2]), p(q[3])]]).collect()
}

/// A file name from a label: letters, digits and `-`.
fn slug(s: &str) -> String {
    let t: String = s.chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_lowercase() } else { '-' }).collect();
    let t = t.trim_matches('-').to_string();
    if t.is_empty() { "part".into() } else { t }
}

/// The scene of a planned job: `bodies` are the parts' bodies as designed (in `job.parts`
/// order), `plan` the plan made from them.
pub fn export(job: &Job, bodies: &[Body], plan: &JobPlan) -> Result<SceneFiles> {
    if bodies.len() != job.parts.len() || plan.ups.len() != job.parts.len() || plan.features.len() != job.parts.len() {
        return Err(CamError::Invalid("the scene needs every part's body, as planned".into()));
    }
    let [w, d] = job.stock.size;
    let t = job.stock.thickness;
    let zero = job.stock.work_zero();
    let (smin, smax) = ([-zero[0], -zero[1], -t], [w - zero[0], d - zero[1], 0.0]);
    let mut files = vec![("stock.stl".to_string(), stl("stock", &stock_tris(smin, smax)))];
    let mut placed: Tris = Vec::new();
    let mut parts = Vec::new();
    for (i, ((p, body), (up, f))) in job.parts.iter().zip(bodies).zip(plan.ups.iter().zip(&plan.features)).enumerate() {
        let turned = crate::part::turned(body, *up)?;
        let mesh = turned.tessellate(MODEL_TOL)?;
        let tris: Tris = mesh.triangles.iter().filter_map(|tr| mesh.tri(tr)).collect();
        let mut copies = Vec::new();
        for pl in plan.nesting.placements.iter().filter(|pl| pl.part == i) {
            // exact quarter turns
            let (s, c) = match pl.quarter_turns % 4 {
                1 => (1.0, 0.0),
                2 => (0.0, -1.0),
                3 => (-1.0, 0.0),
                _ => (0.0, 1.0),
            };
            // the part's bottom on the sheet's bottom, as the toolpaths are placed
            let tr = [pl.offset[0], pl.offset[1], -(f.bottom_z + t)];
            let place = |v: Vec3| Vec3::new(c * v.x - s * v.y + tr[0], s * v.x + c * v.y + tr[1], v.z + tr[2]);
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            let mut mine: Tris = Vec::with_capacity(tris.len());
            for tri in &tris {
                let q = [place(tri[0]), place(tri[1]), place(tri[2])];
                for v in &q {
                    for (k, x) in [v.x, v.y, v.z].into_iter().enumerate() {
                        lo[k] = lo[k].min(x);
                        hi[k] = hi[k].max(x);
                    }
                }
                mine.push(q);
            }
            // to a nanometre: no 2.9999999999999 in the file
            let r = |x: f64| (x * 1e9).round() / 1e9;
            let model = format!("parts/{}-{}-{}.stl", i + 1, slug(p.label()), pl.copy + 1);
            files.push((model.clone(), stl(&format!("{} #{}", p.label(), pl.copy + 1), &mine)));
            placed.extend(mine);
            copies.push(SceneCopy { copy: pl.copy + 1, model, rotation_deg: f64::from(pl.quarter_turns % 4) * 90.0, min: lo.map(r), max: hi.map(r) });
        }
        parts.push(ScenePart { name: p.label().to_string(), design: p.design.clone(), body: p.body.clone(), up: *up, copies });
    }
    files.push(("placed.stl".to_string(), stl("placed parts", &placed)));
    let title = job.parts.iter().map(|p| p.label()).collect::<Vec<_>>().join(", ");
    files.push(("job.ngc".to_string(), crate::post::linuxcnc(job, &plan.ops, &title)?.into_bytes()));
    let doc = SceneDoc {
        version: 1,
        units: "mm".into(),
        work_zero: job.stock.origin,
        stock: SceneStock { size: [w, d, t], min: smin, max: smax, material: job.stock.material.clone(), model: "stock.stl".into() },
        parts,
        placed_model: "placed.stl".into(),
        program: "job.ngc".into(),
        unplaced: plan.nesting.unplaced.iter().map(|u| format!("{} #{}: {}", u.label, u.copy + 1, u.reason)).collect(),
    };
    let json = serde_json::to_string_pretty(&doc).map_err(|e| CamError::Invalid(format!("scene: {e}")))?;
    Ok(SceneFiles { doc, json, files })
}

/// The scene packed into one zip archive: `scene.json` and every model, compressed.
pub fn zip(scene: &SceneFiles) -> Result<Vec<u8>> {
    use std::io::Write as _;
    let fail = |e: String| CamError::Invalid(format!("scene archive: {e}"));
    let mut w = ::zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
    let opts = ::zip::write::SimpleFileOptions::default().compression_method(::zip::CompressionMethod::Deflated);
    for (name, bytes) in std::iter::once(("scene.json", scene.json.as_bytes())).chain(scene.files.iter().map(|(n, b)| (n.as_str(), b.as_slice()))) {
        w.start_file(name, opts).map_err(|e| fail(e.to_string()))?;
        w.write_all(bytes).map_err(|e| fail(e.to_string()))?;
    }
    Ok(w.finish().map_err(|e| fail(e.to_string()))?.into_inner())
}
