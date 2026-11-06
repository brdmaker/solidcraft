//! A turning job as models, for a machine's own viewer: binary STL files and a JSON file saying
//! what everything is, packed like the router's scene ([`crate::scene::pack`]).
//!
//! Work coordinates (mm) as the program has them: Z0 the finished front face, Z along the spindle
//! (the part toward −Z, the chuck behind it), X out from the axis. The models are solids of
//! revolution about the Z axis, X and Y radial: `stock.stl` the bar as set up (with its length in
//! the chuck), `part.stl` the part as designed, placed where it is turned, `turned.stl` the bar
//! as the simulation leaves it (what is still held in the chuck, and the part if it was not
//! parted off; a thread shows as rings at its minor diameter). `job.ngc` is the program.

use schemars::JsonSchema;
use serde::Serialize;
use solidcraft_geom::Vec3;
use solidcraft_kernel::Body;

use super::job::{Front, LatheJob, LatheToolKind, ToolSide};
use super::plan::{LatheOpKind, LathePlan, Spindle, TipMove};
use super::sim::LatheSimReport;
use crate::region::{Area, signed_area};
use crate::{CamError, Result};

/// Chord tolerance of the part's model, mm.
pub const MODEL_TOL: f64 = 0.02;
/// Segments round the axis of the revolved models.
const SEGMENTS: usize = 120;

/// What the lathe scene file says.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct LatheSceneDoc {
    /// The format's version: 1.
    pub version: u32,
    /// Always `lathe`: the router's scene says `units` and `work_zero` instead.
    pub machine_type: String,
    /// Always `mm`.
    pub units: String,
    pub machine: SceneMachine,
    /// Where work zero is and how the axes run.
    pub work_zero: String,
    pub stock: SceneBar,
    pub part: SceneTurnedPart,
    /// The bar as the simulation leaves it, revolved.
    pub turned_model: String,
    /// The LinuxCNC program.
    pub program: String,
    /// The tools the program loads, in the order it first loads them: what a tool table needs.
    pub tools: Vec<SceneLatheTool>,
    pub operations: Vec<SceneLatheOp>,
    pub threads: Vec<SceneThread>,
    pub warnings: Vec<String>,
    pub simulation: LatheSimReport,
}

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct SceneMachine {
    pub name: String,
    pub max_rpm: f64,
    pub z_travel: f64,
    /// The turret's stations (tool numbers); empty: any tool.
    pub turret: Vec<u32>,
    /// `front` (toward the operator) or `back`.
    pub tool_side: String,
}

/// The bar as set up: a cylinder about Z.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct SceneBar {
    pub diameter: f64,
    pub length: f64,
    pub material: String,
    /// Length held in the chuck, and standing out of it.
    pub in_chuck: f64,
    pub stick_out: f64,
    /// Z of the bar's front before facing, of its far end in the chuck, of the jaws' face.
    pub front_z: f64,
    pub back_z: f64,
    pub jaws_z: f64,
    /// Faced off the front, and left behind the part for the parting blade.
    pub face_allowance: f64,
    pub parting_allowance: f64,
    pub model: String,
}

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct SceneTurnedPart {
    pub name: String,
    pub design: String,
    pub body: String,
    /// The design's axis end that faces away from the chuck: `+z` or `-z`.
    pub front: Front,
    pub length: f64,
    pub max_diameter: f64,
    /// Z of the part's front face (0) and chuck end.
    pub z: [f64; 2],
    /// The outline as turned: [z, diameter] from the chuck end to the front, within 0.002.
    pub outline: Vec<[f64; 2]>,
    /// The bore: [z, diameter] where the part is hollow, within 0.002 (empty: solid).
    pub bore: Vec<[f64; 2]>,
    pub model: String,
}

/// One tool of the program, as the job's library has it.
#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct SceneLatheTool {
    /// `T<number> M6`; the station and the tool table's number.
    pub number: u32,
    /// Whether one of the turret's stations holds it (else it is changed by hand).
    pub in_turret: bool,
    pub name: String,
    /// `turning`, `boring`, `grooving`, `threading` or `drill`.
    pub kind: String,
    /// Where the tool is touched off: the point its moves are of.
    pub touch_off: String,
    /// Turning tools and boring bars: the insert's angle and the holder's approach, degrees.
    pub insert_angle_deg: Option<f64>,
    pub approach_deg: Option<f64>,
    pub nose_radius: f64,
    /// Grooving blades: the width; blades and boring bars: how deep they reach.
    pub width: Option<f64>,
    pub reach: Option<f64>,
    /// Drills: the diameter and the peck; drills and threading tools: the point's angle.
    pub diameter: Option<f64>,
    pub tip_angle_deg: Option<f64>,
    pub peck: Option<f64>,
    /// Boring bars: the smallest bore they go into.
    pub min_bore: Option<f64>,
    /// Feed, mm/rev, and depth of cut (a pass, a peck), mm.
    pub feed: f64,
    pub depth: f64,
    /// Cutting speed for the bar's material, m/min, and the tool's own top speed, rpm.
    pub surface_speed: f64,
    pub max_rpm: Option<f64>,
    /// The lowest diameter and Z the program takes the tool's touch-off point to.
    pub x_min: f64,
    pub z_min: f64,
}

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct SceneLatheOp {
    pub name: String,
    /// `facing`, `drilling`, `roughing`, `boring`, `finishing`, `grooving`, `threading`, `parting`.
    pub kind: String,
    pub tool: u32,
    /// `css` (constant surface speed, m/min, G96) or `rpm` (G97).
    pub spindle: String,
    pub speed: f64,
    /// mm/rev.
    pub feed: f64,
    pub moves: usize,
    /// Length fed and run rapid, mm.
    pub feed_length: f64,
    pub rapid_length: f64,
}

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct SceneThread {
    pub designation: String,
    /// The Z span of the passes (run in and out included).
    pub z: [f64; 2],
    pub major_diameter: f64,
    pub minor_diameter: f64,
    pub pitch: f64,
}

/// A scene: its JSON and the files it names.
pub struct LatheSceneFiles {
    pub doc: LatheSceneDoc,
    pub json: String,
    pub files: Vec<(String, Vec<u8>)>,
}

impl LatheSceneFiles {
    /// Packed into one zip archive.
    pub fn zip(&self) -> Result<Vec<u8>> {
        crate::scene::pack(&self.json, &self.files)
    }
}

/// The surface of the half section `area` ([z, r] in work coordinates) revolved about Z.
pub fn revolve(area: &Area) -> Vec<[Vec3; 3]> {
    let mut tris = Vec::new();
    let ang: Vec<(f64, f64)> = (0..=SEGMENTS).map(|k| (std::f64::consts::TAU * k as f64 / SEGMENTS as f64).sin_cos()).collect();
    for shape in &area.shapes {
        for (h, ring) in shape.iter().enumerate() {
            // the material lies inside the first ring and outside the others: the outward side of
            // an anticlockwise ring is to the right of its edges
            let ccw = signed_area(ring) > 0.0;
            let out = if h == 0 { ccw } else { !ccw };
            let n = ring.len();
            for i in 0..n {
                let (Some(a), Some(b)) = (ring.get(i), ring.get((i + 1) % n)) else { continue };
                let (a, b) = ([a[0], a[1].max(0.0)], [b[0], b[1].max(0.0)]);
                if a[1] <= 0.0 && b[1] <= 0.0 {
                    continue;
                }
                let d = [b[0] - a[0], b[1] - a[1]];
                // the outward normal in the section: right of the edge for an outward ring
                let nn = if out { [d[1], -d[0]] } else { [-d[1], d[0]] };
                for w in ang.windows(2) {
                    let (Some(&(s0, c0)), Some(&(s1, c1))) = (w.first(), w.get(1)) else { continue };
                    let p = |q: [f64; 2], s: f64, c: f64| Vec3::new(q[1] * c, q[1] * s, q[0]);
                    let (a0, b0, a1, b1) = (p(a, s0, c0), p(b, s0, c0), p(a, s1, c1), p(b, s1, c1));
                    let (sm, cm) = ((s0 + s1) / 2.0, (c0 + c1) / 2.0);
                    let normal = Vec3::new(nn[1] * cm, nn[1] * sm, nn[0]);
                    let mut push = |t: [Vec3; 3]| {
                        let f = (t[1] - t[0]).cross(t[2] - t[0]);
                        if f.len() < 1e-12 {
                            return;
                        }
                        tris.push(if f.dot(normal) >= 0.0 { t } else { [t[0], t[2], t[1]] });
                    };
                    push([a0, b0, b1]);
                    push([a0, b1, a1]);
                }
            }
        }
    }
    tris
}

/// The part `body`'s triangles placed in work coordinates: its axis on Z, its front face at Z0.
fn placed_part(body: &Body, front: Front) -> Result<Vec<[Vec3; 3]>> {
    let mesh = body.tessellate(MODEL_TOL)?;
    let b = mesh.bounds();
    let place = |v: &Vec3| match front {
        Front::PlusZ => Vec3::new(v.x, v.y, v.z - b.max.z),
        // turned end for end: half a turn about Y
        Front::MinusZ => Vec3::new(-v.x, v.y, b.min.z - v.z),
    };
    let mut out = Vec::with_capacity(mesh.triangles.len());
    for t in &mesh.triangles {
        let (Some(a), Some(bb), Some(c)) = (mesh.positions.get(t[0] as usize), mesh.positions.get(t[1] as usize), mesh.positions.get(t[2] as usize))
        else {
            continue;
        };
        out.push([place(a), place(bb), place(c)]);
    }
    Ok(out)
}

fn tool_kind(k: LatheToolKind) -> &'static str {
    k.name()
}

fn touch_off(k: LatheToolKind) -> &'static str {
    match k {
        LatheToolKind::Turning => "imaginary tip: where the tangents to the nose at the front and below meet",
        LatheToolKind::Boring => "imaginary tip: where the tangents to the nose at the front and outward (the bore's wall) meet",
        LatheToolKind::Grooving => "the blade's corner toward the chuck",
        LatheToolKind::Threading => "the point of the V",
        LatheToolKind::Drill => "the drill's point, on the axis",
    }
}

fn op_kind(k: LatheOpKind) -> &'static str {
    match k {
        LatheOpKind::Facing => "facing",
        LatheOpKind::Drilling => "drilling",
        LatheOpKind::Roughing => "roughing",
        LatheOpKind::Boring => "boring",
        LatheOpKind::Finishing => "finishing",
        LatheOpKind::Grooving => "grooving",
        LatheOpKind::Threading => "threading",
        LatheOpKind::Parting => "parting",
    }
}

/// The scene of a planned and simulated job: `body` is the part as designed, `report` and `left`
/// what [`super::sim::simulate`] made of `plan`.
pub fn export(job: &LatheJob, body: &Body, plan: &LathePlan, report: &LatheSimReport, left: &Area) -> Result<LatheSceneFiles> {
    let su = &plan.setup;
    let p = &plan.profile;
    let l = p.length;
    let r = |x: f64| (x * 1e6).round() / 1e6;
    let title = if job.part.name.is_empty() { job.part.body.clone() } else { job.part.name.clone() };
    let back_z = su.front - su.length;
    let mut files = Vec::new();
    let bar = Area::rect(back_z, 0.0, su.front, su.radius)?;
    files.push(("stock.stl".to_string(), crate::scene::stl("bar", &revolve(&bar))));
    files.push(("part.stl".to_string(), crate::scene::stl(&title, &placed_part(body, job.part.front)?)));
    let turned = left.simplified(super::plan::PATH_TOL)?;
    files.push(("turned.stl".to_string(), crate::scene::stl("turned", &revolve(&turned))));
    files.push(("job.ngc".to_string(), super::post::linuxcnc(job, plan, &title)?.into_bytes()));

    let mut order: Vec<u32> = Vec::new();
    for op in &plan.ops {
        if !order.contains(&op.tool) {
            order.push(op.tool);
        }
    }
    let turret = &job.machine.turret;
    let tools = order
        .iter()
        .filter_map(|n| job.tool(*n))
        .map(|t| {
            let moves = plan.ops.iter().filter(|o| o.tool == t.number).flat_map(|o| o.moves.iter()).map(TipMove::to);
            let (x_min, z_min) = moves.fold((f64::INFINITY, f64::INFINITY), |(x, z), q| (x.min(2.0 * q[0]), z.min(q[1])));
            let turning = matches!(t.kind, LatheToolKind::Turning | LatheToolKind::Boring);
            SceneLatheTool {
                number: t.number,
                in_turret: turret.is_empty() || turret.contains(&t.number),
                name: t.name.clone(),
                kind: tool_kind(t.kind).into(),
                touch_off: touch_off(t.kind).into(),
                insert_angle_deg: turning.then_some(t.insert_angle_deg),
                approach_deg: turning.then_some(t.approach_deg),
                nose_radius: t.nose_radius,
                width: (t.kind == LatheToolKind::Grooving).then(|| t.blade_width()),
                reach: matches!(t.kind, LatheToolKind::Grooving | LatheToolKind::Boring).then(|| t.reach()),
                diameter: t.diameter,
                tip_angle_deg: matches!(t.kind, LatheToolKind::Threading | LatheToolKind::Drill).then(|| t.point_angle()),
                peck: t.peck,
                min_bore: t.min_bore,
                feed: t.feed,
                depth: t.depth,
                surface_speed: t.speed(&job.stock.material).unwrap_or(0.0),
                max_rpm: t.max_rpm,
                x_min: if x_min.is_finite() { r(x_min) } else { 0.0 },
                z_min: if z_min.is_finite() { r(z_min) } else { 0.0 },
            }
        })
        .collect();
    let operations = plan
        .ops
        .iter()
        .map(|op| {
            let (spindle, speed) = match op.spindle {
                Spindle::SurfaceSpeed(v) => ("css", v),
                Spindle::Rpm(n) => ("rpm", n.min(job.machine.max_rpm).round()),
            };
            let (mut fed, mut rapid) = (0.0, 0.0);
            let mut at: Option<[f64; 2]> = None;
            for m in &op.moves {
                let q = m.to();
                if let Some(a) = at {
                    let d = (q[0] - a[0]).hypot(q[1] - a[1]);
                    if matches!(m, TipMove::Rapid(_)) { rapid += d } else { fed += d }
                }
                at = Some(q);
            }
            SceneLatheOp {
                name: op.name.clone(),
                kind: op_kind(op.kind).into(),
                tool: op.tool,
                spindle: spindle.into(),
                speed,
                feed: op.feed,
                moves: op.moves.len(),
                feed_length: r(fed),
                rapid_length: r(rapid),
            }
        })
        .collect();
    let threads = job
        .part
        .threads
        .iter()
        .zip(&plan.threads)
        .map(|(s, t)| SceneThread {
            designation: s.designation.clone(),
            z: t.z.map(r),
            major_diameter: r(2.0 * t.major),
            minor_diameter: r(2.0 * t.minor),
            pitch: t.pitch,
        })
        .collect();
    let line = |radii: &[f64]| -> Vec<[f64; 2]> {
        let pts: Vec<[f64; 2]> = radii.iter().enumerate().map(|(i, v)| [i as f64 * p.dz - l, *v]).collect();
        super::plan::simplify(&pts, 0.002).into_iter().map(|q| [r(q[0]), r(2.0 * q[1])]).collect()
    };
    let bore = if p.id.iter().any(|v| *v > 0.0) { line(&p.id) } else { Vec::new() };
    let doc = LatheSceneDoc {
        version: 1,
        machine_type: "lathe".into(),
        units: "mm".into(),
        machine: SceneMachine {
            name: job.machine.name.clone(),
            max_rpm: job.machine.max_rpm,
            z_travel: job.machine.z_travel,
            turret: turret.clone(),
            tool_side: if job.machine.tool_side == ToolSide::Front { "front" } else { "back" }.into(),
        },
        work_zero: "Z0 the finished front face, on the spindle's axis; Z toward the tailstock (the part at -Z), X out from the axis; the program's X is a diameter (G7)".into(),
        stock: SceneBar {
            diameter: job.stock.diameter,
            length: su.length,
            material: job.stock.material.clone(),
            in_chuck: job.stock.in_chuck,
            stick_out: su.stick_out,
            front_z: r(su.front),
            back_z: r(back_z),
            jaws_z: r(su.jaws),
            face_allowance: job.stock.face_allowance,
            parting_allowance: job.stock.parting_allowance,
            model: "stock.stl".into(),
        },
        part: SceneTurnedPart {
            name: title.clone(),
            design: job.part.design.clone(),
            body: job.part.body.clone(),
            front: job.part.front,
            length: r(l),
            max_diameter: r(2.0 * p.max_od()),
            z: [0.0, r(-l)],
            outline: line(&p.od),
            bore,
            model: "part.stl".into(),
        },
        turned_model: "turned.stl".into(),
        program: "job.ngc".into(),
        tools,
        operations,
        threads,
        warnings: plan.warnings.clone(),
        simulation: report.clone(),
    };
    let json = serde_json::to_string_pretty(&doc).map_err(|e| CamError::Invalid(format!("scene: {e}")))?;
    Ok(LatheSceneFiles { doc, json, files })
}
