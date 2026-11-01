//! CAM for 3-axis routers: machining features, toolpaths and G-code (`solidcraft_cam`).

use std::path::{Path, PathBuf};

use serde_json::Value;
use solidcraft_cam::job::{Job, PartRef};
use solidcraft_cam::nest::{NestItem, nest};
use solidcraft_cam::part::{PartFeatures, oriented};

use super::CommandSpec;
use crate::params::bad;
use crate::schema::cam::{
    JobExportSceneParams, JobExportSceneResult, JobGcodeParams, JobGcodeResult, JobNestParams, JobNestResult, JobSimulateParams, JobSimulateResult,
    NestedPart, PartInspectParams,
};
use crate::schema::{TypedContract, output_schema_for};
use crate::{EngineError, Result, Session};

const PART_INSPECT_CONTRACT: TypedContract<PartInspectParams> = TypedContract::new();
const JOB_NEST_CONTRACT: TypedContract<JobNestParams> = TypedContract::new();
const JOB_EXPORT_SCENE_CONTRACT: TypedContract<JobExportSceneParams> = TypedContract::new();
const JOB_GCODE_CONTRACT: TypedContract<JobGcodeParams> = TypedContract::new();
const JOB_SIMULATE_CONTRACT: TypedContract<JobSimulateParams> = TypedContract::new();

/// Largest job file read (bytes).
const MAX_JOB_BYTES: usize = 4 << 20;

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.part.inspect", "Inspect Machining Features", crate::checked!("cam.part.inspect", part_inspect))
        .icon("list")
        .noundo()
        .contract(&PART_INSPECT_CONTRACT)
        .output(output_schema_for::<solidcraft_cam::part::PartFeatures>)
        .params(
            "body: name → what a 3-axis router cutting from the top (+Z) sees in it: top_z, bottom_z, bounds, silhouette_area (seen from above, \
         through openings excluded), outline_area; floors (upward flat faces: z, area, bounds, top, in_hole); holes (concave round faces on \
         one vertical axis, top down: kind hole|counterbore|countersink|stepped, centre, through, diameter of the narrowest step, steps with \
         kind cylinder|cone, z and radius at top and bottom); chamfers (sloped bands: z range, width, angle, on outline|opening); cutouts \
         (through openings that aren't round); unsupported (faces it cannot machine, each with its reason). Read-only",
        ),
    CommandSpec::new("cam.job.export_scene", "Export Job Scene", crate::checked!("cam.job.export_scene", job_export_scene))
        .icon("export")
        .noundo()
        .contract(&JOB_EXPORT_SCENE_CONTRACT)
        .output(output_schema_for::<JobExportSceneResult>)
        .params(
            "job: as for cam.job.nest; path: a .zip file or a folder → plans the job and writes, for a machine's own viewer (a \
             LinuxCNC screen), every model ready placed in work coordinates (Z0 the sheet's top, X right, Y back, work zero at \
             stock.origin: top_center by default), no transforms needed: stock.stl, parts/<n>-<name>-<copy>.stl (each copy turned as \
             machined and placed), placed.stl (all copies) and scene.json (version, units mm, work_zero, stock {size [w, d, t], min, \
             max, material, model}, parts [{name, design, body, up, copies [{copy, model, rotation_deg (for reference), min, max}]}], \
             placed_model, unplaced). Returns the files written, the scene and the plan's warnings",
        ),
    CommandSpec::new("cam.job.gcode", "Write Job G-code", crate::checked!("cam.job.gcode", job_gcode))
        .icon("export")
        .noundo()
        .contract(&JOB_GCODE_CONTRACT)
        .output(output_schema_for::<JobGcodeResult>)
        .params(
            "job: as for cam.job.nest (post?: {coolant? off|mist|flood, spindle_wait? (2 s), decimals? (4), blend_tolerance? \
             (0.01 mm, 0: exact stop)}); path: the .ngc file → plans the job and writes its LinuxCNC program: G21 G90 G17 G94 G54, \
             work zero at stock.origin (top_center by default, Z0 the sheet's top), each tool T<n> M6 with G43 H<n> and S M3, \
             G0/G1/G2/G3 (I J from the arc's start; helices change Z), drilling as G98 G81 or G83 with Q, M30. Returns the path, \
             line count, tools in order, operations in order and the plan's warnings",
        ),
    CommandSpec::new("cam.job.simulate", "Simulate Job", crate::checked!("cam.job.simulate", job_simulate))
        .icon("play")
        .noundo()
        .contract(&JOB_SIMULATE_CONTRACT)
        .output(output_schema_for::<JobSimulateResult>)
        .params(
            "job: as for cam.job.nest; cell? (0.25 mm); image? (a .png path) → plans the job and cuts it on a height map of the \
             sheet with the real tool shapes (flat end mills, drill and chamfer cones), then compares the result with the parts \
             (finely triangulated): gouges (cut into a part by more than 0.02 mm), material left on the parts (more than 0.05 mm: \
             corners the tool cannot reach, a countersink's steps), rapids through material, cuts below the spoilboard allowance \
             (a drill's point allowed), each copy cut free or where it is held, loose slugs in through openings, and the volume \
             removed; spots are worst first with position, size and area. The image shades the machined sheet and rings the spots \
             (red gouges, orange material left, magenta held). Read-only but for the image",
        ),
    CommandSpec::new("cam.job.nest", "Nest Job", crate::checked!("cam.job.nest", job_nest))
        .icon("list")
        .noundo()
        .contract(&JOB_NEST_CONTRACT)
        .output(output_schema_for::<JobNestResult>)
        .params(
            "job: a job object or the path of a job file: {stock: {size: [width, depth], thickness, material?, spoilboard_allowance? (0.2), \
             origin? (top_center | top_front_left)}, \
             nest?: {margin? (10), spacing? (8), rotate? (true)}, tools?: [{number, kind: flat|drill|chamfer, diameter, flute_length, \
             rpm, feed, plunge, flutes?, tip_angle_deg?, stepdown?, stepover? (fraction), peck?}], parts: [{body, design? (.sc or STEP, \
             relative to the job file; empty: the open design), quantity? (1), name?, orient? (auto | +z | -z | +x | -x | +y | -y: which \
             of the part's axes is up; auto picks one that fits the sheet, then with the fewest faces unreachable from the top, then \
             the largest flat bottom)}]} → each part's axis up, height, box and outline area, \
             and the layout on the sheet in work coordinates (work zero at stock.origin: top_center (default) or top_front_left; X \
             right, Y back): placements {part, copy, \
             quarter_turns, offset, bounds}, unplaced copies with the reason, utilisation. Bounding-box nesting with 90° turns. Read-only",
        ),
];

fn part_inspect(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.part.inspect";
    let name = p.get("body").and_then(Value::as_str).ok_or_else(|| bad(cmd, "body: the name of a body"))?;
    let state = s.model.state();
    let body = state.body(name).ok_or_else(|| bad(cmd, format!("no body named `{name}`")))?;
    let features = solidcraft_cam::part::read_part(&body.body).map_err(|e| EngineError::Other(e.to_string()))?;
    serde_json::to_value(features).map_err(|e| EngineError::Other(e.to_string()))
}

fn job_nest(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job.nest";
    let (job, base) = read_job(s, p.get("job").ok_or_else(|| bad(cmd, "job: a job object or the path of a job file"))?)?;
    let mut parts = Vec::new();
    let mut features: Vec<PartFeatures> = Vec::new();
    for part in &job.parts {
        let body = part_body(s, base.as_deref(), part)?;
        let (up, _, f) = oriented(&body, part.orient, job.stock.thickness).map_err(|e| EngineError::Other(format!("{}: {e}", part.label())))?;
        let height = f.top_z - f.bottom_z;
        parts.push(NestedPart {
            label: part.label().to_string(),
            up,
            design: part.design.clone(),
            body: part.body.clone(),
            quantity: part.quantity,
            height,
            face_off: (job.stock.thickness - height).max(0.0),
            bounds: f.bounds,
            outline_area: f.outline_area,
            unsupported: f.unsupported.len(),
        });
        features.push(f);
    }
    let items: Vec<NestItem> = job
        .parts
        .iter()
        .zip(&features)
        .map(|(p, f)| NestItem { label: p.label(), quantity: p.quantity, outline: &f.outline, height: f.top_z - f.bottom_z })
        .collect();
    let nesting = nest(&job.stock, &job.nest, &items).in_work(job.stock.work_zero());
    let result = JobNestResult { sheet: job.stock.size, thickness: job.stock.thickness, parts, nesting };
    serde_json::to_value(result).map_err(|e| EngineError::Other(e.to_string()))
}

fn job_gcode(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job.gcode";
    let path = p.get("path").and_then(Value::as_str).ok_or_else(|| bad(cmd, "path: the .ngc file to write"))?.to_string();
    let (job, base) = read_job(s, p.get("job").ok_or_else(|| bad(cmd, "job: a job object or the path of a job file"))?)?;
    let bodies = job.parts.iter().map(|part| part_body(s, base.as_deref(), part)).collect::<Result<Vec<_>>>()?;
    let plan = solidcraft_cam::plan::plan_job(&job, &bodies).map_err(|e| EngineError::Other(e.to_string()))?;
    let title = job.parts.iter().map(|p| p.label()).collect::<Vec<_>>().join(", ");
    let program = solidcraft_cam::post::linuxcnc(&job, &plan.ops, &title).map_err(|e| EngineError::Other(e.to_string()))?;
    solidcraft_io::vfs::write(&path, program.as_bytes()).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
    let mut tools: Vec<u32> = Vec::new();
    for op in &plan.ops {
        if !tools.contains(&op.tool) {
            tools.push(op.tool);
        }
    }
    let result = JobGcodeResult {
        path,
        lines: program.lines().count(),
        tools,
        operations: plan.ops.iter().map(|o| o.name.clone()).collect(),
        warnings: plan.warnings,
    };
    serde_json::to_value(result).map_err(|e| EngineError::Other(e.to_string()))
}

fn job_simulate(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job.simulate";
    let cell = p.get("cell").and_then(Value::as_f64).unwrap_or(0.25);
    if !(cell.is_finite() && (0.01..=5.0).contains(&cell)) {
        return Err(bad(cmd, format!("cell must be from 0.01 to 5 mm; it is {cell}")));
    }
    let image = p.get("image").and_then(Value::as_str).map(str::to_string);
    let (job, base) = read_job(s, p.get("job").ok_or_else(|| bad(cmd, "job: a job object or the path of a job file"))?)?;
    let bodies = job.parts.iter().map(|part| part_body(s, base.as_deref(), part)).collect::<Result<Vec<_>>>()?;
    let plan = solidcraft_cam::plan::plan_job(&job, &bodies).map_err(|e| EngineError::Other(e.to_string()))?;
    let (report, map) = solidcraft_cam::sim::simulate(&job, &bodies, &plan, cell).map_err(|e| EngineError::Other(e.to_string()))?;
    if let Some(path) = &image {
        let png = solidcraft_cam::sim::png(&map, job.stock.thickness, Some(&report), 2000).map_err(|e| EngineError::Other(e.to_string()))?;
        solidcraft_io::vfs::write(path, &png).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
    }
    let result = JobSimulateResult { report, warnings: plan.warnings, image };
    serde_json::to_value(result).map_err(|e| EngineError::Other(e.to_string()))
}

fn job_export_scene(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.job.export_scene";
    let dir = p.get("path").and_then(Value::as_str).ok_or_else(|| bad(cmd, "path: the folder to write the scene into"))?.to_string();
    let (job, base) = read_job(s, p.get("job").ok_or_else(|| bad(cmd, "job: a job object or the path of a job file"))?)?;
    let bodies = job.parts.iter().map(|part| part_body(s, base.as_deref(), part)).collect::<Result<Vec<_>>>()?;
    let plan = solidcraft_cam::plan::plan_job(&job, &bodies).map_err(|e| EngineError::Other(e.to_string()))?;
    let out = solidcraft_cam::scene::export(&job, &bodies, &plan).map_err(|e| EngineError::Other(e.to_string()))?;
    let fail = |f: &Path, e: String| EngineError::Other(format!("{}: {e}", f.display()));
    let mut files = vec!["scene.json".to_string()];
    files.extend(out.files.iter().map(|(name, _)| name.clone()));
    if dir.to_ascii_lowercase().ends_with(".zip") {
        let bytes = solidcraft_cam::scene::zip(&out).map_err(|e| EngineError::Other(e.to_string()))?;
        solidcraft_io::vfs::write(&dir, &bytes).map_err(|e| fail(Path::new(&dir), e.to_string()))?;
    } else {
        let folder = Path::new(&dir);
        solidcraft_io::vfs::create_dir_all(&folder.join("parts").to_string_lossy()).map_err(|e| fail(folder, e.to_string()))?;
        let json_path = folder.join("scene.json");
        solidcraft_io::vfs::write(&json_path.to_string_lossy(), out.json.as_bytes()).map_err(|e| fail(&json_path, e.to_string()))?;
        for (name, bytes) in &out.files {
            let f = folder.join(name);
            solidcraft_io::vfs::write(&f.to_string_lossy(), bytes).map_err(|e| fail(&f, e.to_string()))?;
        }
    }
    let result = JobExportSceneResult { path: dir, files, scene: out.doc, warnings: plan.warnings };
    serde_json::to_value(result).map_err(|e| EngineError::Other(e.to_string()))
}

/// The job from the parameter, and the folder its parts' designs are relative to.
pub(crate) fn read_job(s: &Session, v: &Value) -> Result<(Job, Option<PathBuf>)> {
    let cmd = "cam.job";
    let (job, base): (Job, Option<PathBuf>) = match v {
        Value::String(path) => {
            let bytes = solidcraft_io::vfs::read(path, MAX_JOB_BYTES).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
            let text = String::from_utf8(bytes).map_err(|_| EngineError::Other(format!("{path}: not UTF-8 text")))?;
            let job = serde_json::from_str(&text).map_err(|e| EngineError::Other(format!("{path}: job: {e}")))?;
            (job, Path::new(path).parent().map(Path::to_path_buf))
        }
        Value::Object(_) => {
            let job = serde_json::from_value(v.clone()).map_err(|e| bad(cmd, format!("job: {e}")))?;
            (job, s.path.as_deref().and_then(|p| Path::new(p).parent()).map(Path::to_path_buf))
        }
        _ => return Err(bad(cmd, "job: a job object or the path of a job file")),
    };
    // its tool library, if it names one, relative to the job (or the open design)
    let load = |lib: &str| -> solidcraft_cam::Result<String> {
        let path = match &base {
            Some(dir) if Path::new(lib).is_relative() => dir.join(lib),
            _ => PathBuf::from(lib),
        };
        let shown = path.display().to_string();
        let bytes = solidcraft_io::vfs::read(&shown, MAX_JOB_BYTES).map_err(|e| solidcraft_cam::CamError::Invalid(format!("{shown}: {e}")))?;
        String::from_utf8(bytes).map_err(|_| solidcraft_cam::CamError::Invalid(format!("{shown}: not UTF-8 text")))
    };
    let job = job.with_library(&load).map_err(|e| match v {
        Value::String(path) => EngineError::Other(format!("{path}: {e}")),
        _ => bad(cmd, e.to_string()),
    })?;
    Ok((job, base))
}

/// The body a part cuts: from the open design, a design file or a STEP file.
pub(crate) fn part_body(s: &Session, base: Option<&Path>, part: &PartRef) -> Result<solidcraft_kernel::Body> {
    let label = part.label();
    if part.design.is_empty() {
        let state = s.world_state();
        return state
            .body(&part.body)
            .map(|b| b.body.clone())
            .ok_or_else(|| EngineError::Other(format!("{label}: the open design has no body named `{}`", part.body)));
    }
    let path = match base {
        Some(dir) if Path::new(&part.design).is_relative() => dir.join(&part.design),
        _ => PathBuf::from(&part.design),
    };
    let shown = path.display().to_string();
    let fail = |e: String| EngineError::Other(format!("{label} ({shown}): {e}"));
    let lower = shown.to_ascii_lowercase();
    if lower.ends_with(".step") || lower.ends_with(".stp") {
        let bytes = solidcraft_io::vfs::read(&shown, solidcraft_io::limits::max_step_bytes()).map_err(|e| fail(e.to_string()))?;
        let read = solidcraft_kernel::step_import(&String::from_utf8_lossy(&bytes)).map_err(|e| fail(e.to_string()))?;
        let pick = read.bodies.iter().find(|b| b.name == part.body).or_else(|| (read.bodies.len() == 1).then(|| read.bodies.first()).flatten());
        return pick.map(|b| b.body.clone()).ok_or_else(|| fail(format!("no solid named `{}`", part.body)));
    }
    let bytes = solidcraft_io::vfs::read_design_path(&shown, s.path.as_deref(), solidcraft_io::limits::max_design_bytes())
        .map_err(|e| fail(e.to_string()))?;
    let (doc, saved) = if solidcraft_io::is_design_container(&bytes) {
        let mut reader = solidcraft_io::DesignContainerReader::new(std::io::Cursor::new(&bytes)).map_err(|e| fail(e.to_string()))?;
        let saved = reader.geometry();
        (reader.read_document().map_err(|e| fail(e.to_string()))?, saved)
    } else {
        (solidcraft_io::read_design(&bytes).map_err(|e| fail(e.to_string()))?, None)
    };
    let design = Session::opened_at(doc, saved.as_ref(), Some(&shown));
    if let Some(e) = design.model.results.iter().find_map(|r| r.error.clone()) {
        return Err(fail(format!("the design has a feature that fails ({e})")));
    }
    let state = design.world_state();
    state.body(&part.body).map(|b| b.body.clone()).ok_or_else(|| fail(format!("no body named `{}`", part.body)))
}

#[cfg(test)]
#[path = "cam_tests.rs"]
mod tests;
