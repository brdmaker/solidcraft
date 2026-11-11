//! CAM for 3-axis routers: machining features, toolpaths and G-code (`solidcraft_cam`).

use std::path::{Path, PathBuf};

use serde_json::Value;
use solidcraft_cam::job::{Job, PartRef};
use solidcraft_cam::nest::{NestItem, nest};
use solidcraft_cam::part::{PartFeatures, oriented};

use super::CommandSpec;
use crate::params::bad;
use crate::schema::cam::{
    CtxPlanParams, CtxPlanResult, CtxSide, CtxWriteParams, CtxWriteResult, JobExportSceneParams, JobExportSceneResult, JobGcodeParams,
    JobGcodeResult, JobNestParams, JobNestResult, JobSimulateParams, JobSimulateResult, LathePlanParams, LathePlanResult, LatheWriteParams,
    LatheWriteResult, NestedPart, PartInspectParams,
};
use crate::schema::{TypedContract, output_schema_for};
use crate::{EngineError, Result, Session};

const PART_INSPECT_CONTRACT: TypedContract<PartInspectParams> = TypedContract::new();
const JOB_NEST_CONTRACT: TypedContract<JobNestParams> = TypedContract::new();
const JOB_EXPORT_SCENE_CONTRACT: TypedContract<JobExportSceneParams> = TypedContract::new();
const JOB_GCODE_CONTRACT: TypedContract<JobGcodeParams> = TypedContract::new();
const JOB_SIMULATE_CONTRACT: TypedContract<JobSimulateParams> = TypedContract::new();
const LATHE_PLAN_CONTRACT: TypedContract<LathePlanParams> = TypedContract::new();
const LATHE_WRITE_CONTRACT: TypedContract<LatheWriteParams> = TypedContract::new();
const CTX_PLAN_CONTRACT: TypedContract<CtxPlanParams> = TypedContract::new();
const CTX_WRITE_CONTRACT: TypedContract<CtxWriteParams> = TypedContract::new();

/// Largest job file read (bytes).
const MAX_JOB_BYTES: usize = 4 << 20;

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.ctx.gcode", "Write Turn-Mill G-code", crate::checked!("cam.ctx.gcode", ctx_gcode))
        .icon("export")
        .noundo()
        .contract(&CTX_WRITE_CONTRACT)
        .output(output_schema_for::<CtxWriteResult>)
        .params(
            "job: as for cam.ctx.plan; path: <name>.ngc → plans the job and writes its two LinuxCNC programs, <name>_1.ngc for \
             side 1 and <name>_2.ngc for side 2, to the twin's programming manual: o<twin_init>, G18 G7 G21 G90 G40 G80 G95, \
             T<n> M6 G43 on turret stations, G96 S D; live tools between M14 M16 M3 $1 and M17 M15 (G17 or G8 G19 drilling \
             cycles at C, polar interpolation G12.1 for face milling); side 1 meets side 2 at sync point 3, grips with \
             o<twin_grip>, parts off at G97 under the transfer's speed and calls o<twin_cut_done>; side 2 catches with \
             o<twin_catch>. Returns the files, their line counts, each side's operations and tools, and the warnings",
        ),
    CommandSpec::new("cam.ctx.plan", "Simulate Turn-Mill Job", crate::checked!("cam.ctx.plan", ctx_plan))
        .icon("play")
        .noundo()
        .contract(&CTX_PLAN_CONTRACT)
        .output(output_schema_for::<CtxPlanResult>)
        .params(
            "job: a turn-mill job object or the path of a job file, for a twin-spindle lathe with live tools, bar fed through \
             spindle 1: {stock: {diameter, material? (aluminium | brass | steel), stick_out?, face_allowance? (0.5), \
             parting_allowance? (4), back_allowance? (0.3, faced off on side 2), chuck_clearance? (2)}, part: {body, design?, name?, \
             front? (+z | -z: the end finished on side 1), threads?}, library? (default the twin simulation's: turning tools and \
             live tools), tools?, live_tools? [{number, kind (flat | drill), mount (axial | radial), diameter, flute_length, rpm, \
             feed, plunge, flutes?, stepdown?, stepover?, peck?, reach?, holder_diameter?}], machine? {name?, max_rpm?, \
             live_max_rpm?, sync_max_rpm?, stations?, z_travel?, x_min?, polar_c_rpm?}, transfer? {grip?, pickoff_z?}, strategy?, \
             post? {decimals?, blend_tolerance?, coolant?, work_z_1?, work_z_2?, transfer_wait?}} → side 1 turns the part from \
             the bar (facing, drilling, roughing, boring, finishing, grooving, threading), machines its front and radial features \
             with live tools (axial holes off the axis, cross holes, keyways, faces milled in polar interpolation) and parts it \
             off while spindle 2 grips it and follows; side 2 faces the back to length, bores it and machines the back's \
             features. Returns chuck 2's grip, each side's operations and tools, the simulation (both sides' turning in the \
             half section; live tool points inside the part) and the warnings. Read-only",
        ),
    CommandSpec::new("cam.lathe.plan", "Simulate Lathe Job", crate::checked!("cam.lathe.plan", lathe_plan))
        .icon("play")
        .noundo()
        .contract(&LATHE_PLAN_CONTRACT)
        .output(output_schema_for::<LathePlanResult>)
        .params(
            "job: a lathe job object or the path of a job file: {stock: {diameter, material? (aluminium | brass | steel), length? \
             (default: what the part needs and the chuck holds), in_chuck? (20), face_allowance? (0.5), parting_allowance? (3), \
             chuck_clearance? (2)}, part: {body, design? (.sc or STEP, relative to the job file; empty: the open design), name?, \
             front? (+z | -z: the end of its Z axis away from the chuck), threads? [{designation (M12, M12x1.25), z_from, z_to}]}, \
             library? (a lathe tool library file; default the Emco Compact 5's six-station turret), tools?, machine?, strategy? \
             {rough_tool?, finish_tool?, leave_radial? (0.2), leave_axial? (0.1), clearance? (1), change_clearance? (10)}, post? \
             {coolant?, decimals?, blend_tolerance?}}; image? (a .png path) → reads the part's profile about its Z axis (from its exact \
             faces where all are of revolution about Z), plans facing, drilling, roughing, boring, finishing, grooving, threading and \
             parting, and simulates them in the bar's half section: the scene (machine, bar as set up, part outline and bore, tools \
             in order with their data, operations, threads, warnings, simulation: gouges, material left, rapids through material, \
             moves at the jaws). The image draws the half section with the paths. Read-only but for the image",
        ),
    CommandSpec::new("cam.lathe.gcode", "Write Lathe G-code", crate::checked!("cam.lathe.gcode", lathe_gcode))
        .icon("export")
        .noundo()
        .contract(&LATHE_WRITE_CONTRACT)
        .output(output_schema_for::<LatheWriteResult>)
        .params(
            "job: as for cam.lathe.plan; path: the .ngc file → plans the job and writes its LinuxCNC lathe program: G18 G7 (X a \
             diameter) G21 G90 G40 G80 G95 (feed per revolution), G54 with Z0 the finished front face, each tool T<n> M6 G43 at the \
             change position, G96 S D (constant surface speed under the top speed) or G97 S for drills and threads, G0/G1 moves of \
             the tools' touch-off points, threads in G33 passes, M30. Returns the path, line count, tools, operations and warnings",
        ),
    CommandSpec::new("cam.lathe.export_scene", "Export Lathe Job Scene", crate::checked!("cam.lathe.export_scene", lathe_export_scene))
        .icon("export")
        .noundo()
        .contract(&LATHE_WRITE_CONTRACT)
        .output(output_schema_for::<LatheWriteResult>)
        .params(
            "job: as for cam.lathe.plan; path: a .zip file or a folder → plans and simulates the job and writes, for a machine's \
             own viewer, in work coordinates (Z0 the finished front face, the part toward -Z, X and Y radial): stock.stl (the bar as \
             set up), part.stl (the part where it is turned), turned.stl (the bar as the simulation leaves it), job.ngc and \
             scene.json (as cam.lathe.plan returns it). Returns the files written, the tools, operations and warnings",
        ),
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
    let program = solidcraft_cam::post::linuxcnc(&job, &plan.ops, &title, &plan.warnings).map_err(|e| EngineError::Other(e.to_string()))?;
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

/// A lathe job from the parameter, and the folder its part's design is relative to.
fn read_lathe_job(s: &Session, v: &Value) -> Result<(solidcraft_cam::lathe::job::LatheJob, Option<PathBuf>)> {
    let cmd = "cam.lathe";
    let (text, base) = match v {
        Value::String(path) => {
            let bytes = solidcraft_io::vfs::read(path, MAX_JOB_BYTES).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
            let text = String::from_utf8(bytes).map_err(|_| EngineError::Other(format!("{path}: not UTF-8 text")))?;
            (text, Path::new(path).parent().map(Path::to_path_buf))
        }
        Value::Object(_) => (v.to_string(), s.path.as_deref().and_then(|p| Path::new(p).parent()).map(Path::to_path_buf)),
        _ => return Err(bad(cmd, "job: a lathe job object or the path of a job file")),
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
    let job = solidcraft_cam::lathe::job::LatheJob::from_json_with(&text, &load).map_err(|e| match v {
        Value::String(path) => EngineError::Other(format!("{path}: {e}")),
        _ => bad(cmd, e.to_string()),
    })?;
    Ok((job, base))
}

/// A lathe job read, its part's body, its plan and simulation.
struct LatheRun {
    job: solidcraft_cam::lathe::job::LatheJob,
    body: solidcraft_kernel::Body,
    plan: solidcraft_cam::lathe::plan::LathePlan,
    report: solidcraft_cam::lathe::sim::LatheSimReport,
    left: solidcraft_cam::region::Area,
}

fn lathe_run(s: &Session, cmd: &str, p: &Value) -> Result<LatheRun> {
    use solidcraft_cam::lathe::{plan, profile, sim};
    let (job, base) = read_lathe_job(s, p.get("job").ok_or_else(|| bad(cmd, "job: a lathe job object or the path of a job file"))?)?;
    let part = PartRef {
        name: job.part.name.clone(),
        design: job.part.design.clone(),
        body: job.part.body.clone(),
        quantity: 1,
        orient: Default::default(),
    };
    let body = part_body(s, base.as_deref(), &part)?;
    let other = |e: solidcraft_cam::CamError| EngineError::Other(format!("{}: {e}", part.label()));
    let prof = profile::read(&body, job.part.front).map_err(other)?;
    let plan = plan::plan(&job, prof).map_err(other)?;
    let (report, left) = sim::simulate(&job, &plan).map_err(other)?;
    Ok(LatheRun { job, body, plan, report, left })
}

fn lathe_plan(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.lathe.plan";
    let image = p.get("image").and_then(Value::as_str).map(str::to_string);
    let run = lathe_run(s, cmd, p)?;
    if let Some(path) = &image {
        let png = solidcraft_cam::lathe::sim::png(&run.plan, &run.left, 1600).map_err(|e| EngineError::Other(e.to_string()))?;
        solidcraft_io::vfs::write(path, &png).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
    }
    let result = LathePlanResult { scene: solidcraft_cam::lathe::scene::describe(&run.job, &run.plan, &run.report), image };
    serde_json::to_value(result).map_err(|e| EngineError::Other(e.to_string()))
}

/// What a lathe write returns: the run's tools, operations and warnings.
fn lathe_written(run: &LatheRun, path: String, files: Vec<String>, lines: usize) -> Result<Value> {
    let mut tools: Vec<u32> = Vec::new();
    for op in &run.plan.ops {
        if !tools.contains(&op.tool) {
            tools.push(op.tool);
        }
    }
    let result = LatheWriteResult {
        path,
        files,
        lines,
        tools,
        operations: run.plan.ops.iter().map(|o| o.name.clone()).collect(),
        warnings: run.plan.warnings.clone(),
    };
    serde_json::to_value(result).map_err(|e| EngineError::Other(e.to_string()))
}

fn lathe_gcode(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.lathe.gcode";
    let path = p.get("path").and_then(Value::as_str).ok_or_else(|| bad(cmd, "path: the .ngc file to write"))?.to_string();
    let run = lathe_run(s, cmd, p)?;
    let title = solidcraft_cam::lathe::scene::title(&run.job);
    let program = solidcraft_cam::lathe::post::linuxcnc(&run.job, &run.plan, &title).map_err(|e| EngineError::Other(e.to_string()))?;
    solidcraft_io::vfs::write(&path, program.as_bytes()).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
    let name = Path::new(&path).file_name().map_or(path.clone(), |f| f.to_string_lossy().into_owned());
    lathe_written(&run, path, vec![name], program.lines().count())
}

fn lathe_export_scene(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.lathe.export_scene";
    let dir = p.get("path").and_then(Value::as_str).ok_or_else(|| bad(cmd, "path: a .zip file or a folder to write the scene into"))?.to_string();
    let run = lathe_run(s, cmd, p)?;
    let out = solidcraft_cam::lathe::scene::export(&run.job, &run.body, &run.plan, &run.report, &run.left)
        .map_err(|e| EngineError::Other(e.to_string()))?;
    let fail = |f: &Path, e: String| EngineError::Other(format!("{}: {e}", f.display()));
    let mut files = vec!["scene.json".to_string()];
    files.extend(out.files.iter().map(|(name, _)| name.clone()));
    if dir.to_ascii_lowercase().ends_with(".zip") {
        let bytes = out.zip().map_err(|e| EngineError::Other(e.to_string()))?;
        solidcraft_io::vfs::write(&dir, &bytes).map_err(|e| fail(Path::new(&dir), e.to_string()))?;
    } else {
        let folder = Path::new(&dir);
        solidcraft_io::vfs::create_dir_all(&folder.to_string_lossy()).map_err(|e| fail(folder, e.to_string()))?;
        let json_path = folder.join("scene.json");
        solidcraft_io::vfs::write(&json_path.to_string_lossy(), out.json.as_bytes()).map_err(|e| fail(&json_path, e.to_string()))?;
        for (name, bytes) in &out.files {
            let f = folder.join(name);
            solidcraft_io::vfs::write(&f.to_string_lossy(), bytes).map_err(|e| fail(&f, e.to_string()))?;
        }
    }
    let lines = out.files.iter().find(|(n, _)| n == "job.ngc").map_or(0, |(_, b)| b.iter().filter(|c| **c == b'\n').count());
    lathe_written(&run, dir, files, lines)
}

/// A turn-mill job read, its part's body, its plan and simulation.
struct CtxRun {
    job: solidcraft_cam::ctx::job::CtxJob,
    plan: solidcraft_cam::ctx::plan::CtxPlan,
    report: solidcraft_cam::ctx::sim::CtxSimReport,
}

fn ctx_run(s: &Session, cmd: &str, p: &Value) -> Result<CtxRun> {
    let what = "job: a turn-mill job object or the path of a job file";
    let v = p.get("job").ok_or_else(|| bad(cmd, what))?;
    let (text, base) = match v {
        Value::String(path) => {
            let bytes = solidcraft_io::vfs::read(path, MAX_JOB_BYTES).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
            let text = String::from_utf8(bytes).map_err(|_| EngineError::Other(format!("{path}: not UTF-8 text")))?;
            (text, Path::new(path).parent().map(Path::to_path_buf))
        }
        Value::Object(_) => (v.to_string(), s.path.as_deref().and_then(|p| Path::new(p).parent()).map(Path::to_path_buf)),
        _ => return Err(bad(cmd, what)),
    };
    let load = |lib: &str| -> solidcraft_cam::Result<String> {
        let path = match &base {
            Some(dir) if Path::new(lib).is_relative() => dir.join(lib),
            _ => PathBuf::from(lib),
        };
        let shown = path.display().to_string();
        let bytes = solidcraft_io::vfs::read(&shown, MAX_JOB_BYTES).map_err(|e| solidcraft_cam::CamError::Invalid(format!("{shown}: {e}")))?;
        String::from_utf8(bytes).map_err(|_| solidcraft_cam::CamError::Invalid(format!("{shown}: not UTF-8 text")))
    };
    let job = solidcraft_cam::ctx::job::CtxJob::from_json_with(&text, &load).map_err(|e| match v {
        Value::String(path) => EngineError::Other(format!("{path}: {e}")),
        _ => bad(cmd, e.to_string()),
    })?;
    let part = PartRef {
        name: job.part.name.clone(),
        design: job.part.design.clone(),
        body: job.part.body.clone(),
        quantity: 1,
        orient: Default::default(),
    };
    let body = part_body(s, base.as_deref(), &part)?;
    let other = |e: solidcraft_cam::CamError| EngineError::Other(format!("{}: {e}", part.label()));
    let plan = solidcraft_cam::ctx::plan::plan(&job, &body).map_err(other)?;
    let (report, _, _) = solidcraft_cam::ctx::sim::simulate(&job, &plan, &body).map_err(other)?;
    Ok(CtxRun { job, plan, report })
}

/// Each side's operations and tools, in program order.
fn ctx_sides(run: &CtxRun) -> (CtxSide, CtxSide) {
    use solidcraft_cam::lathe::plan::LatheOpKind;
    let side = |turn: Vec<(&str, u32)>, live: &[solidcraft_cam::ctx::live::LiveOp], last: Vec<(&str, u32)>| {
        let all: Vec<(String, u32)> = turn
            .into_iter()
            .map(|(n, t)| (n.to_string(), t))
            .chain(live.iter().map(|o| (o.name.clone(), o.tool)))
            .chain(last.into_iter().map(|(n, t)| (n.to_string(), t)))
            .collect();
        let mut tools: Vec<u32> = Vec::new();
        for (_, t) in &all {
            if !tools.contains(t) {
                tools.push(*t);
            }
        }
        CtxSide { operations: all.into_iter().map(|(n, _)| n).collect(), tools }
    };
    let p1 = &run.plan.side1.ops;
    let one = side(
        p1.iter().filter(|o| o.kind != LatheOpKind::Parting).map(|o| (o.name.as_str(), o.tool)).collect(),
        &run.plan.live1,
        p1.iter().filter(|o| o.kind == LatheOpKind::Parting).map(|o| (o.name.as_str(), o.tool)).collect(),
    );
    let two = side(run.plan.side2.ops.iter().map(|o| (o.name.as_str(), o.tool)).collect(), &run.plan.live2, Vec::new());
    (one, two)
}

fn ctx_plan(s: &mut Session, p: &Value) -> Result<Value> {
    let run = ctx_run(s, "cam.ctx.plan", p)?;
    let (side1, side2) = ctx_sides(&run);
    let result = CtxPlanResult { grip: run.plan.grip, side1, side2, simulation: run.report.clone(), warnings: run.plan.warnings.clone() };
    serde_json::to_value(result).map_err(|e| EngineError::Other(e.to_string()))
}

fn ctx_gcode(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.ctx.gcode";
    let path = p.get("path").and_then(Value::as_str).ok_or_else(|| bad(cmd, "path: <name>.ngc: the programs' path"))?.to_string();
    let run = ctx_run(s, cmd, p)?;
    let stem = path.strip_suffix(".ngc").unwrap_or(&path).to_string();
    let title = if run.job.part.name.is_empty() { run.job.part.body.clone() } else { run.job.part.name.clone() };
    let programs = [
        solidcraft_cam::ctx::post::side1(&run.job, &run.plan, &title).map_err(|e| EngineError::Other(e.to_string()))?,
        solidcraft_cam::ctx::post::side2(&run.job, &run.plan, &title).map_err(|e| EngineError::Other(e.to_string()))?,
    ];
    let mut files = Vec::new();
    let mut lines = Vec::new();
    for (i, program) in programs.iter().enumerate() {
        let f = format!("{stem}_{}.ngc", i + 1);
        solidcraft_io::vfs::write(&f, program.as_bytes()).map_err(|e| EngineError::Other(format!("{f}: {e}")))?;
        files.push(f);
        lines.push(program.lines().count());
    }
    let (side1, side2) = ctx_sides(&run);
    let result = CtxWriteResult { files, lines, side1, side2, warnings: run.plan.warnings.clone() };
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
        let state = s.world_state()?;
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
    let state = design.world_state()?;
    state.body(&part.body).map(|b| b.body.clone()).ok_or_else(|| fail(format!("no body named `{}`", part.body)))
}

#[cfg(test)]
#[path = "cam_tests.rs"]
mod tests;
