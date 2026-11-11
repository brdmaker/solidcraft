//! Contracts and result types for the commands in `cmd/cam.rs`.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// Which body to read machining features from.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct PartInspectParams {
    /// The body's name. It is machined from the top (+Z) on a 3-axis router.
    pub body: String,
}

/// A job to nest: the job itself, or the path of its JSON file.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct JobNestParams {
    /// The job (stock, nest, tools, parts) as an object, or the path of a job file. A part's
    /// `design` is relative to the job file (an object: to the open design's folder); an empty
    /// `design` takes the body from the open design.
    pub job: serde_json::Value,
}

/// One part of a nested job, as read from its design.
#[derive(Debug, Serialize, JsonSchema)]
pub struct NestedPart {
    pub label: String,
    /// Which of the part's own axes is up on the machine.
    pub up: solidcraft_cam::job::Up,
    pub design: String,
    pub body: String,
    pub quantity: u32,
    /// Height of the part, mm.
    pub height: f64,
    /// Sheet thickness left above the part when its bottom sits on the sheet's: faced off first.
    pub face_off: f64,
    /// The part's box seen from above, in its own coordinates.
    pub bounds: [[f64; 2]; 2],
    pub outline_area: f64,
    /// Faces it cannot machine (see `cam.part.inspect`).
    pub unsupported: usize,
}

/// What `cam.job.nest` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct JobNestResult {
    pub sheet: [f64; 2],
    pub thickness: f64,
    pub parts: Vec<NestedPart>,
    pub nesting: solidcraft_cam::nest::Nesting,
}

/// A job to plan and export as models for a machine's own viewer.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct JobExportSceneParams {
    /// The job (as for `cam.job.nest`): an object or the path of a job file.
    pub job: serde_json::Value,
    /// Where to write `scene.json`, `stock.stl`, `placed.stl` and `parts/*.stl`: a `.zip` file
    /// holding them all, or a folder.
    pub path: String,
}

/// What `cam.job.export_scene` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct JobExportSceneResult {
    /// The archive or folder written.
    pub path: String,
    /// The files written, relative to the folder or as named in the archive.
    pub files: Vec<String>,
    /// What `scene.json` says.
    pub scene: solidcraft_cam::scene::SceneDoc,
    /// The plan's warnings (faces not machined, copies not placed...).
    pub warnings: Vec<String>,
}

/// A job to plan and write as a LinuxCNC program.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct JobGcodeParams {
    /// The job (as for `cam.job.nest`): an object or the path of a job file.
    pub job: serde_json::Value,
    /// The program file to write (`.ngc`).
    pub path: String,
}

/// What `cam.job.gcode` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct JobGcodeResult {
    pub path: String,
    /// Lines in the program.
    pub lines: usize,
    /// The tools it loads, in order of first use.
    pub tools: Vec<u32>,
    /// Operations in the order they run.
    pub operations: Vec<String>,
    /// The plan's warnings.
    pub warnings: Vec<String>,
}

/// A job to plan and simulate.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct JobSimulateParams {
    /// The job (as for `cam.job.nest`): an object or the path of a job file.
    pub job: serde_json::Value,
    /// A cell of the height map, mm (default 0.25).
    #[serde(default)]
    pub cell: Option<f64>,
    /// A PNG to draw the machined sheet in, with the report's spots marked.
    #[serde(default)]
    pub image: Option<String>,
}

/// What `cam.job.simulate` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct JobSimulateResult {
    pub report: solidcraft_cam::sim::SimReport,
    /// The plan's warnings.
    pub warnings: Vec<String>,
    /// The picture written, if one was asked for.
    pub image: Option<String>,
}

/// A lathe job to plan and simulate.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct LathePlanParams {
    /// The lathe job as an object, or the path of a job file: {machine?, stock: {diameter,
    /// material?, length?, in_chuck?, face_allowance?, parting_allowance?, chuck_clearance?},
    /// part: {body, design?, name?, front?, threads?}, library?, tools?, strategy?, post?}. The
    /// part's `design` is relative to the job file (an object: to the open design's folder); an
    /// empty `design` takes the body from the open design.
    pub job: serde_json::Value,
    /// A PNG to draw the simulated half section in.
    #[serde(default)]
    pub image: Option<String>,
}

/// What `cam.lathe.plan` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct LathePlanResult {
    /// The machine, the bar as set up, the part's outline and bore, the tools in the order the
    /// program loads them, the operations, the threads, the warnings and the simulation's report:
    /// what an exported scene's `scene.json` says.
    pub scene: solidcraft_cam::lathe::scene::LatheSceneDoc,
    /// The picture written, if one was asked for.
    pub image: Option<String>,
}

/// A lathe job to write out: its program, or its scene for a machine's viewer.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct LatheWriteParams {
    /// The lathe job (as for `cam.lathe.plan`): an object or the path of a job file.
    pub job: serde_json::Value,
    /// The file to write: the program (`.ngc`), or the scene (a `.zip`, or a folder).
    pub path: String,
}

/// What `cam.lathe.gcode` and `cam.lathe.export_scene` return.
#[derive(Debug, Serialize, JsonSchema)]
pub struct LatheWriteResult {
    /// The file or folder written.
    pub path: String,
    /// The files written: the program, or the scene's files as named in the archive or folder.
    pub files: Vec<String>,
    /// Lines in the program.
    pub lines: usize,
    /// The tools the program loads, in order of first use.
    pub tools: Vec<u32>,
    /// Operations in the order they run.
    pub operations: Vec<String>,
    /// The plan's warnings.
    pub warnings: Vec<String>,
}

/// A turn-mill job for the twin-spindle lathe to plan and check.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CtxPlanParams {
    /// The turn-mill job as an object, or the path of a job file: {machine?, stock: {diameter,
    /// material?, stick_out?, face_allowance?, parting_allowance?, back_allowance?,
    /// chuck_clearance?}, part: {body, design?, name?, front?, threads?}, library?, tools?,
    /// live_tools?, transfer?, strategy?, post?}. The part's `design` is relative to the job file
    /// (an object: to the open design's folder); an empty `design` takes the body from the open
    /// design.
    pub job: serde_json::Value,
}

/// One side's work in a turn-mill plan.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CtxSide {
    /// Operations in the order the side's program runs them (turning, then the live tools; side
    /// 1's parting last).
    pub operations: Vec<String>,
    /// The tools the program loads, in order of first use.
    pub tools: Vec<u32>,
}

/// What `cam.ctx.plan` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CtxPlanResult {
    /// How much of the part's front chuck 2 grips, mm.
    pub grip: f64,
    pub side1: CtxSide,
    pub side2: CtxSide,
    /// Both sides' turning simulated in the half section (side 2's on what side 1 cut off), and
    /// the live tools' points found inside the finished part.
    pub simulation: solidcraft_cam::ctx::sim::CtxSimReport,
    pub warnings: Vec<String>,
}

/// A turn-mill job to write out as its two programs.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CtxWriteParams {
    /// The turn-mill job (as for `cam.ctx.plan`): an object or the path of a job file.
    pub job: serde_json::Value,
    /// The programs' path: `<name>.ngc` writes `<name>_1.ngc` (side 1) and `<name>_2.ngc` (side
    /// 2) beside it.
    pub path: String,
}

/// What `cam.ctx.gcode` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CtxWriteResult {
    /// The two programs written, side 1's then side 2's.
    pub files: Vec<String>,
    /// Lines in each.
    pub lines: Vec<usize>,
    pub side1: CtxSide,
    pub side2: CtxSide,
    pub warnings: Vec<String>,
}
