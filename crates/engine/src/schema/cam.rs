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
