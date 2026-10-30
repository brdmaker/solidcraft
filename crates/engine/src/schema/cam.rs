//! Contracts and result types for the commands in `cmd/cam.rs`.

use schemars::JsonSchema;
use serde::Deserialize;

/// Which body to read machining features from.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct PartInspectParams {
    /// The body's name. It is machined from the top (+Z) on a 3-axis router.
    pub body: String,
}
