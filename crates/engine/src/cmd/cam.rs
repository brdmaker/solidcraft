//! CAM for 3-axis routers: machining features, toolpaths and G-code (`solidcraft_cam`).

use serde_json::Value;

use super::CommandSpec;
use crate::params::bad;
use crate::schema::cam::PartInspectParams;
use crate::schema::{TypedContract, output_schema_for};
use crate::{EngineError, Result, Session};

const PART_INSPECT_CONTRACT: TypedContract<PartInspectParams> = TypedContract::new();

pub static COMMANDS: &[CommandSpec] =
    &[CommandSpec::new("cam.part.inspect", "Inspect Machining Features", crate::checked!("cam.part.inspect", part_inspect))
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
        )];

fn part_inspect(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.part.inspect";
    let name = p.get("body").and_then(Value::as_str).ok_or_else(|| bad(cmd, "body: the name of a body"))?;
    let state = s.model.state();
    let body = state.body(name).ok_or_else(|| bad(cmd, format!("no body named `{name}`")))?;
    let features = solidcraft_cam::part::read_part(&body.body).map_err(|e| EngineError::Other(e.to_string()))?;
    serde_json::to_value(features).map_err(|e| EngineError::Other(e.to_string()))
}

#[cfg(test)]
#[path = "cam_tests.rs"]
mod tests;
