//! The twin's turning, simulated in the half section: side 1 from the bar, then side 2 on what
//! side 1 cut off (turned end for end), each checked as the lathe's simulation checks it
//! ([`crate::lathe::sim`]); side 2's comparison with the part is the finished part's.

use schemars::JsonSchema;
use serde::Serialize;

use super::job::CtxJob;
use super::plan::CtxPlan;
use crate::lathe::sim::{LatheSimReport, simulate_from};
use crate::region::{Area, Ring};
use crate::{CamError, Result};

#[derive(Clone, Debug, Serialize, JsonSchema)]
pub struct CtxSimReport {
    pub side1: LatheSimReport,
    /// Its `left` and `gouges` are the finished part's.
    pub side2: LatheSimReport,
}

/// `a` turned end for end about work Z `-l / 2`: z → −l − z.
fn turned(a: &Area, l: f64) -> Result<Area> {
    let rings: Vec<Ring> = a.rings().map(|r| r.iter().map(|p| [-l - p[0], p[1]]).collect()).collect();
    Area::from_rings(&rings)
}

/// Simulate both sides; the half sections left on side 1 (the bar and the part) and side 2.
pub fn simulate(job: &CtxJob, plan: &CtxPlan) -> Result<(CtxSimReport, Area, Area)> {
    let job1 = job.side_job(1, 0.0)?;
    let (r1, left1) = simulate_from(&job1, &plan.side1, None)?;
    let parting = plan.parting().ok_or_else(|| CamError::Invalid("side 1 doesn't part off".into()))?;
    let w = job1.tool(parting.tool).map_or(0.0, |t| t.blade_width());
    let cut = parting.moves.iter().map(|m| m.to()[1]).fold(f64::INFINITY, f64::min) + w;
    let su = &plan.side1.setup;
    let piece = left1.intersection(&Area::rect(cut, -1.0, su.front + 1.0, su.radius + 1.0)?)?;
    let l = plan.side1.profile.length;
    let job2 = job.side_job(2, plan.grip)?;
    let (r2, left2) = simulate_from(&job2, &plan.side2, Some(turned(&piece, l)?))?;
    Ok((CtxSimReport { side1: r1, side2: r2 }, left1, left2))
}
