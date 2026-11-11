//! A turn-mill job for a twin-spindle lathe, read from JSON: the machine, the bar, the part, the
//! turret's tools (turning tools and live tools) and how the part goes from spindle 1 to
//! spindle 2.
//!
//! Coordinates as on a lathe ([`crate::lathe::job`]): each side's program has Z0 on the face it
//! finishes, the part at negative Z toward its chuck, X a diameter. In the part's own frame z
//! runs from its back (the end cut off the bar, finished in spindle 2) to its front (finished in
//! spindle 1, away from chuck 1).

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::lathe::job::{LatheJob, LatheMachine, LathePost, LatheStrategy, LatheTool, SpindleDirection, ToolSide, TurnedPart};
use crate::{CamError, Result};

/// The built-in tool library for the twin's simulation (`brdmot`'s `linuxcnc/ctxtwin/sim/tool.tbl`).
pub const CTX_TWIN_SIM_LIBRARY: &str = include_str!("../../library/ctx-twin-sim.json");

/// Most live tools a job or library may list.
const MAX_LIVE: usize = 32;

#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CtxJob {
    /// The format's version: 1.
    #[serde(default = "one")]
    pub version: u32,
    #[serde(default)]
    pub machine: CtxMachine,
    pub stock: CtxBar,
    pub part: TurnedPart,
    /// A tool library file (relative to the job) filling in the tools the job doesn't list; empty:
    /// the twin simulation's; `none`: only the job's own.
    #[serde(default)]
    pub library: String,
    /// The turret's tools for turning, drilling on the axis, grooving and parting (the same
    /// stations on both sides).
    #[serde(default)]
    pub tools: Vec<LatheTool>,
    /// The live tools (driven in the turret's cutting station).
    #[serde(default)]
    pub live_tools: Vec<LiveTool>,
    /// How spindle 2 takes the part.
    #[serde(default)]
    pub transfer: Transfer,
    #[serde(default)]
    pub strategy: LatheStrategy,
    #[serde(default)]
    pub post: CtxPost,
}

fn one() -> u32 {
    1
}

/// The twin-spindle lathe.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields, default)]
pub struct CtxMachine {
    pub name: String,
    /// Highest main spindle speed, rpm (the G96 limit), both sides.
    pub max_rpm: f64,
    /// Highest live tool speed, rpm.
    pub live_max_rpm: f64,
    /// Highest speed while spindle 2 follows spindle 1 (parting off with both chucks holding the
    /// part), rpm.
    pub sync_max_rpm: f64,
    /// The turret's stations: T`n` turns it to station `n` mod this.
    pub stations: u32,
    /// Z travel of side 1, mm: the bar's free length is checked against it.
    pub z_travel: f64,
}

impl Default for CtxMachine {
    fn default() -> Self {
        CtxMachine { name: "CTX twin".into(), max_rpm: 4000.0, live_max_rpm: 3000.0, sync_max_rpm: 2000.0, stations: 12, z_travel: 300.0 }
    }
}

/// The bar, fed through spindle 1.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CtxBar {
    /// Diameter, mm.
    pub diameter: f64,
    /// How far it stands out of chuck 1, mm (default: the part, the facing and parting
    /// allowances and the clearance to the jaws, rounded up to whole millimetres).
    #[serde(default)]
    pub stick_out: Option<f64>,
    /// Faced off the front, mm (default 0.5).
    #[serde(default = "face_allowance")]
    pub face_allowance: f64,
    /// Turned behind the part to part it off, mm (default 4).
    #[serde(default = "parting_allowance")]
    pub parting_allowance: f64,
    /// Left on the back by the parting cut, faced off in spindle 2, mm (default 0.3).
    #[serde(default = "back_allowance")]
    pub back_allowance: f64,
    /// The tools keep this far from the jaws, mm (default 2).
    #[serde(default = "chuck_clearance")]
    pub chuck_clearance: f64,
    /// `aluminium`, `brass` or `steel`: picks each tool's surface speed.
    #[serde(default = "steel")]
    pub material: String,
}

fn face_allowance() -> f64 {
    0.5
}
fn parting_allowance() -> f64 {
    4.0
}
fn back_allowance() -> f64 {
    0.3
}
fn chuck_clearance() -> f64 {
    2.0
}
fn steel() -> String {
    "steel".into()
}

/// How spindle 2 takes the part off the bar.
#[derive(Clone, Debug, Default, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields, default)]
pub struct Transfer {
    /// How much of the part's front chuck 2 holds, mm (default: up to 20, clear of the parting
    /// blade's holder, at least 5).
    pub grip: Option<f64>,
    /// Where Z2 grips it, in side 2's machine Z (`o<twin_catch>`'s argument; default the
    /// machine's `[TWIN]Z_PICKOFF`).
    pub pickoff_z: Option<f64>,
}

/// The programs' settings.
#[derive(Clone, Debug, Default, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields, default)]
pub struct CtxPost {
    #[serde(flatten)]
    pub lathe: LathePost,
    /// Side 1's work offset: the program sets G54's Z to it (`G10 L2 P1 Z…`), machine Z of the
    /// finished front face; none: set at the machine.
    pub work_z_1: Option<f64>,
    /// Side 2's: machine Z of the finished back face with the part in chuck 2.
    pub work_z_2: Option<f64>,
    /// How long side 2 waits for side 1 to come to the transfer, s (default: twice side 1's
    /// estimated time before it, plus 2 minutes, at least 10 minutes).
    pub transfer_wait: Option<f64>,
}

/// How a live tool is mounted in the turret.
#[derive(Clone, Copy, Debug, Deserialize, Serialize, JsonSchema, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Mount {
    /// Along the spindle's axis: drills and mills the faces.
    Axial,
    /// Square to it: cross holes, flats and slots on the diameter.
    Radial,
}

/// A live tool: a drill or an end mill turning in the turret's cutting station.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LiveTool {
    pub number: u32,
    #[serde(default)]
    pub name: String,
    /// `flat` (an end mill) or `drill`.
    pub kind: crate::job::ToolKind,
    pub mount: Mount,
    /// Cutting diameter, mm.
    pub diameter: f64,
    /// Length of the cutting part, mm.
    pub flute_length: f64,
    #[serde(default = "two")]
    pub flutes: u32,
    /// Drill point angle, degrees (default 118).
    #[serde(default)]
    pub tip_angle_deg: Option<f64>,
    /// Speed, rpm.
    pub rpm: f64,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge feed, mm/min.
    pub plunge: f64,
    /// Depth of one pass, mm (default the diameter).
    #[serde(default)]
    pub stepdown: Option<f64>,
    /// Side step of clearing passes as a fraction of the diameter (default 0.4).
    #[serde(default)]
    pub stepover: Option<f64>,
    /// Peck depth for drilling, mm (none: in one go).
    #[serde(default)]
    pub peck: Option<f64>,
    /// How far the tool stands out of its driven holder, mm (default the flute length + 5).
    #[serde(default)]
    pub reach: Option<f64>,
    /// The driven holder's diameter, mm (default 3 × the tool's, at least 30).
    #[serde(default)]
    pub holder_diameter: Option<f64>,
}

fn two() -> u32 {
    2
}

impl LiveTool {
    /// As a router tool, for the milling planner.
    pub fn as_tool(&self) -> crate::job::Tool {
        crate::job::Tool {
            number: self.number,
            name: self.name.clone(),
            kind: self.kind,
            diameter: self.diameter,
            flute_length: self.flute_length,
            flutes: self.flutes,
            tip_angle_deg: self.tip_angle_deg,
            rpm: self.rpm,
            feed: self.feed,
            chip_load: None,
            plunge: self.plunge,
            stepdown: self.stepdown,
            stepover: Some(self.stepover.unwrap_or(0.4)),
            peck: self.peck,
        }
    }

    pub fn reach(&self) -> f64 {
        self.reach.unwrap_or(self.flute_length + 5.0)
    }

    pub fn holder_diameter(&self) -> f64 {
        self.holder_diameter.unwrap_or((3.0 * self.diameter).max(30.0))
    }
}

/// A tool library file for the twin.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CtxLibrary {
    #[serde(default = "one")]
    pub version: u32,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub note: String,
    #[serde(default)]
    pub tools: Vec<LatheTool>,
    #[serde(default)]
    pub live_tools: Vec<LiveTool>,
}

fn positive(what: &str, v: f64, max: f64) -> Result<()> {
    if v.is_finite() && v > 0.0 && v <= max { Ok(()) } else { Err(CamError::Invalid(format!("{what} must be above 0 and at most {max}; it is {v}"))) }
}

fn not_negative(what: &str, v: f64, max: f64) -> Result<()> {
    if v.is_finite() && (0.0..=max).contains(&v) { Ok(()) } else { Err(CamError::Invalid(format!("{what} must be from 0 to {max}; it is {v}"))) }
}

fn validate_live(tools: &[LiveTool], what: &str) -> Result<()> {
    if tools.len() > MAX_LIVE {
        return Err(CamError::Invalid(format!("{what}: at most {MAX_LIVE} live tools")));
    }
    for (i, t) in tools.iter().enumerate() {
        let at = |f: &str| format!("{what}[{i}] (T{}).{f}", t.number);
        if t.number == 0 || t.number > 99 {
            return Err(CamError::Invalid(format!("{what}[{i}].number must be 1 to 99; it is {}", t.number)));
        }
        if tools.iter().take(i).any(|o| o.number == t.number) {
            return Err(CamError::Invalid(format!("live tool T{} is listed twice", t.number)));
        }
        if t.kind == crate::job::ToolKind::Chamfer {
            return Err(CamError::Unsupported(format!("{}: a chamfer cutter as a live tool", at("kind"))));
        }
        positive(&at("diameter"), t.diameter, 100.0)?;
        positive(&at("flute_length"), t.flute_length, 300.0)?;
        positive(&at("rpm"), t.rpm, 100_000.0)?;
        positive(&at("feed"), t.feed, 100_000.0)?;
        positive(&at("plunge"), t.plunge, 100_000.0)?;
        if !(1..=16).contains(&t.flutes) {
            return Err(CamError::Invalid(format!("{} must be 1 to 16", at("flutes"))));
        }
        for (f, v) in [("stepdown", t.stepdown), ("peck", t.peck), ("reach", t.reach), ("holder_diameter", t.holder_diameter)] {
            if let Some(v) = v {
                positive(&at(f), v, 500.0)?;
            }
        }
        if let Some(s) = t.stepover
            && !(s.is_finite() && s > 0.01 && s <= 1.0)
        {
            return Err(CamError::Invalid(format!("{} must be above 0.01 and at most 1", at("stepover"))));
        }
        if let Some(a) = t.tip_angle_deg
            && !(a.is_finite() && a > 20.0 && a < 179.0)
        {
            return Err(CamError::Invalid(format!("{} must be between 20 and 179 degrees", at("tip_angle_deg"))));
        }
    }
    Ok(())
}

impl CtxJob {
    /// Read a job; a library it names is read with `load`.
    pub fn from_json_with(text: &str, load: &dyn Fn(&str) -> Result<String>) -> Result<CtxJob> {
        let mut job: CtxJob = serde_json::from_str(text).map_err(|e| CamError::Invalid(format!("turn-mill job: {e}")))?;
        validate_live(&job.live_tools, "live_tools")?;
        let lib = match job.library.trim() {
            "" => Some(CTX_TWIN_SIM_LIBRARY.to_string()),
            "none" => None,
            name => Some(load(name)?),
        };
        if let Some(text) = lib {
            let lib: CtxLibrary = serde_json::from_str(&text).map_err(|e| CamError::Invalid(format!("turn-mill tool library: {e}")))?;
            validate_live(&lib.live_tools, "library.live_tools")?;
            // a station holds one tool: the job's own, turning or live, first
            let taken = |n: u32, job: &CtxJob| job.tools.iter().any(|o| o.number == n) || job.live_tools.iter().any(|o| o.number == n);
            let mut add_turning = Vec::new();
            for t in lib.tools {
                if !taken(t.number, &job) {
                    add_turning.push(t);
                }
            }
            let mut add_live = Vec::new();
            for t in lib.live_tools {
                if !taken(t.number, &job) {
                    add_live.push(t);
                }
            }
            job.tools.extend(add_turning);
            job.live_tools.extend(add_live);
            job.tools.sort_by_key(|t| t.number);
            job.live_tools.sort_by_key(|t| t.number);
        }
        job.validate()?;
        Ok(job)
    }

    /// Read a job naming no library file.
    pub fn from_json(text: &str) -> Result<CtxJob> {
        CtxJob::from_json_with(text, &|p| Err(CamError::Invalid(format!("library {p}: no library files can be read here"))))
    }

    pub fn validate(&self) -> Result<()> {
        if self.version != 1 {
            return Err(CamError::Unsupported(format!("turn-mill job format version {}", self.version)));
        }
        let m = &self.machine;
        positive("machine.max_rpm", m.max_rpm, 100_000.0)?;
        positive("machine.live_max_rpm", m.live_max_rpm, 100_000.0)?;
        positive("machine.sync_max_rpm", m.sync_max_rpm, 100_000.0)?;
        positive("machine.z_travel", m.z_travel, 10_000.0)?;
        if !(1..=64).contains(&m.stations) {
            return Err(CamError::Invalid(format!("machine.stations must be 1 to 64; it is {}", m.stations)));
        }
        let s = &self.stock;
        not_negative("stock.back_allowance", s.back_allowance, 10.0)?;
        if let Some(v) = s.stick_out {
            positive("stock.stick_out", v, 10_000.0)?;
        }
        if let Some(g) = self.transfer.grip {
            positive("transfer.grip", g, 1000.0)?;
        }
        if let Some(z) = self.transfer.pickoff_z
            && !(z.is_finite() && z.abs() <= 10_000.0)
        {
            return Err(CamError::Invalid("transfer.pickoff_z must be within ±10000".into()));
        }
        for (what, v) in [("post.work_z_1", self.post.work_z_1), ("post.work_z_2", self.post.work_z_2)] {
            if let Some(v) = v
                && !(v.is_finite() && v.abs() <= 10_000.0)
            {
                return Err(CamError::Invalid(format!("{what} must be within ±10000")));
            }
        }
        if let Some(w) = self.post.transfer_wait {
            positive("post.transfer_wait", w, 86_400.0)?;
        }
        for t in &self.live_tools {
            if self.tools.iter().any(|o| o.number == t.number) {
                return Err(CamError::Invalid(format!("T{} is both a turning and a live tool", t.number)));
            }
        }
        // the rest as a lathe job
        self.side_job(1, 0.0)?.validate()
    }

    pub fn tool(&self, n: u32) -> Option<&LatheTool> {
        self.tools.iter().find(|t| t.number == n)
    }

    pub fn live(&self, n: u32) -> Option<&LiveTool> {
        self.live_tools.iter().find(|t| t.number == n)
    }

    /// The turret station tool `n` is in.
    pub fn station(&self, n: u32) -> u32 {
        n % self.machine.stations.max(1)
    }

    /// The lathe job for one side's turning: side 1 from the bar, side 2 the part as cut off (its
    /// back allowance to face off, held by `grip` of its front).
    pub fn side_job(&self, side: u8, grip: f64) -> Result<LatheJob> {
        let s = &self.stock;
        let mut strategy = self.strategy.clone();
        let stock = if side == 1 {
            crate::lathe::job::Bar {
                diameter: s.diameter,
                length: s.stick_out,
                in_chuck: 0.0,
                face_allowance: s.face_allowance,
                parting_allowance: s.parting_allowance,
                chuck_clearance: s.chuck_clearance,
                material: s.material.clone(),
            }
        } else {
            // facing the back is the last cut there: nothing left on it for a finishing pass
            strategy.leave_axial = 0.0;
            crate::lathe::job::Bar {
                diameter: s.diameter,
                length: None,
                in_chuck: grip,
                face_allowance: s.back_allowance,
                parting_allowance: 0.0,
                chuck_clearance: s.chuck_clearance,
                material: s.material.clone(),
            }
        };
        let mut part = self.part.clone();
        if side == 2 {
            part.front = match part.front {
                crate::lathe::job::Front::PlusZ => crate::lathe::job::Front::MinusZ,
                crate::lathe::job::Front::MinusZ => crate::lathe::job::Front::PlusZ,
            };
            part.threads.clear();
        }
        let job = LatheJob {
            version: 1,
            machine: LatheMachine {
                name: format!("{}, side {side}", self.machine.name),
                max_rpm: self.machine.max_rpm,
                z_travel: self.machine.z_travel,
                turret: (1..=self.machine.stations).collect(),
                tool_side: ToolSide::Front,
                spindle: SpindleDirection::M3,
            },
            stock,
            part,
            library: "none".into(),
            tools: self.tools.clone(),
            strategy,
            post: self.post.lathe.clone(),
        };
        Ok(job)
    }
}
