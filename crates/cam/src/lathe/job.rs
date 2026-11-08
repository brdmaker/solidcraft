//! A lathe job: the machine, the bar, the part and the tools, read from JSON.
//!
//! Coordinates: the part is turned about its Z axis. In the part's own frame z runs from the end
//! that faces the chuck to its front face; in the program's (work) frame Z0 is the finished front
//! face and the part lies at negative Z, X is a diameter (G7).

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::{CamError, Result};

/// The built-in tool library for the Emco Compact 5.
pub const EMCO_COMPACT5_LIBRARY: &str = include_str!("../../library/lathe-emco-compact5.json");

/// Most tools a lathe job or library may list.
const MAX_TOOLS: usize = 64;

#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LatheJob {
    /// The format's version: 1.
    #[serde(default = "one")]
    pub version: u32,
    #[serde(default)]
    pub machine: LatheMachine,
    pub stock: Bar,
    pub part: TurnedPart,
    /// A tool library file (relative to the job) filling in the tools the job doesn't list; empty:
    /// the Emco Compact 5's; `none`: only the job's own tools.
    #[serde(default)]
    pub library: String,
    #[serde(default)]
    pub tools: Vec<LatheTool>,
    #[serde(default)]
    pub strategy: LatheStrategy,
    #[serde(default)]
    pub post: LathePost,
}

fn one() -> u32 {
    1
}

/// The lathe.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields, default)]
pub struct LatheMachine {
    pub name: String,
    /// Highest spindle speed, rpm (the G96 limit).
    pub max_rpm: f64,
    /// Travel along Z, mm: the bar's free length is checked against it.
    pub z_travel: f64,
    /// The turret's stations; a plan using another tool says so. Empty: no check.
    pub turret: Vec<u32>,
    /// Where the tools are: `front` (between the operator and the work) or `back`. The program is
    /// the same either way (X is a diameter); it is noted in its header.
    pub tool_side: ToolSide,
    /// The spindle's turning direction for these tools: `m3` (default) or `m4`.
    pub spindle: SpindleDirection,
}

impl Default for LatheMachine {
    fn default() -> Self {
        LatheMachine {
            name: "Emco Compact 5".into(),
            max_rpm: 3200.0,
            z_travel: 300.0,
            turret: (1..=6).collect(),
            tool_side: ToolSide::Front,
            spindle: SpindleDirection::M3,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, JsonSchema, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum ToolSide {
    #[default]
    Front,
    Back,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, JsonSchema, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum SpindleDirection {
    #[default]
    M3,
    M4,
}

/// The bar in the chuck.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Bar {
    /// Diameter, mm.
    pub diameter: f64,
    /// Length of the piece, mm (default: the part, the facing and parting allowances, the
    /// clearance to the jaws and what the chuck holds, rounded up to whole millimetres).
    #[serde(default)]
    pub length: Option<f64>,
    /// How much of it the chuck holds, mm (default 20).
    #[serde(default = "in_chuck")]
    pub in_chuck: f64,
    /// Faced off the front, mm (default 0.5).
    #[serde(default = "face_allowance")]
    pub face_allowance: f64,
    /// Left turned behind the part to part it off, mm (default 3).
    #[serde(default = "parting_allowance")]
    pub parting_allowance: f64,
    /// The tools keep this far from the jaws, mm (default 2).
    #[serde(default = "chuck_clearance")]
    pub chuck_clearance: f64,
    /// `aluminium`, `brass` or `steel`: picks each tool's surface speed.
    #[serde(default = "steel")]
    pub material: String,
}

fn in_chuck() -> f64 {
    20.0
}
fn face_allowance() -> f64 {
    0.5
}
fn parting_allowance() -> f64 {
    3.0
}
fn chuck_clearance() -> f64 {
    2.0
}
fn steel() -> String {
    "steel".into()
}

/// The part to turn.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TurnedPart {
    #[serde(default)]
    pub name: String,
    /// Its design or STEP file (relative to the job); empty for the open design.
    #[serde(default)]
    pub design: String,
    pub body: String,
    /// Which end of its Z axis is the front (away from the chuck): `+z` (default) or `-z`.
    #[serde(default)]
    pub front: Front,
    /// External threads to cut (a design's cosmetic threads are not in a STEP file).
    #[serde(default)]
    pub threads: Vec<ThreadSpec>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, JsonSchema, PartialEq, Eq, Default)]
pub enum Front {
    #[default]
    #[serde(rename = "+z")]
    PlusZ,
    #[serde(rename = "-z")]
    MinusZ,
}

/// A lathe tool. Turning tools and boring bars: an insert of `insert_angle_deg` held at
/// `approach_deg` (the angle between its leading edge and the feed toward the chuck), cutting
/// toward the chuck, outside or in a bore. A grooving blade `width` wide cuts `max_depth` deep
/// (touched off on its corner toward the chuck); a threading tool cuts a `tip_angle_deg` V; a
/// drill of `diameter` with its point's `tip_angle_deg` drills along the axis `peck` at a time.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LatheTool {
    pub number: u32,
    #[serde(default)]
    pub name: String,
    /// `turning`, `boring`, `grooving`, `threading` or `drill`.
    pub kind: LatheToolKind,
    #[serde(default)]
    pub insert_angle_deg: f64,
    #[serde(default)]
    pub approach_deg: f64,
    #[serde(default)]
    pub nose_radius: f64,
    /// Grooving: the blade's width, mm.
    #[serde(default)]
    pub width: Option<f64>,
    /// Grooving: how deep the blade reaches; boring: how far the bar reaches into a bore, mm.
    #[serde(default)]
    pub max_depth: Option<f64>,
    /// Drills: the diameter, mm.
    #[serde(default)]
    pub diameter: Option<f64>,
    /// Drills: the point's included angle (default 118); threading: the thread's (default 60).
    #[serde(default)]
    pub tip_angle_deg: Option<f64>,
    /// Drills: depth of each peck, mm (default the diameter).
    #[serde(default)]
    pub peck: Option<f64>,
    /// Boring bars: the smallest bore the bar fits, mm.
    #[serde(default)]
    pub min_bore: Option<f64>,
    /// The fastest the tool runs, rpm (threading and drilling run at a fixed speed).
    #[serde(default)]
    pub max_rpm: Option<f64>,
    /// Feed, mm per revolution.
    pub feed: f64,
    /// Most depth of cut, mm on the radius.
    pub depth: f64,
    /// Surface speed by material, m/min.
    pub surface_speed: BTreeMap<String, f64>,
    /// What holds the insert, blade or drill: checked against the bar and the jaws as the tool
    /// moves (none: not checked).
    #[serde(default)]
    pub holder: Option<Holder>,
}

/// A tool's holder: a block (a square shank, a blade's clamp) or a round bar (a boring bar's
/// shank, a drill's collet chuck), placed from the tool's touch-off point. In the plane the tool
/// cuts in (the axis and X): along Z (+ toward the front) from `z[0]` to `z[1]`, out from the
/// axis along X from `x[0]` to `x[1]` (negative: across the touch-off point toward and past the
/// axis), `height` across that plane (Y). A round bar is `x[1] − x[0]` across.
#[derive(Clone, Copy, Debug, Deserialize, Serialize, JsonSchema, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Holder {
    pub z: [f64; 2],
    pub x: [f64; 2],
    /// Across the cutting plane, mm (a round bar: its diameter).
    #[serde(default)]
    pub height: f64,
    #[serde(default)]
    pub round: bool,
}

impl Holder {
    /// The radii the holder covers round the axis with the touch-off point at radius `r`: the
    /// nearest and farthest of its points from the axis (it reaches across the axis where its X
    /// span does).
    pub fn radii(&self, r: f64) -> [f64; 2] {
        let (x0, x1) = (r + self.x[0], r + self.x[1]);
        if self.round {
            let (c, half) = ((x0 + x1) / 2.0, (x1 - x0) / 2.0);
            [(c.abs() - half).max(0.0), c.abs() + half]
        } else {
            let near = if x0 <= 0.0 && x1 >= 0.0 { 0.0 } else { x0.abs().min(x1.abs()) };
            [near, x0.abs().max(x1.abs()).hypot(self.height / 2.0)]
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, JsonSchema, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum LatheToolKind {
    Turning,
    Boring,
    Grooving,
    Threading,
    Drill,
}

impl LatheToolKind {
    pub fn name(self) -> &'static str {
        match self {
            LatheToolKind::Turning => "turning",
            LatheToolKind::Boring => "boring",
            LatheToolKind::Grooving => "grooving",
            LatheToolKind::Threading => "threading",
            LatheToolKind::Drill => "drill",
        }
    }
}

/// Clearance the insert keeps from a surface it trails over, degrees.
pub const TRAIL_CLEARANCE_DEG: f64 = 2.0;

impl LatheTool {
    /// The leading edge's direction from the nose, degrees from +Z (toward the front) in the
    /// (z, r) plane: the feed toward the chuck is 180°, the edge `approach_deg` round from it.
    pub fn lead_angle(&self) -> f64 {
        180.0 - self.approach_deg
    }

    /// The trailing edge's direction from the nose, degrees from +Z.
    pub fn trail_angle(&self) -> f64 {
        self.lead_angle() - self.insert_angle_deg
    }

    /// The steepest slope the tool can follow down toward the chuck (the trailing edge's angle
    /// less a clearance), degrees; never below 0.
    pub fn descent_deg(&self) -> f64 {
        (self.trail_angle() - TRAIL_CLEARANCE_DEG).max(0.0)
    }

    /// Surface speed for `material`, m/min.
    pub fn speed(&self, material: &str) -> Option<f64> {
        self.surface_speed.get(material).copied()
    }

    /// Grooving blade's width (default 2).
    pub fn blade_width(&self) -> f64 {
        self.width.unwrap_or(2.0)
    }

    /// Grooving blade's or boring bar's reach (default 10).
    pub fn reach(&self) -> f64 {
        self.max_depth.unwrap_or(10.0)
    }

    /// Drill's or thread's included angle.
    pub fn point_angle(&self) -> f64 {
        self.tip_angle_deg.unwrap_or(match self.kind {
            LatheToolKind::Threading => 60.0,
            _ => 118.0,
        })
    }

    /// Spindle speed for cutting at `diameter` in `material`, rpm: the surface speed's, held
    /// under the tool's and the machine's limits.
    pub fn rpm(&self, material: &str, diameter: f64, machine_max: f64) -> f64 {
        let vc = self.speed(material).unwrap_or(50.0);
        let n = vc * 1000.0 / (std::f64::consts::PI * diameter.max(0.5));
        n.min(self.max_rpm.unwrap_or(machine_max)).min(machine_max).round()
    }
}

/// A thread on the part: ISO metric, from `z_from` to `z_to` along the part's axis (its own z,
/// from its chuck end).
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ThreadSpec {
    /// `M12`, or `M12x1.25` for a fine pitch.
    pub designation: String,
    pub z_from: f64,
    pub z_to: f64,
}

/// ISO metric coarse pitches by nominal diameter.
const COARSE: &[(f64, f64)] = &[
    (1.0, 0.25),
    (1.2, 0.25),
    (1.6, 0.35),
    (2.0, 0.4),
    (2.5, 0.45),
    (3.0, 0.5),
    (4.0, 0.7),
    (5.0, 0.8),
    (6.0, 1.0),
    (8.0, 1.25),
    (10.0, 1.5),
    (12.0, 1.75),
    (14.0, 2.0),
    (16.0, 2.0),
    (18.0, 2.5),
    (20.0, 2.5),
    (22.0, 2.5),
    (24.0, 3.0),
    (27.0, 3.0),
    (30.0, 3.5),
    (33.0, 3.5),
    (36.0, 4.0),
    (39.0, 4.0),
    (42.0, 4.5),
    (45.0, 4.5),
    (48.0, 5.0),
];

impl ThreadSpec {
    /// The nominal (major) diameter and pitch of the designation.
    pub fn size(&self) -> Result<(f64, f64)> {
        let d = self.designation.trim();
        let bad = || CamError::Invalid(format!("thread {d}: give it as M<diameter> or M<diameter>x<pitch>"));
        let rest = d.strip_prefix('M').or_else(|| d.strip_prefix('m')).ok_or_else(bad)?;
        let (dia, pitch) = match rest.split_once(['x', 'X']) {
            Some((a, b)) => (a.trim().parse::<f64>().map_err(|_| bad())?, Some(b.trim().parse::<f64>().map_err(|_| bad())?)),
            None => (rest.trim().parse::<f64>().map_err(|_| bad())?, None),
        };
        let pitch = match pitch {
            Some(p) => p,
            None => COARSE
                .iter()
                .find(|(n, _)| (n - dia).abs() < 1e-9)
                .map(|(_, p)| *p)
                .ok_or_else(|| CamError::Invalid(format!("thread {d}: no coarse pitch for M{dia}; give one (M{dia}x<pitch>)")))?,
        };
        if !(dia.is_finite() && (1.0..=200.0).contains(&dia) && pitch.is_finite() && pitch > 0.05 && pitch < dia / 2.0) {
            return Err(bad());
        }
        Ok((dia, pitch))
    }
}

/// How the part is turned.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields, default)]
pub struct LatheStrategy {
    /// Tool for facing and roughing (default the library's first turning tool).
    pub rough_tool: Option<u32>,
    /// Tool for the finishing passes (default the turning tool reaching down the steepest slopes).
    pub finish_tool: Option<u32>,
    /// Material roughing leaves on diameters, mm on the radius (default 0.2).
    pub leave_radial: f64,
    /// Material roughing leaves on faces, mm (default 0.1).
    pub leave_axial: f64,
    /// Feeds start this far from the material, mm (default 1).
    pub clearance: f64,
    /// Where the turret indexes: this far outside the bar and in front of it, mm (default 10).
    pub change_clearance: f64,
}

impl Default for LatheStrategy {
    fn default() -> Self {
        LatheStrategy { rough_tool: None, finish_tool: None, leave_radial: 0.2, leave_axial: 0.1, clearance: 1.0, change_clearance: 10.0 }
    }
}

/// The LinuxCNC lathe program's settings.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields, default)]
pub struct LathePost {
    pub coolant: crate::job::Coolant,
    /// Decimals of coordinates, default 4.
    pub decimals: u32,
    /// Path blending tolerance (G64 P), mm (default 0.005).
    pub blend_tolerance: f64,
}

impl Default for LathePost {
    fn default() -> Self {
        LathePost { coolant: crate::job::Coolant::Off, decimals: 4, blend_tolerance: 0.005 }
    }
}

/// A lathe tool library file.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LatheLibrary {
    #[serde(default = "one")]
    pub version: u32,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub note: String,
    pub tools: Vec<LatheTool>,
}

fn positive(what: &str, v: f64, max: f64) -> Result<()> {
    if v.is_finite() && v > 0.0 && v <= max { Ok(()) } else { Err(CamError::Invalid(format!("{what} must be above 0 and at most {max}; it is {v}"))) }
}

fn not_negative(what: &str, v: f64, max: f64) -> Result<()> {
    if v.is_finite() && (0.0..=max).contains(&v) { Ok(()) } else { Err(CamError::Invalid(format!("{what} must be from 0 to {max}; it is {v}"))) }
}

fn validate_tools(tools: &[LatheTool], what: &str) -> Result<()> {
    if tools.len() > MAX_TOOLS {
        return Err(CamError::Invalid(format!("{what}: at most {MAX_TOOLS} tools")));
    }
    for (i, t) in tools.iter().enumerate() {
        let at = |f: &str| format!("{what}[{i}] (T{}).{f}", t.number);
        if t.number == 0 || t.number > 99 {
            return Err(CamError::Invalid(format!("{what}[{i}].number must be 1 to 99; it is {}", t.number)));
        }
        if tools.iter().take(i).any(|o| o.number == t.number) {
            return Err(CamError::Invalid(format!("tool T{} is listed twice", t.number)));
        }
        if matches!(t.kind, LatheToolKind::Turning | LatheToolKind::Boring) {
            if !(t.insert_angle_deg.is_finite() && (20.0..=100.0).contains(&t.insert_angle_deg)) {
                return Err(CamError::Invalid(format!("{} must be from 20 to 100 degrees; it is {}", at("insert_angle_deg"), t.insert_angle_deg)));
            }
            if !(t.approach_deg.is_finite() && (90.0..=120.0).contains(&t.approach_deg)) {
                return Err(CamError::Invalid(format!("{} must be from 90 to 120 degrees; it is {}", at("approach_deg"), t.approach_deg)));
            }
            if t.trail_angle() <= 0.0 {
                return Err(CamError::Invalid(format!(
                    "T{}: an insert of {}° at {}° leaves no trailing edge clear of the work",
                    t.number, t.insert_angle_deg, t.approach_deg
                )));
            }
            positive(&at("nose_radius"), t.nose_radius, 5.0)?;
        } else {
            not_negative(&at("nose_radius"), t.nose_radius, 5.0)?;
        }
        match t.kind {
            LatheToolKind::Grooving => {
                positive(&at("width"), t.width.unwrap_or(0.0), 20.0)?;
                positive(&at("max_depth"), t.reach(), 200.0)?;
            }
            LatheToolKind::Drill => {
                positive(&at("diameter"), t.diameter.unwrap_or(0.0), 100.0)?;
                if let Some(p) = t.peck {
                    positive(&at("peck"), p, 100.0)?;
                }
            }
            LatheToolKind::Boring => {
                positive(&at("min_bore"), t.min_bore.unwrap_or(0.0), 500.0)?;
                positive(&at("max_depth"), t.reach(), 500.0)?;
            }
            LatheToolKind::Threading | LatheToolKind::Turning => {}
        }
        if let Some(h) = &t.holder {
            let span = |v: [f64; 2], f: &str| -> Result<()> {
                if !(v.iter().all(|x| x.is_finite() && x.abs() <= 1000.0) && v[1] - v[0] > 1e-6) {
                    return Err(CamError::Invalid(format!("{}: give it as [from, to], from below to, within 1000 mm", at(f))));
                }
                Ok(())
            };
            span(h.z, "holder.z")?;
            span(h.x, "holder.x")?;
            if !h.round {
                positive(&at("holder.height"), h.height, 1000.0)?;
            }
        }
        if let Some(a) = t.tip_angle_deg
            && !(a.is_finite() && a > 20.0 && a < 179.0)
        {
            return Err(CamError::Invalid(format!("{} must be between 20 and 179 degrees; it is {a}", at("tip_angle_deg"))));
        }
        if let Some(n) = t.max_rpm {
            positive(&at("max_rpm"), n, 100_000.0)?;
        }
        positive(&at("feed"), t.feed, 2.0)?;
        positive(&at("depth"), t.depth, 10.0)?;
        if t.surface_speed.is_empty() || t.surface_speed.len() > 32 {
            return Err(CamError::Invalid(format!("{}: give a speed for each material (1 to 32)", at("surface_speed"))));
        }
        for (m, v) in &t.surface_speed {
            positive(&at(&format!("surface_speed.{m}")), *v, 2000.0)?;
        }
    }
    Ok(())
}

impl LatheJob {
    /// Read a job; a library it names is read with `load`.
    pub fn from_json_with(text: &str, load: &dyn Fn(&str) -> Result<String>) -> Result<LatheJob> {
        let mut job: LatheJob = serde_json::from_str(text).map_err(|e| CamError::Invalid(format!("lathe job: {e}")))?;
        validate_tools(&job.tools, "tools")?;
        // the library named, or the Emco Compact 5's: it fills in the tools the job doesn't list
        // (`none`: only the job's own)
        let lib = match job.library.trim() {
            "" => Some(EMCO_COMPACT5_LIBRARY.to_string()),
            "none" => None,
            name => Some(load(name)?),
        };
        if let Some(text) = lib {
            let lib: LatheLibrary = serde_json::from_str(&text).map_err(|e| CamError::Invalid(format!("lathe tool library: {e}")))?;
            validate_tools(&lib.tools, "library.tools")?;
            for t in lib.tools {
                if !job.tools.iter().any(|o| o.number == t.number) {
                    job.tools.push(t);
                }
            }
            job.tools.sort_by_key(|t| t.number);
        }
        job.validate()?;
        Ok(job)
    }

    /// Read a job naming no library file.
    pub fn from_json(text: &str) -> Result<LatheJob> {
        LatheJob::from_json_with(text, &|p| Err(CamError::Invalid(format!("library {p}: no library files can be read here"))))
    }

    pub fn validate(&self) -> Result<()> {
        if self.version != 1 {
            return Err(CamError::Unsupported(format!("lathe job format version {}", self.version)));
        }
        let m = &self.machine;
        positive("machine.max_rpm", m.max_rpm, 100_000.0)?;
        positive("machine.z_travel", m.z_travel, 10_000.0)?;
        if m.turret.len() > 64 {
            return Err(CamError::Invalid("machine.turret: at most 64 stations".into()));
        }
        let s = &self.stock;
        positive("stock.diameter", s.diameter, 1000.0)?;
        if let Some(l) = s.length {
            positive("stock.length", l, 10_000.0)?;
        }
        not_negative("stock.in_chuck", s.in_chuck, 1000.0)?;
        not_negative("stock.face_allowance", s.face_allowance, 50.0)?;
        not_negative("stock.parting_allowance", s.parting_allowance, 100.0)?;
        not_negative("stock.chuck_clearance", s.chuck_clearance, 100.0)?;
        let st = &self.strategy;
        not_negative("strategy.leave_radial", st.leave_radial, 5.0)?;
        not_negative("strategy.leave_axial", st.leave_axial, 5.0)?;
        positive("strategy.clearance", st.clearance, 100.0)?;
        positive("strategy.change_clearance", st.change_clearance, 500.0)?;
        if !(1..=6).contains(&self.post.decimals) {
            return Err(CamError::Invalid(format!("post.decimals must be 1 to 6; it is {}", self.post.decimals)));
        }
        if !(self.post.blend_tolerance.is_finite() && (0.0005..=0.5).contains(&self.post.blend_tolerance)) {
            return Err(CamError::Invalid(format!("post.blend_tolerance must be from 0.0005 to 0.5 mm; it is {}", self.post.blend_tolerance)));
        }
        for (what, n) in [("rough_tool", st.rough_tool), ("finish_tool", st.finish_tool)] {
            if let Some(n) = n
                && self.tool(n).is_none()
            {
                return Err(CamError::Invalid(format!("strategy.{what}: no tool T{n}")));
            }
        }
        if self.rough_tool().is_none() {
            return Err(CamError::Invalid("the job has no turning tool".into()));
        }
        for t in &self.tools {
            if t.speed(&s.material).is_none() {
                return Err(CamError::Invalid(format!("T{} has no surface speed for {}", t.number, s.material)));
            }
        }
        if self.part.threads.len() > 32 {
            return Err(CamError::Invalid("part.threads: at most 32".into()));
        }
        for th in &self.part.threads {
            th.size()?;
            if !(th.z_from.is_finite() && th.z_to.is_finite() && th.z_to > th.z_from) {
                return Err(CamError::Invalid(format!("thread {}: z_to must be beyond z_from", th.designation)));
            }
        }
        Ok(())
    }

    pub fn tool(&self, n: u32) -> Option<&LatheTool> {
        self.tools.iter().find(|t| t.number == n)
    }

    /// The first tool of `kind`, turret stations first.
    pub fn first_of(&self, kind: LatheToolKind) -> Option<&LatheTool> {
        let turret = &self.machine.turret;
        self.tools.iter().filter(|t| t.kind == kind).min_by_key(|t| (!turret.is_empty() && !turret.contains(&t.number), t.number))
    }

    pub fn rough_tool(&self) -> Option<&LatheTool> {
        match self.strategy.rough_tool {
            Some(n) => self.tool(n),
            None => self.tools.iter().find(|t| t.kind == LatheToolKind::Turning),
        }
    }

    pub fn finish_tool(&self) -> Option<&LatheTool> {
        match self.strategy.finish_tool {
            Some(n) => self.tool(n),
            None => self.tools.iter().filter(|t| t.kind == LatheToolKind::Turning).max_by(|a, b| a.descent_deg().total_cmp(&b.descent_deg())),
        }
    }
}
