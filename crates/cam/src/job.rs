//! A CAM job: one sheet of stock, the tools in the machine, and the parts to cut from it.
//!
//! The job is a small JSON document of its own (parts come from several designs). Work zero is
//! the top face of the sheet at its front-left corner: X to the right, Y to the back, Z up, so
//! the sheet spans X 0…width, Y 0…depth, Z −thickness…0.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::region::MAX_COORD;
use crate::{CamError, Result};

/// Most parts a job may cut (all copies together).
pub const MAX_COPIES: u32 = 10_000;

/// Most tools a job may list.
pub const MAX_TOOLS: usize = 200;

#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Job {
    /// The job format's version: 1.
    #[serde(default = "one")]
    pub version: u32,
    pub stock: Stock,
    #[serde(default)]
    pub nest: NestOptions,
    #[serde(default)]
    pub tools: Vec<Tool>,
    pub parts: Vec<PartRef>,
}

fn one() -> u32 {
    1
}

/// The sheet the parts are cut from.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Stock {
    /// Width (X) and depth (Y) of the sheet, mm.
    pub size: [f64; 2],
    /// Thickness of the sheet, mm.
    pub thickness: f64,
    /// Free text, such as `aluminium 5083`; feeds and speeds come from the tools.
    #[serde(default)]
    pub material: String,
    /// How far through-cuts go below the sheet into the spoilboard, mm (default 0.2).
    #[serde(default = "spoil")]
    pub spoilboard_allowance: f64,
}

fn spoil() -> f64 {
    0.2
}

/// How parts are laid out on the sheet.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NestOptions {
    /// Clear border round the sheet's edge, mm (default 10).
    #[serde(default = "ten")]
    pub margin: f64,
    /// Gap between parts, mm (default 10). Make it at least the profile cutter's diameter plus
    /// a little, so the cutter's paths round neighbouring parts don't meet.
    #[serde(default = "ten")]
    pub spacing: f64,
    /// May parts be turned by 90° to fit (default true)?
    #[serde(default = "yes")]
    pub rotate: bool,
}

fn ten() -> f64 {
    10.0
}

fn yes() -> bool {
    true
}

impl Default for NestOptions {
    fn default() -> Self {
        NestOptions { margin: 10.0, spacing: 10.0, rotate: true }
    }
}

/// What kind of cutter a tool is.
#[derive(Clone, Copy, Debug, Deserialize, Serialize, JsonSchema, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ToolKind {
    /// A flat end mill: roughing, floors, walls, profiles, helical bores.
    Flat,
    /// A twist drill (`tip_angle_deg`, default 118).
    Drill,
    /// A chamfer or countersink cutter (`tip_angle_deg` included angle, default 90; `diameter`
    /// is its largest diameter).
    Chamfer,
}

/// A tool in the machine's tool table: LinuxCNC tool number, geometry and cutting data.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Tool {
    /// LinuxCNC tool (and pocket) number, `T<number> M6`; its length comes from the tool table.
    pub number: u32,
    #[serde(default)]
    pub name: String,
    pub kind: ToolKind,
    /// Cutting diameter, mm.
    pub diameter: f64,
    /// Length of the cutting part, mm: the deepest a single pass may reach.
    pub flute_length: f64,
    #[serde(default = "two")]
    pub flutes: u32,
    /// Drill point or chamfer included angle, degrees.
    #[serde(default)]
    pub tip_angle_deg: Option<f64>,
    /// Spindle speed, rpm.
    pub rpm: f64,
    /// Cutting feed, mm/min.
    pub feed: f64,
    /// Plunge or ramp feed, mm/min.
    pub plunge: f64,
    /// Depth of one pass, mm (default: the diameter).
    #[serde(default)]
    pub stepdown: Option<f64>,
    /// Side step of clearing passes as a fraction of the diameter (default 0.15).
    #[serde(default)]
    pub stepover: Option<f64>,
    /// Peck depth for drilling, mm (G83; none: drill in one go, G81).
    #[serde(default)]
    pub peck: Option<f64>,
}

fn two() -> u32 {
    2
}

impl Tool {
    pub fn tip_angle(&self) -> f64 {
        self.tip_angle_deg.unwrap_or(match self.kind {
            ToolKind::Drill => 118.0,
            _ => 90.0,
        })
    }

    pub fn stepdown(&self) -> f64 {
        self.stepdown.unwrap_or(self.diameter)
    }

    pub fn stepover(&self) -> f64 {
        self.stepover.unwrap_or(0.15) * self.diameter
    }
}

/// Copies of one body of one design.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PartRef {
    /// A name for the part in reports and the G-code's comments (default: the body's name).
    #[serde(default)]
    pub name: String,
    /// The design file (`.sc`, or a STEP file), relative to the job file; empty for the open design.
    #[serde(default)]
    pub design: String,
    /// The body to cut.
    pub body: String,
    #[serde(default = "one")]
    pub quantity: u32,
}

impl PartRef {
    pub fn label(&self) -> &str {
        if self.name.is_empty() { &self.body } else { &self.name }
    }
}

fn length(what: &str, v: f64, min: f64) -> Result<()> {
    if !v.is_finite() || v < min || v > MAX_COORD {
        return Err(CamError::Invalid(format!("{what} must be a length from {min} to {MAX_COORD} mm; it is {v}")));
    }
    Ok(())
}

impl Job {
    /// Read a job from its JSON text.
    pub fn from_json(text: &str) -> Result<Job> {
        let job: Job = serde_json::from_str(text).map_err(|e| CamError::Invalid(format!("job: {e}")))?;
        job.validate()?;
        Ok(job)
    }

    /// Check every number and reference before anything is planned.
    pub fn validate(&self) -> Result<()> {
        if self.version != 1 {
            return Err(CamError::Unsupported(format!("job format version {} (this build reads version 1)", self.version)));
        }
        let s = &self.stock;
        length("stock.size[0] (width)", s.size[0], 1.0)?;
        length("stock.size[1] (depth)", s.size[1], 1.0)?;
        length("stock.thickness", s.thickness, 0.01)?;
        length("stock.spoilboard_allowance", s.spoilboard_allowance, 0.0)?;
        if s.spoilboard_allowance > 5.0 {
            return Err(CamError::Invalid(format!(
                "stock.spoilboard_allowance {} mm cuts deep into the spoilboard; at most 5 mm",
                s.spoilboard_allowance
            )));
        }
        length("nest.margin", self.nest.margin, 0.0)?;
        length("nest.spacing", self.nest.spacing, 0.0)?;
        if 2.0 * self.nest.margin >= s.size[0].min(s.size[1]) {
            return Err(CamError::Invalid("nest.margin leaves no room on the sheet".into()));
        }
        if self.tools.len() > MAX_TOOLS {
            return Err(CamError::Invalid(format!("at most {MAX_TOOLS} tools")));
        }
        for (i, t) in self.tools.iter().enumerate() {
            let at = |field: &str| format!("tools[{i}] (T{}).{field}", t.number);
            if t.number == 0 || t.number > 99_999 {
                return Err(CamError::Invalid(format!("tools[{i}].number must be 1 to 99999; it is {}", t.number)));
            }
            if self.tools.iter().take(i).any(|o| o.number == t.number) {
                return Err(CamError::Invalid(format!("tool number T{} is listed twice", t.number)));
            }
            length(&at("diameter"), t.diameter, 0.05)?;
            length(&at("flute_length"), t.flute_length, 0.05)?;
            for (field, v) in [("rpm", t.rpm), ("feed", t.feed), ("plunge", t.plunge)] {
                if !v.is_finite() || v <= 0.0 || v > 1e6 {
                    return Err(CamError::Invalid(format!("{} must be positive and at most 1e6; it is {v}", at(field))));
                }
            }
            if let Some(a) = t.tip_angle_deg
                && !(a.is_finite() && a > 1.0 && a < 179.0)
            {
                return Err(CamError::Invalid(format!("{} must be between 1 and 179 degrees; it is {a}", at("tip_angle_deg"))));
            }
            if let Some(d) = t.stepdown {
                length(&at("stepdown"), d, 0.01)?;
            }
            if let Some(p) = t.peck {
                length(&at("peck"), p, 0.01)?;
            }
            if let Some(f) = t.stepover
                && !(f.is_finite() && f > 0.0 && f <= 1.0)
            {
                return Err(CamError::Invalid(format!("{} is a fraction of the diameter, above 0 and at most 1; it is {f}", at("stepover"))));
            }
        }
        if self.parts.is_empty() {
            return Err(CamError::Invalid("a job needs at least one part".into()));
        }
        let mut copies = 0u64;
        for (i, p) in self.parts.iter().enumerate() {
            if p.body.is_empty() {
                return Err(CamError::Invalid(format!("parts[{i}].body: the name of the body to cut")));
            }
            if p.quantity == 0 {
                return Err(CamError::Invalid(format!("parts[{i}] ({}): quantity must be at least 1", p.label())));
            }
            copies += u64::from(p.quantity);
        }
        if copies > u64::from(MAX_COPIES) {
            return Err(CamError::Invalid(format!("{copies} copies in all; a job cuts at most {MAX_COPIES}")));
        }
        Ok(())
    }

    /// The tool with LinuxCNC number `n`.
    pub fn tool(&self, n: u32) -> Option<&Tool> {
        self.tools.iter().find(|t| t.number == n)
    }
}
