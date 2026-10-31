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
    #[serde(default)]
    pub strategy: Strategy,
    pub parts: Vec<PartRef>,
}

/// How the toolpaths are made: which tool does what, and the cutting allowances.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields, default)]
pub struct Strategy {
    /// Tool for facing and roughing (default: the largest flat end mill).
    pub rough_tool: Option<u32>,
    /// Tool for floors, walls and holes no drill fits (default: the smallest flat end mill).
    pub finish_tool: Option<u32>,
    /// Tool for cutting the parts out (default: the largest flat end mill long enough to cut
    /// through, else the largest).
    pub profile_tool: Option<u32>,
    /// Tool for countersinks and chamfers (default: the first chamfer cutter).
    pub chamfer_tool: Option<u32>,
    /// Material roughing leaves on walls and floors for the finishing passes, mm (default 0.2).
    pub stock_to_leave: f64,
    /// Steepest ramp into the material, degrees from horizontal (default 3).
    pub ramp_angle_deg: f64,
    /// Height above the sheet for rapid moves, mm (default 5).
    pub clearance: f64,
    /// How far above a surface feeds start, mm (default 1).
    pub retract: f64,
    /// Tabs holding each part to the sheet as its outline is cut through.
    pub tabs: Tabs,
    /// How far below a chamfer's foot the chamfer cutter's tip goes, mm (default 0.3): the cut
    /// uses the flank, not the fragile tip.
    pub chamfer_tip_below: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields, default)]
pub struct Tabs {
    /// Tabs round each part (default 4; 0: none, as when the parts are held down otherwise).
    pub count: u32,
    /// Length of a tab along the outline, mm (default 6).
    pub width: f64,
    /// Height of a tab above the bottom of the part, mm (default 1).
    pub height: f64,
}

impl Default for Tabs {
    fn default() -> Self {
        Tabs { count: 4, width: 6.0, height: 1.0 }
    }
}

impl Default for Strategy {
    fn default() -> Self {
        Strategy {
            rough_tool: None,
            finish_tool: None,
            profile_tool: None,
            chamfer_tool: None,
            stock_to_leave: 0.2,
            ramp_angle_deg: 3.0,
            clearance: 5.0,
            retract: 1.0,
            tabs: Tabs::default(),
            chamfer_tip_below: 0.3,
        }
    }
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
        let st = &self.strategy;
        length("strategy.stock_to_leave", st.stock_to_leave, 0.0)?;
        length("strategy.clearance", st.clearance, 0.5)?;
        length("strategy.retract", st.retract, 0.1)?;
        length("strategy.chamfer_tip_below", st.chamfer_tip_below, 0.0)?;
        length("strategy.tabs.width", st.tabs.width, 0.0)?;
        length("strategy.tabs.height", st.tabs.height, 0.0)?;
        if st.retract > st.clearance {
            return Err(CamError::Invalid("strategy.retract must not be above strategy.clearance".into()));
        }
        if !(st.ramp_angle_deg.is_finite() && st.ramp_angle_deg >= 0.5 && st.ramp_angle_deg <= 90.0) {
            return Err(CamError::Invalid(format!("strategy.ramp_angle_deg must be from 0.5 to 90; it is {}", st.ramp_angle_deg)));
        }
        if st.tabs.count > 100 {
            return Err(CamError::Invalid("strategy.tabs.count: at most 100 tabs".into()));
        }
        for (field, n) in
            [("rough_tool", st.rough_tool), ("finish_tool", st.finish_tool), ("profile_tool", st.profile_tool), ("chamfer_tool", st.chamfer_tool)]
        {
            if let Some(n) = n {
                let want = if field == "chamfer_tool" { ToolKind::Chamfer } else { ToolKind::Flat };
                match self.tool(n) {
                    None => return Err(CamError::Invalid(format!("strategy.{field}: no tool T{n} in tools"))),
                    Some(t) if t.kind != want => {
                        return Err(CamError::Invalid(format!("strategy.{field}: T{n} is a {:?} tool, not {:?}", t.kind, want)));
                    }
                    _ => {}
                }
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

    fn flats(&self) -> impl Iterator<Item = &Tool> {
        self.tools.iter().filter(|t| t.kind == ToolKind::Flat)
    }

    /// The tool for facing and roughing.
    pub fn rough_tool(&self) -> Option<&Tool> {
        self.strategy.rough_tool.and_then(|n| self.tool(n)).or_else(|| self.flats().max_by(|a, b| a.diameter.total_cmp(&b.diameter)))
    }

    /// The tool for finishing and for holes no drill fits.
    pub fn finish_tool(&self) -> Option<&Tool> {
        self.strategy.finish_tool.and_then(|n| self.tool(n)).or_else(|| self.flats().min_by(|a, b| a.diameter.total_cmp(&b.diameter)))
    }

    /// The tool for cutting parts out of the sheet.
    pub fn profile_tool(&self) -> Option<&Tool> {
        let through = self.stock.thickness + self.stock.spoilboard_allowance;
        self.strategy.profile_tool.and_then(|n| self.tool(n)).or_else(|| {
            self.flats().filter(|t| t.flute_length >= through).max_by(|a, b| a.diameter.total_cmp(&b.diameter)).or_else(|| self.rough_tool())
        })
    }

    pub fn chamfer_tool(&self) -> Option<&Tool> {
        self.strategy.chamfer_tool.and_then(|n| self.tool(n)).or_else(|| self.tools.iter().find(|t| t.kind == ToolKind::Chamfer))
    }

    /// A drill of `diameter` (within 0.01 mm).
    pub fn drill(&self, diameter: f64) -> Option<&Tool> {
        self.tools.iter().find(|t| t.kind == ToolKind::Drill && (t.diameter - diameter).abs() <= 0.01)
    }

    /// The tool with LinuxCNC number `n`.
    pub fn tool(&self, n: u32) -> Option<&Tool> {
        self.tools.iter().find(|t| t.number == n)
    }
}
