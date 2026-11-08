//! The CAM Job window's lathe job: the bar, the part and its threads, and the turning settings
//! as a form, planned and simulated with `cam.lathe.plan` and written out with `cam.lathe.gcode`
//! and `cam.lathe.export_scene` (the window's runner, in `cam_panel.rs`). Simulate shows the
//! turned half section with the paths, the operations, the simulation's findings and the plan's
//! warnings.

use egui::RichText;
use serde_json::{Map, Value, json};

use crate::cam_panel::{Num, at, set, shown};
use crate::i18n::{format, tr};
use crate::theme::Tokens;
use crate::ui_kit::{self, StatusTone};

/// The bar's and the turning's numbers.
pub(crate) const NUMBERS: &[Num] = &[
    Num { path: &["stock", "diameter"], label: "cam_panel.bar_diameter", optional: false },
    Num { path: &["stock", "length"], label: "cam_panel.bar_length", optional: true },
    Num { path: &["stock", "in_chuck"], label: "cam_panel.in_chuck", optional: false },
    Num { path: &["stock", "face_allowance"], label: "cam_panel.face_allowance", optional: false },
    Num { path: &["stock", "parting_allowance"], label: "cam_panel.parting_allowance", optional: false },
    Num { path: &["stock", "chuck_clearance"], label: "cam_panel.chuck_clearance", optional: false },
    Num { path: &["strategy", "leave_radial"], label: "cam_panel.leave_radial", optional: false },
    Num { path: &["strategy", "leave_axial"], label: "cam_panel.leave_axial", optional: false },
    Num { path: &["strategy", "clearance"], label: "cam_panel.approach", optional: false },
    Num { path: &["strategy", "change_clearance"], label: "cam_panel.change_clearance", optional: false },
];

/// How many of [`NUMBERS`] are the bar's.
const BAR_NUMBERS: usize = 6;

/// The bar's materials: the job's values and their labels' message ids.
const MATERIALS: &[&str] = &["aluminium", "brass", "steel"];
const MATERIAL_LABELS: &[&str] = &["cam_panel.aluminium", "cam_panel.brass", "cam_panel.steel"];

/// Which end of the part's Z axis faces away from the chuck.
const FRONTS: &[&str] = &["+z", "-z"];
const FRONT_LABELS: &[&str] = &["cam_panel.front_pz", "cam_panel.front_mz"];

/// Coolant: the job's values and their labels.
const COOLANTS: &[&str] = &["off", "mist", "flood"];
const COOLANT_LABELS: &[&str] = &["cam_panel.coolant_off", "cam_panel.coolant_mist", "cam_panel.coolant_flood"];

/// Most height the half section's picture takes, points.
const PICTURE_HEIGHT: f32 = 240.0;

/// A thread to cut, as edited.
#[derive(Clone, Debug, Default)]
pub struct ThreadRow {
    pub designation: String,
    pub from: String,
    pub to: String,
}

/// What the last simulation found.
pub struct LatheSimulated {
    /// The scene `cam.lathe.plan` describes.
    pub scene: Value,
    pub texture: Option<egui::TextureHandle>,
    pub picture: Option<egui::ColorImage>,
}

/// The lathe job as edited.
pub struct LathePanel {
    /// The job as last read or written: settings the form has no field for are kept from it.
    pub base: Value,
    /// The numbers as typed, one for each of [`NUMBERS`].
    pub numbers: Vec<String>,
    pub material: usize,
    pub coolant: usize,
    /// The part's design or STEP file (absolute), empty for the open design.
    pub design: String,
    pub body: String,
    pub front: usize,
    pub threads: Vec<ThreadRow>,
    pub result: Option<LatheSimulated>,
}

/// The defaults a lathe job takes, for the fields a job file leaves out.
fn defaults() -> Value {
    match solidcraft_cam::lathe::job::LatheJob::from_json(r#"{"stock": {"diameter": 30}, "part": {"body": "-"}}"#) {
        Ok(j) => serde_json::to_value(&j).unwrap_or(Value::Null),
        Err(_) => Value::Null,
    }
}

/// Is `job` a lathe job (one `part`, a bar's `diameter`) rather than a router job?
pub fn is_lathe_job(job: &Value) -> bool {
    job.get("part").is_some_and(Value::is_object) || at(job, &["stock", "diameter"]).is_some()
}

impl LathePanel {
    /// The form for `job` (its part's design relative to `dir`, made absolute).
    pub fn from_job(job: Value, dir: Option<&std::path::Path>) -> LathePanel {
        let d = defaults();
        let pick = |path: &[&str]| at(&job, path).or_else(|| at(&d, path)).cloned().unwrap_or(Value::Null);
        let numbers = NUMBERS
            .iter()
            .map(|n| match at(&job, n.path).or_else(|| if n.optional { None } else { at(&d, n.path) }).and_then(Value::as_f64) {
                Some(x) => shown(x),
                None => String::new(),
            })
            .collect();
        let index = |v: Value, values: &[&str]| v.as_str().and_then(|s| values.iter().position(|x| *x == s)).unwrap_or(0);
        let design = job["part"]["design"].as_str().unwrap_or("");
        let design = match dir {
            Some(dir) if !design.is_empty() && std::path::Path::new(design).is_relative() => dir.join(design).to_string_lossy().into_owned(),
            _ => design.to_string(),
        };
        let threads = job["part"]["threads"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|t| ThreadRow {
                designation: t["designation"].as_str().unwrap_or("").to_string(),
                from: t["z_from"].as_f64().map(shown).unwrap_or_default(),
                to: t["z_to"].as_f64().map(shown).unwrap_or_default(),
            })
            .collect();
        LathePanel {
            material: index(pick(&["stock", "material"]), MATERIALS),
            coolant: index(pick(&["post", "coolant"]), COOLANTS),
            front: index(pick(&["part", "front"]), FRONTS),
            body: job["part"]["body"].as_str().unwrap_or("").to_string(),
            design,
            threads,
            numbers,
            base: job,
            result: None,
        }
    }

    /// The job the form describes, or what is wrong with it.
    pub fn job(&self) -> Result<Value, String> {
        let mut job = self.base.clone();
        if !job.is_object() {
            job = json!({});
        }
        for (n, text) in NUMBERS.iter().zip(&self.numbers) {
            let text = text.trim();
            if text.is_empty() && n.optional {
                if let Some((last, head)) = n.path.split_last()
                    && let Some(Value::Object(o)) = head.iter().try_fold(&mut job, |v, k| v.get_mut(*k))
                {
                    o.remove(*last);
                }
                continue;
            }
            let x: f64 = text.replace(',', ".").parse().map_err(|_| format("cam_panel.not_a_number", &[("field", tr(n.label))]))?;
            set(&mut job, n.path, json!(x));
        }
        set(&mut job, &["stock", "material"], json!(MATERIALS.get(self.material).copied().unwrap_or("steel")));
        set(&mut job, &["post", "coolant"], json!(COOLANTS.get(self.coolant).copied().unwrap_or("off")));
        if self.body.trim().is_empty() {
            return Err(tr("cam_panel.no_part").to_string());
        }
        let mut part = job["part"].as_object().cloned().unwrap_or_default();
        part.insert("design".into(), json!(self.design));
        part.insert("body".into(), json!(self.body.trim()));
        part.insert("front".into(), json!(FRONTS.get(self.front).copied().unwrap_or("+z")));
        let mut threads = Vec::new();
        for t in &self.threads {
            let z = |s: &str, label: &str| -> Result<f64, String> {
                s.trim().replace(',', ".").parse().map_err(|_| format("cam_panel.not_a_number", &[("field", tr(label))]))
            };
            let mut o = Map::new();
            o.insert("designation".into(), json!(t.designation.trim()));
            o.insert("z_from".into(), json!(z(&t.from, "cam_panel.thread_from")?));
            o.insert("z_to".into(), json!(z(&t.to, "cam_panel.thread_to")?));
            threads.push(Value::Object(o));
        }
        part.insert("threads".into(), Value::Array(threads));
        job["part"] = Value::Object(part);
        Ok(job)
    }

    /// The part as the form names it.
    fn part_name(&self) -> String {
        if self.design.is_empty() {
            format("cam_panel.in_open_design", &[("body", &self.body)])
        } else {
            let file = std::path::Path::new(&self.design).file_name().map_or(self.design.clone(), |f| f.to_string_lossy().into_owned());
            if self.body.is_empty() { file } else { format!("{file} · {}", self.body) }
        }
    }
}

/// What the lathe form asked for, done after the window is drawn.
pub enum LatheAsk {
    ChooseFile,
    UseOpen,
}

/// Draw the lathe job's form.
pub fn form(ui: &mut egui::Ui, d: &mut LathePanel) -> Option<LatheAsk> {
    let t = Tokens::get();
    let mut ask = None;
    ui.label(RichText::new(tr("cam_panel.bar")).strong());
    for (n, v) in NUMBERS.iter().zip(d.numbers.iter_mut()).take(BAR_NUMBERS) {
        if n.optional {
            ui_kit::hint_row(ui, tr(n.label), v, tr("cam_panel.automatic"));
        } else {
            ui_kit::expr_row(ui, tr(n.label), v);
        }
    }
    let materials: Vec<&str> = MATERIAL_LABELS.iter().map(|l| tr(l)).collect();
    ui_kit::choice_row(ui, tr("cam_panel.material"), "sc_cam_lathe_material", &mut d.material, &materials);

    ui.separator();
    ui.label(RichText::new(tr("cam_panel.part")).strong());
    ui.label(RichText::new(d.part_name()).color(t.text));
    ui_kit::text_row(ui, tr("cam_panel.body"), &mut d.body, "");
    let fronts: Vec<&str> = FRONT_LABELS.iter().map(|l| tr(l)).collect();
    ui_kit::choice_row(ui, tr("cam_panel.front"), "sc_cam_lathe_front", &mut d.front, &fronts);
    ui.horizontal_wrapped(|ui| {
        if ui_kit::button(ui, tr("cam_panel.choose_file"), ui_kit::Button::Normal).clicked() {
            ask = Some(LatheAsk::ChooseFile);
        }
        if ui_kit::button(ui, tr("cam_panel.use_open"), ui_kit::Button::Normal).clicked() {
            ask = Some(LatheAsk::UseOpen);
        }
    });

    ui.separator();
    ui.label(RichText::new(tr("cam_panel.threads")).strong());
    let mut remove = None;
    for (i, th) in d.threads.iter_mut().enumerate() {
        ui_kit::text_row(ui, tr("cam_panel.thread_size"), &mut th.designation, "M12");
        ui_kit::expr_row(ui, tr("cam_panel.thread_from"), &mut th.from);
        ui_kit::expr_row(ui, tr("cam_panel.thread_to"), &mut th.to);
        if ui_kit::button(ui, tr("cam_panel.remove"), ui_kit::Button::Normal).clicked() {
            remove = Some(i);
        }
    }
    if let Some(i) = remove
        && i < d.threads.len()
    {
        d.threads.remove(i);
    }
    if ui_kit::button(ui, tr("cam_panel.add_thread"), ui_kit::Button::Normal).clicked() {
        d.threads.push(ThreadRow { designation: "M12".into(), ..ThreadRow::default() });
    }

    ui.separator();
    ui.label(RichText::new(tr("cam_panel.strategy")).strong());
    for (n, v) in NUMBERS.iter().zip(d.numbers.iter_mut()).skip(BAR_NUMBERS) {
        ui_kit::expr_row(ui, tr(n.label), v);
    }
    let coolants: Vec<&str> = COOLANT_LABELS.iter().map(|l| tr(l)).collect();
    ui_kit::choice_row(ui, tr("cam_panel.coolant"), "sc_cam_lathe_coolant", &mut d.coolant, &coolants);
    ask
}

/// The simulation's findings: label and value.
fn report_lines(scene: &Value) -> Vec<(String, String)> {
    let n = |v: &Value| v.as_array().map_or(0, Vec::len).to_string();
    let sim = &scene["simulation"];
    let profile = if scene["part"]["exact"] == json!(true) { "cam_panel.profile_exact" } else { "cam_panel.profile_mesh" };
    vec![
        (tr("cam_panel.profile").to_string(), tr(profile).to_string()),
        (tr("cam_panel.gouge").to_string(), n(&sim["gouges"])),
        (tr("cam_panel.left").to_string(), n(&sim["left"])),
        (tr("cam_panel.rapid_hits").to_string(), n(&sim["rapid_hits"])),
        (tr("cam_panel.jaw_hits").to_string(), sim["jaw_hits"].as_u64().unwrap_or(0).to_string()),
        (tr("cam_panel.holder_hits").to_string(), n(&sim["holder_hits"])),
    ]
}

/// Draw the last simulation: its findings, the operations, the warnings and the half section.
pub fn result(ui: &mut egui::Ui, d: &mut LathePanel, running: bool) {
    let t = Tokens::get();
    let Some(s) = d.result.as_mut() else {
        if !running {
            ui_kit::status_line(ui, tr("cam_panel.press_simulate_lathe"), StatusTone::Dim);
        }
        return;
    };
    ui.label(RichText::new(tr("cam_panel.result")).strong());
    egui::Grid::new("sc_cam_lathe_report").num_columns(4).spacing([12.0, 4.0]).show(ui, |ui| {
        for (i, (label, value)) in report_lines(&s.scene).into_iter().enumerate() {
            ui.add(egui::Label::new(RichText::new(label).color(t.text_dim)).extend());
            ui.add(egui::Label::new(value).extend());
            if i % 2 == 1 {
                ui.end_row();
            }
        }
    });
    if let Some(img) = s.picture.take() {
        s.texture = Some(ui.ctx().load_texture("sc-cam-lathe-simulation", img, egui::TextureOptions::LINEAR));
    }
    if let Some(tex) = &s.texture {
        let size = tex.size_vec2();
        let k = (ui.available_width() / size.x.max(1.0)).min(PICTURE_HEIGHT / size.y.max(1.0)).min(1.0);
        ui.image((tex.id(), size * k));
    }
    ui.label(RichText::new(tr("cam_panel.operations")).strong());
    for op in s.scene["operations"].as_array().into_iter().flatten() {
        ui.label(format!("T{} {}", op["tool"].as_u64().unwrap_or(0), op["name"].as_str().unwrap_or("")));
    }
    for w in s.scene["warnings"].as_array().into_iter().flatten().filter_map(Value::as_str) {
        ui_kit::status_line(ui, w, StatusTone::Warning);
    }
}

#[cfg(test)]
#[path = "cam_lathe_tests.rs"]
mod tests;
