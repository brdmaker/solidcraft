//! The CAM Job window: a router job's sheet, parts and main settings as a form, simulated and
//! written out with `cam.job.simulate`, `cam.job.gcode` and `cam.job.export_scene`; or, switched
//! to the lathe, a lathe job (`cam_lathe.rs`) with `cam.lathe.plan`, `cam.lathe.gcode` and
//! `cam.lathe.export_scene`. Open job…
//! and Save job… read and write the job file; parts come from design or STEP files, or from the
//! open design's bodies. Simulate shows the machined sheet and the simulation's report. Planning
//! and simulating a sheet takes from seconds to minutes, so each command runs on a copy of the
//! session beside the window (in the browser, where no thread can start, here). The command
//! search's Simulate Job (`ui.start` of `cam.job.simulate`) opens it on the router, Simulate Lathe
//! Job (`cam.lathe.plan`) on the lathe.

use egui::RichText;
use serde_json::{Map, Value, json};

use crate::SolveApp;
use crate::cam_lathe::{self, LatheAsk, LathePanel};
use crate::i18n::{format, tr};
use crate::theme::Tokens;
use crate::ui_kit::{self, StatusTone};

/// A number of the job: where it lives (keys from the job's root), its label's message id, and
/// whether it may be left empty (the job's default then).
pub(crate) struct Num {
    pub(crate) path: &'static [&'static str],
    pub(crate) label: &'static str,
    pub(crate) optional: bool,
}

const fn num(path: &'static [&'static str], label: &'static str) -> Num {
    Num { path, label, optional: false }
}

/// The sheet's numbers.
const STOCK: &[Num] = &[
    num(&["stock", "size", "0"], "cam_panel.sheet_width"),
    num(&["stock", "size", "1"], "cam_panel.sheet_depth"),
    num(&["stock", "thickness"], "cam_panel.thickness"),
    num(&["stock", "spoilboard_allowance"], "cam_panel.spoilboard_allowance"),
    num(&["nest", "margin"], "cam_panel.margin"),
    num(&["nest", "spacing"], "cam_panel.spacing"),
];

/// The strategy's numbers.
const STRATEGY: &[Num] = &[
    num(&["strategy", "stock_to_leave"], "cam_panel.stock_to_leave"),
    num(&["strategy", "floor_stock_to_leave"], "cam_panel.floor_stock_to_leave"),
    num(&["strategy", "top_stock_to_leave"], "cam_panel.top_stock_to_leave"),
    Num { path: &["strategy", "finish_stepdown"], label: "cam_panel.finish_stepdown", optional: true },
    num(&["strategy", "cut_out", "skin"], "cam_panel.skin"),
    num(&["strategy", "cut_out", "skin_stepdown"], "cam_panel.skin_stepdown"),
    num(&["strategy", "cut_out", "final_skin"], "cam_panel.final_skin"),
    num(&["strategy", "clearance"], "cam_panel.clearance"),
    num(&["strategy", "ramp_angle_deg"], "cam_panel.ramp_angle"),
];

/// The feeds' numbers.
const FEEDS: &[Num] = &[
    num(&["strategy", "feeds", "max_thinning"], "cam_panel.max_thinning"),
    num(&["strategy", "feeds", "link"], "cam_panel.link_feed"),
    num(&["strategy", "rapid_feed"], "cam_panel.rapid_feed"),
];

/// A choice of the job: where it lives, its label, its values and their labels' message ids.
struct Choice {
    path: &'static [&'static str],
    label: &'static str,
    values: &'static [&'static str],
    labels: &'static [&'static str],
}

const CHOICES: &[Choice] = &[
    Choice {
        path: &["stock", "origin"],
        label: "cam_panel.work_zero",
        values: &["top_center", "top_front_left"],
        labels: &["cam_panel.zero_top_middle", "cam_panel.zero_top_front_left"],
    },
    Choice {
        path: &["strategy", "roughing", "pattern"],
        label: "cam_panel.roughing",
        values: &["adaptive", "offset"],
        labels: &["cam_panel.roughing_adaptive", "cam_panel.roughing_offset"],
    },
    Choice {
        path: &["strategy", "cut_out", "method"],
        label: "cam_panel.cut_out",
        values: &["skin", "profile"],
        labels: &["cam_panel.cut_out_skin", "cam_panel.cut_out_tabs"],
    },
    Choice {
        path: &["post", "coolant"],
        label: "cam_panel.coolant",
        values: &["off", "mist", "flood"],
        labels: &["cam_panel.coolant_off", "cam_panel.coolant_mist", "cam_panel.coolant_flood"],
    },
];

/// The job's switches: where each lives and its label.
const SWITCHES: &[(&[&str], &str)] = &[
    (&["nest", "rotate"], "cam_panel.rotate"),
    (&["strategy", "feeds", "chip_thinning"], "cam_panel.chip_thinning"),
    (&["strategy", "feeds", "trim_air"], "cam_panel.trim_air"),
    (&["post", "fit_arcs"], "cam_panel.fit_arcs"),
];

/// Most height the machined sheet's picture takes, points.
const PICTURE_HEIGHT: f32 = 200.0;

/// Which way up a part is machined: the job's values, and their labels' message ids.
const ORIENTS: &[&str] = &["auto", "+z", "-z", "+x", "-x", "+y", "-y"];
const ORIENT_LABELS: &[&str] =
    &["cam_panel.up_auto", "cam_panel.up_pz", "cam_panel.up_mz", "cam_panel.up_px", "cam_panel.up_mx", "cam_panel.up_py", "cam_panel.up_my"];

/// One part of the job as edited.
#[derive(Clone, Debug, Default)]
pub struct PartRow {
    /// Its design or STEP file (absolute), empty for the open design.
    pub design: String,
    pub body: String,
    pub quantity: String,
    /// Index into [`ORIENTS`].
    pub orient: usize,
}

/// A command running on a copy of the session.
struct Running {
    command: &'static str,
    done: std::sync::mpsc::Receiver<Result<Value, String>>,
}

/// What the last simulation found.
pub struct Simulated {
    pub report: Value,
    pub warnings: Vec<String>,
    texture: Option<egui::TextureHandle>,
    picture: Option<egui::ColorImage>,
}

/// The machines the window plans for: the router's and the lathe's message ids.
const MACHINES: &[&str] = &["cam_panel.router", "cam_panel.lathe"];

/// The window's state.
pub struct CamPanel {
    /// Which of [`MACHINES`] the job is for: 0 the router, 1 the lathe.
    pub mode: usize,
    /// The lathe job.
    pub lathe: LathePanel,
    /// The job as last read or written: settings the form has no field for are kept from it.
    pub base: Value,
    /// The numbers as typed, one for each of [`STOCK`], [`STRATEGY`] and [`FEEDS`].
    pub numbers: Vec<String>,
    /// The choices, one index for each of [`CHOICES`].
    pub choices: Vec<usize>,
    /// The switches, one for each of [`SWITCHES`].
    pub switches: Vec<bool>,
    pub parts: Vec<PartRow>,
    /// The simulation's cell, mm.
    pub cell: String,
    running: Option<Running>,
    pub result: Option<Simulated>,
    /// The last outcome to show: a refusal, or what a write did.
    pub message: Option<(String, bool)>,
}

/// The job's value at `path` (array indices as numbers).
pub(crate) fn at<'a>(v: &'a Value, path: &[&str]) -> Option<&'a Value> {
    path.iter().try_fold(v, |v, k| match k.parse::<usize>() {
        Ok(i) => v.get(i),
        Err(_) => v.get(*k),
    })
}

/// Set the job's value at `path`, making the objects (and array) on the way.
pub(crate) fn set(v: &mut Value, path: &[&str], value: Value) {
    let Some((last, head)) = path.split_last() else { return };
    let mut cur = v;
    for (i, k) in head.iter().enumerate() {
        let next_is_index = path.get(i + 1).is_some_and(|n| n.parse::<usize>().is_ok());
        if !cur.get(*k).is_some_and(|c| c.is_object() || c.is_array()) {
            cur[*k] = if next_is_index { json!([]) } else { json!({}) };
        }
        let Some(next) = cur.get_mut(*k) else { return };
        cur = next;
    }
    match last.parse::<usize>() {
        Ok(i) => {
            if let Some(a) = cur.as_array_mut() {
                while a.len() <= i {
                    a.push(json!(0.0));
                }
                if let Some(slot) = a.get_mut(i) {
                    *slot = value;
                }
            }
        }
        Err(_) => cur[*last] = value,
    }
}

/// A number as the form shows it: no trailing zeros.
pub(crate) fn shown(x: f64) -> String {
    let s = format!("{x:.4}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// The defaults the job takes, for the fields a job file leaves out.
fn defaults() -> Value {
    match solidcraft_cam::job::Job::from_json(r#"{"stock": {"size": [400, 300], "thickness": 10}, "parts": [{"body": "-"}]}"#) {
        Ok(j) => serde_json::to_value(&j).unwrap_or(Value::Null),
        Err(_) => Value::Null,
    }
}

impl CamPanel {
    /// The form for `job` (designs relative to `dir`, made absolute).
    pub fn from_job(job: Value, dir: Option<&std::path::Path>) -> CamPanel {
        let d = defaults();
        let pick = |path: &[&str]| at(&job, path).or_else(|| at(&d, path)).cloned().unwrap_or(Value::Null);
        let numbers = STOCK
            .iter()
            .chain(STRATEGY)
            .chain(FEEDS)
            .map(|n| match pick(n.path).as_f64() {
                Some(x) => shown(x),
                None => String::new(),
            })
            .collect();
        let choices = CHOICES.iter().map(|c| pick(c.path).as_str().and_then(|s| c.values.iter().position(|v| *v == s)).unwrap_or(0)).collect();
        let switches = SWITCHES.iter().map(|(p, _)| pick(p).as_bool().unwrap_or(false)).collect();
        let parts = job["parts"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|p| {
                let design = p["design"].as_str().unwrap_or("");
                let design = match dir {
                    Some(dir) if !design.is_empty() && std::path::Path::new(design).is_relative() => dir.join(design).to_string_lossy().into_owned(),
                    _ => design.to_string(),
                };
                PartRow {
                    design,
                    body: p["body"].as_str().unwrap_or("").to_string(),
                    quantity: p["quantity"].as_u64().unwrap_or(1).to_string(),
                    orient: p["orient"].as_str().and_then(|o| ORIENTS.iter().position(|x| *x == o)).unwrap_or(0),
                }
            })
            .collect();
        CamPanel {
            mode: 0,
            lathe: LathePanel::from_job(json!({"stock": {"diameter": 30}, "part": {"body": ""}}), None),
            base: job,
            numbers,
            choices,
            switches,
            parts,
            cell: "0.25".into(),
            running: None,
            result: None,
            message: None,
        }
    }

    /// The job the form describes, or what is wrong with it.
    pub fn job(&self) -> Result<Value, String> {
        let mut job = self.base.clone();
        if !job.is_object() {
            job = json!({});
        }
        for (n, text) in STOCK.iter().chain(STRATEGY).chain(FEEDS).zip(&self.numbers) {
            let text = text.trim();
            if text.is_empty() && n.optional {
                if let Some((last, head)) = n.path.split_last()
                    && let Some(Value::Object(o)) = at_mut(&mut job, head)
                {
                    o.remove(*last);
                }
                continue;
            }
            let x: f64 = text.replace(',', ".").parse().map_err(|_| format("cam_panel.not_a_number", &[("field", tr(n.label))]))?;
            set(&mut job, n.path, json!(x));
        }
        for (c, i) in CHOICES.iter().zip(&self.choices) {
            set(&mut job, c.path, json!(c.values.get(*i).copied().unwrap_or("")));
        }
        for ((p, _), on) in SWITCHES.iter().zip(&self.switches) {
            set(&mut job, p, json!(on));
        }
        let mut parts = Vec::new();
        for p in &self.parts {
            let q: u32 = p.quantity.trim().parse().map_err(|_| format("cam_panel.not_a_number", &[("field", tr("cam_panel.quantity"))]))?;
            let mut o = Map::new();
            o.insert("design".into(), json!(p.design));
            o.insert("body".into(), json!(p.body));
            o.insert("quantity".into(), json!(q));
            o.insert("orient".into(), json!(ORIENTS.get(p.orient).copied().unwrap_or("auto")));
            parts.push(Value::Object(o));
        }
        if parts.is_empty() {
            return Err(tr("cam_panel.no_parts").to_string());
        }
        job["parts"] = Value::Array(parts);
        Ok(job)
    }

    /// The job of the machine the window is on.
    pub fn current_job(&self) -> Result<Value, String> {
        if self.mode == 1 { self.lathe.job() } else { self.job() }
    }

    pub fn busy(&self) -> bool {
        self.running.is_some()
    }
}

fn at_mut<'a>(v: &'a mut Value, path: &[&str]) -> Option<&'a mut Value> {
    path.iter().try_fold(v, |v, k| v.get_mut(*k))
}

/// Open the window: an empty job on a 400 × 300 × 10 sheet with the built-in tools, its parts
/// the open design's bodies.
pub fn open(app: &mut SolveApp) {
    if app.cam_job.is_some() {
        return;
    }
    let mut panel = CamPanel::from_job(json!({"stock": {"size": [400, 300], "thickness": 10}, "parts": []}), None);
    add_open_design(app, &mut panel);
    use_open_body(app, &mut panel.lathe);
    app.cam_job = Some(panel);
}

/// Open the window on the lathe: a Ø30 bar of steel with the Emco Compact 5's tools, the part
/// the open design's first body.
pub fn open_lathe(app: &mut SolveApp) {
    open(app);
    if let Some(panel) = app.cam_job.as_mut() {
        panel.mode = 1;
    }
}

/// The lathe job's part: the open design's first body.
fn use_open_body(app: &SolveApp, lathe: &mut LathePanel) {
    let Ok(world) = app.session.world_state() else { return };
    if let Some(b) = world.bodies.first() {
        lathe.design.clear();
        lathe.body.clone_from(&b.name);
    }
}

/// Add each body of the open design as a part.
fn add_open_design(app: &SolveApp, panel: &mut CamPanel) {
    let Ok(world) = app.session.world_state() else { return };
    for b in &world.bodies {
        if !panel.parts.iter().any(|p| p.design.is_empty() && p.body == b.name) {
            panel.parts.push(PartRow { design: String::new(), body: b.name.clone(), quantity: "1".into(), orient: 0 });
        }
    }
}

/// Where a simulation draws its picture for the window to read back.
fn picture_path() -> std::path::PathBuf {
    std::env::temp_dir().join(format!("solidcraft-cam-{}.png", std::process::id()))
}

/// Run `command` with `params` on a copy of the session beside the window; in the browser, or
/// when no thread can start, here.
fn start(app: &mut SolveApp, panel: &mut CamPanel, command: &'static str, params: Value) {
    panel.message = None;
    #[cfg(not(target_arch = "wasm32"))]
    {
        let mut copy = app.session.scratch();
        copy.path.clone_from(&app.session.path);
        let (tx, rx) = std::sync::mpsc::channel();
        let given = params.clone();
        let job = std::thread::Builder::new().name("cam".into()).spawn(move || {
            let _ = tx.send(copy.execute(command, &given).map_err(|e| e.to_string()));
        });
        if job.is_ok() {
            panel.running = Some(Running { command, done: rx });
            return;
        }
    }
    let r = app.session.execute(command, &params).map_err(|e| e.to_string());
    finish(panel, command, r);
}

/// Take up a finished command (`wait`: wait for a running one).
pub fn tick(panel: &mut CamPanel, wait: bool) {
    let Some(running) = panel.running.as_ref() else { return };
    let outcome = if wait {
        running.done.recv().ok()
    } else {
        match running.done.try_recv() {
            Ok(r) => Some(r),
            Err(std::sync::mpsc::TryRecvError::Empty) => return,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => None,
        }
    };
    let Some(Running { command, .. }) = panel.running.take() else { return };
    finish(panel, command, outcome.unwrap_or_else(|| Err(format("cam_panel.stopped", &[("command", command)]))));
}

fn finish(panel: &mut CamPanel, command: &str, r: Result<Value, String>) {
    match r {
        Err(e) => panel.message = Some((e, true)),
        Ok(v) if command == "cam.lathe.plan" => {
            panel.lathe.result = Some(cam_lathe::LatheSimulated { scene: v["scene"].clone(), texture: None, picture: read_picture(&v) });
        }
        Ok(v) if command == "cam.job.simulate" => {
            let picture = read_picture(&v);
            let warnings = v["warnings"].as_array().into_iter().flatten().filter_map(|w| w.as_str().map(str::to_string)).collect();
            panel.result = Some(Simulated { report: v["report"].clone(), warnings, texture: None, picture });
        }
        Ok(v) => {
            let path = v["path"].as_str().unwrap_or("");
            panel.message = Some((format("cam_panel.written", &[("path", path)]), false));
        }
    }
}

/// The picture a simulation wrote (its `image`).
fn read_picture(v: &Value) -> Option<egui::ColorImage> {
    let bytes = solidcraft_engine::io::vfs::read(v["image"].as_str()?, 64 << 20).ok()?;
    let img = image::load_from_memory(&bytes).ok()?.to_rgba8();
    let size = [img.width() as usize, img.height() as usize];
    Some(egui::ColorImage::from_rgba_unmultiplied(size, img.as_raw()))
}

/// The report's lines: label and value.
fn report_lines(r: &Value) -> Vec<(String, String)> {
    let mm = |x: &Value| format!("{} mm", shown(x.as_f64().unwrap_or(0.0)));
    let min = |x: &Value| format("cam_panel.minutes", &[("minutes", &format!("{:.1}", x.as_f64().unwrap_or(0.0)))]);
    let parts = r["parts"].as_array().cloned().unwrap_or_default();
    let free = parts.iter().filter(|p| p["free"] == true).count();
    vec![
        (tr("cam_panel.feed_time").to_string(), min(&r["feed_minutes"])),
        (tr("cam_panel.air_time").to_string(), min(&r["air_minutes"])),
        (tr("cam_panel.left").to_string(), mm(&r["max_left"])),
        (tr("cam_panel.gouge").to_string(), mm(&r["max_gouge"])),
        (tr("cam_panel.rapid_hits").to_string(), r["rapid_hits"].as_array().map_or(0, Vec::len).to_string()),
        (tr("cam_panel.free").to_string(), format("cam_panel.of", &[("done", &free.to_string()), ("all", &parts.len().to_string())])),
    ]
}

/// What a button asked for, done after the window is drawn.
enum Ask {
    Open,
    Save,
    AddFile,
    AddOpen,
    Simulate,
    Gcode,
    Zip,
    Lathe(LatheAsk),
}

/// Draw the window when it is open. Called once per frame.
pub fn show(app: &mut SolveApp, ctx: &egui::Context) {
    let Some(mut d) = app.cam_job.take() else { return };
    tick(&mut d, false);
    if d.busy() {
        ctx.request_repaint_after(std::time::Duration::from_millis(100));
    }
    let t = Tokens::get();
    let mut ask = None;
    let mut remove = None;
    let title = tr("cam_panel.title").to_string();
    let closed = ui_kit::Window::new(egui::Id::unique("sc_cam_job"), "settings", &title).cards(5.0).page_rows(24.0).show_body(ctx, |ui| {
        // the actions and what they did, above the form so they stay in view
        ui.horizontal_wrapped(|ui| {
            ui.add_enabled_ui(!d.busy(), |ui| {
                if ui_kit::button(ui, tr("cam_panel.simulate"), ui_kit::Button::Primary).clicked() {
                    ask = Some(Ask::Simulate);
                }
                if ui_kit::button(ui, tr("cam_panel.write_gcode"), ui_kit::Button::Normal).clicked() {
                    ask = Some(Ask::Gcode);
                }
                if ui_kit::button(ui, tr("cam_panel.export_zip"), ui_kit::Button::Normal).clicked() {
                    ask = Some(Ask::Zip);
                }
            });
            if ui_kit::button(ui, tr("cam_panel.open_job"), ui_kit::Button::Normal).clicked() {
                ask = Some(Ask::Open);
            }
            if ui_kit::button(ui, tr("cam_panel.save_job"), ui_kit::Button::Normal).clicked() {
                ask = Some(Ask::Save);
            }
        });
        if let Some(r) = &d.running {
            let what = if matches!(r.command, "cam.job.simulate" | "cam.lathe.plan") { "cam_panel.simulating" } else { "cam_panel.writing" };
            ui_kit::status_line(ui, tr(what), StatusTone::Dim);
        }
        if let Some((m, error)) = &d.message {
            ui_kit::status_line(ui, m, if *error { StatusTone::Error } else { StatusTone::Dim });
        }
        let machines: Vec<&str> = MACHINES.iter().map(|l| tr(l)).collect();
        ui_kit::choice_row(ui, tr("cam_panel.machine"), "sc_cam_machine", &mut d.mode, &machines);
        // the settings on the left, the simulation on the right
        ui.columns(2, |cols| {
            let [left, right] = cols else { return };
            if d.mode == 1 {
                let running = d.running.is_some();
                if let Some(a) = ui_kit::scope(left, |ui| cam_lathe::form(ui, &mut d.lathe)) {
                    ask = Some(Ask::Lathe(a));
                }
                cam_lathe::result(right, &mut d.lathe, running);
                return;
            }
            ui_kit::scope(left, |ui| {
                ui.label(RichText::new(tr("cam_panel.sheet")).strong());
                let mut k = 0;
                for n in STOCK {
                    if let Some(v) = d.numbers.get_mut(k) {
                        ui_kit::expr_row(ui, tr(n.label), v);
                    }
                    k += 1;
                }
                choice(ui, &mut d.choices, 0);
                switch(ui, &mut d.switches, 0);

                ui.separator();
                ui.label(RichText::new(tr("cam_panel.parts")).strong());
                let orients: Vec<&str> = ORIENT_LABELS.iter().map(|l| tr(l)).collect();
                for (i, p) in d.parts.iter_mut().enumerate() {
                    let name = if p.design.is_empty() {
                        format("cam_panel.in_open_design", &[("body", &p.body)])
                    } else {
                        let file = std::path::Path::new(&p.design).file_name().map_or(p.design.clone(), |f| f.to_string_lossy().into_owned());
                        if p.body.is_empty() { file } else { format!("{file} · {}", p.body) }
                    };
                    ui.label(RichText::new(name).color(t.text));
                    if !p.design.is_empty() {
                        ui_kit::text_row(ui, tr("cam_panel.body"), &mut p.body, "");
                    }
                    ui_kit::expr_row(ui, tr("cam_panel.quantity"), &mut p.quantity);
                    ui_kit::choice_row(ui, tr("cam_panel.way_up"), &format!("sc_cam_up_{i}"), &mut p.orient, &orients);
                    if ui_kit::button(ui, tr("cam_panel.remove"), ui_kit::Button::Normal).clicked() {
                        remove = Some(i);
                    }
                }
                ui.horizontal_wrapped(|ui| {
                    if ui_kit::button(ui, tr("cam_panel.add_file"), ui_kit::Button::Normal).clicked() {
                        ask = Some(Ask::AddFile);
                    }
                    if ui_kit::button(ui, tr("cam_panel.add_open"), ui_kit::Button::Normal).clicked() {
                        ask = Some(Ask::AddOpen);
                    }
                });

                ui.separator();
                ui.label(RichText::new(tr("cam_panel.strategy")).strong());
                choice(ui, &mut d.choices, 1);
                choice(ui, &mut d.choices, 2);
                for n in STRATEGY {
                    if let Some(v) = d.numbers.get_mut(k) {
                        if n.optional {
                            ui_kit::hint_row(ui, tr(n.label), v, tr("cam_panel.one_pass"));
                        } else {
                            ui_kit::expr_row(ui, tr(n.label), v);
                        }
                    }
                    k += 1;
                }

                ui.separator();
                ui.label(RichText::new(tr("cam_panel.feeds")).strong());
                switch(ui, &mut d.switches, 1);
                for n in FEEDS {
                    if let Some(v) = d.numbers.get_mut(k) {
                        ui_kit::expr_row(ui, tr(n.label), v);
                    }
                    k += 1;
                }
                switch(ui, &mut d.switches, 2);
                switch(ui, &mut d.switches, 3);
                choice(ui, &mut d.choices, 3);

                ui.separator();
                ui_kit::expr_row(ui, tr("cam_panel.cell"), &mut d.cell);
            });
            let ui = right;
            if d.result.is_none() && d.running.is_none() {
                ui_kit::status_line(ui, tr("cam_panel.press_simulate"), StatusTone::Dim);
            }
            if let Some(s) = d.result.as_mut() {
                ui.label(RichText::new(tr("cam_panel.result")).strong());
                // two to a row, labels whole
                egui::Grid::new("sc_cam_report").num_columns(4).spacing([12.0, 4.0]).show(ui, |ui| {
                    for (i, (label, value)) in report_lines(&s.report).into_iter().enumerate() {
                        ui.add(egui::Label::new(RichText::new(label).color(t.text_dim)).extend());
                        ui.add(egui::Label::new(value).extend());
                        if i % 2 == 1 {
                            ui.end_row();
                        }
                    }
                });
                for w in &s.warnings {
                    ui_kit::status_line(ui, w, StatusTone::Warning);
                }
                if let Some(img) = s.picture.take() {
                    s.texture = Some(ui.ctx().load_texture("sc-cam-simulation", img, egui::TextureOptions::LINEAR));
                }
                if let Some(tex) = &s.texture {
                    let size = tex.size_vec2();
                    // the whole sheet in view beside the form: as wide as the column, at most
                    // `PICTURE_HEIGHT` tall
                    let k = (ui.available_width() / size.x.max(1.0)).min(PICTURE_HEIGHT / size.y.max(1.0)).min(1.0);
                    ui.image((tex.id(), size * k));
                }
            }
        });
    });
    if let Some(i) = remove
        && i < d.parts.len()
    {
        d.parts.remove(i);
    }
    match ask {
        None => {}
        Some(Ask::AddOpen) => add_open_design(app, &mut d),
        Some(Ask::Lathe(LatheAsk::UseOpen)) => use_open_body(app, &mut d.lathe),
        Some(Ask::Lathe(LatheAsk::ChooseFile)) => {
            app.pick_open_for(&["sc", "step", "stp"], |app, path| {
                let (Some(path), Some(panel)) = (path, app.cam_job.as_mut()) else { return };
                panel.lathe.design = path;
                panel.lathe.body.clear();
            });
        }
        Some(Ask::Open) => {
            app.pick_open_for(&["json"], |app, path| {
                let (Some(path), Some(panel)) = (path, app.cam_job.as_mut()) else { return };
                match solidcraft_engine::io::vfs::read(&path, 16 << 20)
                    .map_err(|e| e.to_string())
                    .and_then(|b| serde_json::from_slice::<Value>(&b).map_err(|e| e.to_string()))
                {
                    Ok(job) if cam_lathe::is_lathe_job(&job) => {
                        let dir = std::path::Path::new(&path).parent().map(std::path::Path::to_path_buf);
                        panel.lathe = LathePanel::from_job(job, dir.as_deref());
                        panel.mode = 1;
                    }
                    Ok(job) => {
                        let dir = std::path::Path::new(&path).parent().map(std::path::Path::to_path_buf);
                        let mut fresh = CamPanel::from_job(job, dir.as_deref());
                        fresh.cell.clone_from(&panel.cell);
                        fresh.lathe = std::mem::replace(&mut panel.lathe, LathePanel::from_job(Value::Null, None));
                        *panel = fresh;
                    }
                    Err(e) => panel.message = Some((format!("{path}: {e}"), true)),
                }
            });
        }
        Some(Ask::Save) => match d.current_job() {
            Ok(job) => {
                app.pick_save("job.json", &["json"], move |app, path| {
                    let (Some(path), Some(panel)) = (path, app.cam_job.as_mut()) else { return };
                    let text = serde_json::to_string_pretty(&job).unwrap_or_default();
                    panel.message = Some(match solidcraft_engine::io::vfs::write(&path, text.as_bytes()) {
                        Ok(()) => {
                            if panel.mode == 1 {
                                panel.lathe.base = job;
                            } else {
                                panel.base = job;
                            }
                            (format("cam_panel.written", &[("path", &path)]), false)
                        }
                        Err(e) => (format!("{path}: {e}"), true),
                    });
                });
            }
            Err(e) => d.message = Some((e, true)),
        },
        Some(Ask::AddFile) => {
            app.pick_open_for(&["sc", "step", "stp"], |app, path| {
                let (Some(path), Some(panel)) = (path, app.cam_job.as_mut()) else { return };
                panel.parts.push(PartRow { design: path, body: String::new(), quantity: "1".into(), orient: 0 });
            });
        }
        Some(Ask::Simulate) if d.mode == 1 => match d.lathe.job() {
            Ok(job) => start(app, &mut d, "cam.lathe.plan", json!({"job": job, "image": picture_path().to_string_lossy()})),
            Err(e) => d.message = Some((e, true)),
        },
        Some(Ask::Simulate) => match (d.job(), d.cell.trim().replace(',', ".").parse::<f64>()) {
            (Ok(job), Ok(cell)) => {
                let params = json!({"job": job, "cell": cell, "image": picture_path().to_string_lossy()});
                start(app, &mut d, "cam.job.simulate", params);
            }
            (Err(e), _) => d.message = Some((e, true)),
            (_, Err(_)) => d.message = Some((format("cam_panel.not_a_number", &[("field", tr("cam_panel.cell"))]), true)),
        },
        Some(ask @ (Ask::Gcode | Ask::Zip)) => match d.current_job() {
            Ok(job) => {
                let (name, ext, command) = match (ask, d.mode == 1) {
                    (Ask::Gcode, false) => ("job.ngc", "ngc", "cam.job.gcode"),
                    (Ask::Gcode, true) => ("job.ngc", "ngc", "cam.lathe.gcode"),
                    (_, false) => ("job.zip", "zip", "cam.job.export_scene"),
                    (_, true) => ("job.zip", "zip", "cam.lathe.export_scene"),
                };
                app.pick_save(name, &[ext], move |app, path| {
                    let Some(path) = path else { return };
                    let Some(mut panel) = app.cam_job.take() else { return };
                    start(app, &mut panel, command, json!({"job": job, "path": path}));
                    app.cam_job = Some(panel);
                });
            }
            Err(e) => d.message = Some((e, true)),
        },
    }
    if !closed {
        app.cam_job = Some(d);
    }
}

/// The row of choice `i` of [`CHOICES`].
fn choice(ui: &mut egui::Ui, choices: &mut [usize], i: usize) {
    let (Some(c), Some(sel)) = (CHOICES.get(i), choices.get_mut(i)) else { return };
    let labels: Vec<&str> = c.labels.iter().map(|l| tr(l)).collect();
    ui_kit::choice_row(ui, tr(c.label), &format!("sc_cam_choice_{i}"), sel, &labels);
}

/// The checkbox of switch `i` of [`SWITCHES`].
fn switch(ui: &mut egui::Ui, switches: &mut [bool], i: usize) {
    let (Some((_, label)), Some(on)) = (SWITCHES.get(i), switches.get_mut(i)) else { return };
    ui_kit::checkbox(ui, tr(label), on);
}

#[cfg(test)]
#[path = "cam_panel_tests.rs"]
mod tests;
