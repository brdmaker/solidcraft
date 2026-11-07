use serde_json::json;

use super::{LathePanel, ThreadRow, is_lathe_job};
use crate::cam_panel::tick;
use crate::scenario::Harness;

/// The lathe test parts.
fn fixtures() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/cam/lathe")
}

/// Simulate Lathe Job opens the CAM Job window on the lathe, the part the open design's first
/// body, on the lathe job's defaults: a Ø30 steel bar, 20 in the chuck, 0.5 faced off, 3 left for
/// parting, 2 clear of the jaws, its length worked out (left empty), 0.2 and 0.1 left for
/// finishing.
#[test]
fn simulate_lathe_job_opens_the_window_on_the_lathe() {
    let mut h = Harness::new();
    h.app.run("solid.box", json!({"length": 10, "width": 10, "height": 10, "corner": [-5, -5, 0], "body_name": "Pin"})).unwrap();
    h.step(&json!({"start": "cam.lathe.plan"})).unwrap();
    h.frames(2);
    let d = h.app.cam_job.as_ref().expect("ui.start opens the window");
    assert_eq!(d.mode, 1);
    assert_eq!((d.lathe.design.as_str(), d.lathe.body.as_str()), ("", "Pin"));
    assert!(h.widget("Pin (open design)").is_some() && h.widget("Bar diameter").is_some() && h.widget("Add thread").is_some());
    let job = d.lathe.job().unwrap();
    assert_eq!(job["stock"]["diameter"], json!(30.0));
    assert_eq!(job["stock"]["material"], json!("steel"));
    assert_eq!((&job["stock"]["in_chuck"], &job["stock"]["face_allowance"]), (&json!(20.0), &json!(0.5)));
    assert_eq!((&job["stock"]["parting_allowance"], &job["stock"]["chuck_clearance"]), (&json!(3.0), &json!(2.0)));
    assert!(job["stock"].get("length").is_none(), "worked out from the part");
    assert_eq!((&job["strategy"]["leave_radial"], &job["strategy"]["leave_axial"]), (&json!(0.2), &json!(0.1)));
    assert_eq!(job["part"], json!({"design": "", "body": "Pin", "front": "+z", "threads": []}));
    solidcraft_cam::lathe::job::LatheJob::from_json(&job.to_string()).expect("a job the lathe CAM reads");
}

/// Simulate on the lathe plans and simulates the flanged bushing (Ø45 brass bar) beside the
/// window and shows what it found: read from its exact faces, nothing cut into it or left on it,
/// its six operations (faced, drilled, roughed, bored, finished, parted off) and the half
/// section's picture.
#[test]
fn simulate_on_the_lathe_shows_the_turned_part() {
    let mut h = Harness::new();
    h.step(&json!({"start": "cam.lathe.plan"})).unwrap();
    h.frames(1);
    let d = h.app.cam_job.as_mut().unwrap();
    d.lathe.design = fixtures().join("flanged-bushing.step").to_string_lossy().into_owned();
    d.lathe.body = "FlangedBushing".into();
    d.lathe.numbers[0] = "45".into();
    d.lathe.material = 1;
    h.frames(1);
    h.step(&json!({"widget": "Simulate"})).unwrap();
    h.frames(1);
    tick(h.app.cam_job.as_mut().unwrap(), true);
    h.frames(3);
    let d = h.app.cam_job.as_ref().unwrap();
    assert!(d.message.is_none(), "{:?}", d.message);
    let s = d.lathe.result.as_ref().expect("a simulation");
    let kinds: Vec<&str> = s.scene["operations"].as_array().unwrap().iter().map(|o| o["kind"].as_str().unwrap()).collect();
    assert_eq!(kinds, ["facing", "drilling", "roughing", "boring", "finishing", "parting"]);
    assert_eq!(s.scene["stock"]["material"], json!("brass"));
    assert!(s.scene["simulation"]["gouges"].as_array().unwrap().is_empty());
    assert!(h.widget("The exact faces").is_some() && h.widget("Operations").is_some() && h.widget("T5 drilling Ø8").is_some());
    assert!(s.texture.is_some(), "the half section is shown");
}

/// The lathe form reads a lathe job file (its part's design made absolute beside the file) and
/// writes back what is typed: the bar's length left empty is worked out; threads; settings
/// without a field kept; a word where a number goes refused, naming the field; no part refused.
#[test]
fn the_lathe_form_reads_and_writes_the_job() {
    let job = json!({
        "library": "my-tools.json",
        "stock": {"diameter": 32, "length": 90, "material": "aluminium"},
        "part": {"design": "parts/arbor.step", "body": "TaperArbor", "front": "-z", "threads": [{"designation": "M12", "z_from": 48, "z_to": 60}]},
        "post": {"coolant": "flood"}
    });
    assert!(is_lathe_job(&job));
    assert!(!is_lathe_job(&json!({"stock": {"size": [400, 300], "thickness": 10}, "parts": []})));
    let dir = std::path::Path::new("/jobs");
    let mut d = LathePanel::from_job(job, Some(dir));
    assert_eq!(d.numbers[..3], ["32", "90", "20"]);
    assert_eq!((d.material, d.coolant, d.front), (0, 2, 1));
    assert_eq!(d.design, dir.join("parts/arbor.step").to_string_lossy());
    assert_eq!((d.threads[0].designation.as_str(), d.threads[0].from.as_str(), d.threads[0].to.as_str()), ("M12", "48", "60"));
    let out = d.job().unwrap();
    assert_eq!(out["library"], json!("my-tools.json"), "kept");
    assert_eq!(out["stock"]["length"], json!(90.0));
    assert_eq!(out["part"]["threads"], json!([{"designation": "M12", "z_from": 48.0, "z_to": 60.0}]));
    assert_eq!((&out["part"]["front"], &out["post"]["coolant"]), (&json!("-z"), &json!("flood")));
    d.numbers[1] = String::new();
    assert!(d.job().unwrap()["stock"].get("length").is_none(), "worked out");
    d.threads.push(ThreadRow { designation: "M8".into(), from: "0".into(), to: "end".into() });
    assert!(d.job().unwrap_err().contains("To z"));
    d.threads.pop();
    d.numbers[0] = "wide".into();
    assert!(d.job().unwrap_err().contains("Bar diameter"));
    d.numbers[0] = "32".into();
    d.body.clear();
    assert!(d.job().is_err(), "no part");
}
