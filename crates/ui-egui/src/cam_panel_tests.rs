use serde_json::json;

use super::{CamPanel, PartRow, tick};
use crate::scenario::Harness;

/// A 40 × 30 × 10 plate named Plate in the open design.
fn plate(h: &mut Harness) {
    h.app.run("solid.box", json!({"length": 40, "width": 30, "height": 10, "corner": [0, 0, 0], "body_name": "Plate"})).unwrap();
}

/// Simulate Job opens the window with the open design's bodies as the parts, on the job's
/// defaults: a 400 × 300 × 10 sheet, work zero in the top's middle, 0.5 on the walls, a 0.5
/// skin cut 0.1 at a time.
#[test]
fn simulate_job_opens_the_window_with_the_open_designs_bodies() {
    let mut h = Harness::new();
    plate(&mut h);
    h.step(&json!({"start": "cam.job.simulate"})).unwrap();
    h.frames(2);
    let d = h.app.cam_job.as_ref().expect("ui.start opens the window");
    assert_eq!(d.parts.len(), 1);
    assert_eq!((d.parts[0].design.as_str(), d.parts[0].body.as_str(), d.parts[0].quantity.as_str()), ("", "Plate", "1"));
    assert!(h.widget("Plate (open design)").is_some());
    assert!(h.widget("Simulate").is_some() && h.widget("Write G-code…").is_some() && h.widget("Export zip…").is_some());
    let job = d.job().unwrap();
    assert_eq!(job["stock"]["size"], json!([400.0, 300.0]));
    assert_eq!(job["stock"]["thickness"], json!(10.0));
    assert_eq!(job["stock"]["origin"], json!("top_center"));
    assert_eq!(job["strategy"]["stock_to_leave"], json!(0.5));
    assert_eq!((&job["strategy"]["cut_out"]["skin"], &job["strategy"]["cut_out"]["skin_stepdown"]), (&json!(0.5), &json!(0.1)));
    assert!(job["strategy"].get("finish_stepdown").is_none(), "one pass: left to the default");
    solidcraft_cam::job::Job::from_json(&job.to_string()).expect("a job the CAM reads");
}

/// Simulate plans and simulates the job beside the window, then shows the report: the plate on
/// a 100 × 80 sheet is cut free, nothing left on it or cut into it, and the machined sheet's
/// picture.
#[test]
fn simulate_shows_the_report_and_the_picture() {
    let mut h = Harness::new();
    plate(&mut h);
    h.step(&json!({"start": "cam.job.simulate"})).unwrap();
    h.frames(1);
    let d = h.app.cam_job.as_mut().unwrap();
    d.numbers[0] = "100".into();
    d.numbers[1] = "80".into();
    d.cell = "0.5".into();
    h.frames(1);
    h.step(&json!({"widget": "Simulate"})).unwrap();
    h.frames(1);
    tick(h.app.cam_job.as_mut().unwrap(), true);
    h.frames(3);
    let d = h.app.cam_job.as_ref().unwrap();
    assert!(d.message.is_none(), "{:?}", d.message);
    let s = d.result.as_ref().expect("a report");
    assert!(s.report["feed_minutes"].as_f64().unwrap() > 0.0);
    assert_eq!(s.report["max_gouge"], json!(0.0));
    assert_eq!(s.report["parts"][0]["free"], json!(true));
    assert!(h.widget("Parts cut free").is_some() && h.widget("1 of 1").is_some());
    assert!(s.texture.is_some(), "the picture is shown");
}

/// The form reads a job file's settings, its designs made absolute beside the file, and writes
/// back what is typed; an empty finishing depth is the default (one pass), a number takes the
/// walls level by level; settings without a field are kept; a word where a number goes is
/// refused, naming the field.
#[test]
fn the_form_reads_and_writes_the_job() {
    let job = json!({
        "stock": {"size": [500, 250], "thickness": 6, "origin": "top_front_left"},
        "strategy": {"finish_stepdown": 2, "feeds": {"trim_air": false}},
        "machine": {"rack": [1, 2, 3]},
        "parts": [{"design": "a/b.step", "body": "B", "quantity": 3, "orient": "-z"}]
    });
    let dir = std::path::Path::new("/jobs");
    let mut d = CamPanel::from_job(job, Some(dir));
    assert_eq!(d.numbers[..3], ["500", "250", "6"]);
    assert_eq!(d.parts[0].design, dir.join("a/b.step").to_string_lossy());
    assert_eq!((d.parts[0].quantity.as_str(), d.parts[0].orient), ("3", 2));
    let out = d.job().unwrap();
    assert_eq!(out["stock"]["origin"], json!("top_front_left"));
    assert_eq!(out["strategy"]["finish_stepdown"], json!(2.0));
    assert_eq!(out["strategy"]["feeds"]["trim_air"], json!(false));
    assert_eq!(out["machine"]["rack"], json!([1, 2, 3]), "kept");
    assert_eq!(out["parts"][0]["orient"], json!("-z"));
    // the finishing depth is the 4th of the strategy's numbers, after the sheet's six
    d.numbers[6 + 3] = String::new();
    assert!(d.job().unwrap()["strategy"].get("finish_stepdown").is_none());
    d.numbers[2] = "ten".into();
    let e = d.job().unwrap_err();
    assert!(e.contains("Thickness"), "{e}");
    d.numbers[2] = "10".into();
    d.parts = vec![PartRow { quantity: "two".into(), ..PartRow::default() }];
    assert!(d.job().unwrap_err().contains("Quantity"));
    d.parts.clear();
    assert!(d.job().is_err(), "no parts");
}
