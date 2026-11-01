//! `cam.part.inspect` on the CAM test designs: the command reads the `.sc` body and returns the
//! features the design was made with (`tests/cam/features.json`). The feature-by-feature checks
//! live in `crates/cam/src/fixture_tests.rs`, on the parts' STEP files.

use std::path::{Path, PathBuf};

use serde_json::{Value, json};

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/cam")
}

/// How many features of each kind the design was made with.
fn counts(part: &Value) -> (usize, usize, f64) {
    let feats = part["features"].as_array().unwrap();
    let n = |kinds: &[&str]| feats.iter().filter(|f| kinds.contains(&f["kind"].as_str().unwrap())).count();
    (n(&["hole", "counterbore", "countersink"]), n(&["cutout"]), part["top_z"].as_f64().unwrap())
}

#[test]
fn inspect_reads_the_test_designs() {
    let expected: Value = serde_json::from_str(&std::fs::read_to_string(fixtures().join("features.json")).unwrap()).unwrap();
    for part in expected["parts"].as_array().unwrap() {
        let mut s = Session::default();
        run(&mut s, "doc.open", json!({"path": fixtures().join(part["design"].as_str().unwrap()).to_string_lossy()}));
        let got = run(&mut s, "cam.part.inspect", json!({"body": part["body"]}));
        let (holes, cutouts, top) = counts(part);
        assert_eq!(got["holes"].as_array().unwrap().len(), holes, "{}", part["design"]);
        assert_eq!(got["cutouts"].as_array().unwrap().len(), cutouts, "{}", part["design"]);
        assert!((got["top_z"].as_f64().unwrap() - top).abs() < 1e-6);
        assert!(got["unsupported"].as_array().unwrap().is_empty());
    }
}

#[test]
fn inspect_refuses_a_missing_body() {
    let mut s = Session::default();
    let err = s.execute("cam.part.inspect", &json!({"body": "Nothing"})).unwrap_err().to_string();
    assert!(err.contains("no body named `Nothing`"), "{err}");
    assert!(s.execute("cam.part.inspect", &json!({})).is_err());
}

/// The flange design is open; the job takes 3 copies of its body (empty `design`), 2 flanges
/// from the `.sc` file beside it and 2 panels from the panel's STEP file. Their boxes (90 × 90,
/// 200 × 100) fit a 600 × 400 sheet, so all 7 go on; the 3 mm panels leave 7 mm of the 10 mm
/// sheet to face off.
#[test]
fn nest_takes_parts_from_the_open_design_a_design_file_and_a_step_file() {
    let mut s = Session::default();
    run(&mut s, "doc.open", json!({"path": fixtures().join("nema23-flange.sc").to_string_lossy()}));
    let job = json!({
        "stock": {"size": [600, 400], "thickness": 10},
        "nest": {"margin": 10, "spacing": 8},
        "parts": [
            {"body": "Flange", "quantity": 3, "name": "open flange"},
            {"design": "nema23-flange.sc", "body": "Flange", "quantity": 2},
            {"design": "front-panel.step", "body": "Panel", "quantity": 2}
        ]
    });
    let r = run(&mut s, "cam.job.nest", json!({"job": job}));
    assert_eq!(r["nesting"]["placements"].as_array().unwrap().len(), 7, "{:#}", r["nesting"]["unplaced"]);
    assert_eq!(r["parts"][0]["label"], "open flange");
    assert_eq!(r["parts"][1]["label"], "Flange");
    let h = |i: usize| r["parts"][i]["height"].as_f64().unwrap();
    assert!((h(0) - 10.0).abs() < 1e-6 && (h(1) - 10.0).abs() < 1e-6 && (h(2) - 3.0).abs() < 1e-6);
    assert!((r["parts"][2]["face_off"].as_f64().unwrap() - 7.0).abs() < 1e-6);
    // Both flanges read the same outline: rounded 90 × 90 (R6), its corners drawn as chords
    // within AREA_TOL (at most 2/3 · tol · 2π · 6 of area).
    let flange = 8100.0 - (4.0 - std::f64::consts::PI) * 36.0;
    let chord = 2.0 / 3.0 * solidcraft_cam::part::AREA_TOL * 2.0 * std::f64::consts::PI * 6.0;
    for i in [0, 1] {
        assert!((r["parts"][i]["outline_area"].as_f64().unwrap() - flange).abs() <= chord, "{}", r["parts"][i]["outline_area"]);
    }
}

/// The same job from a file: its parts' designs are relative to the file.
#[test]
fn nest_reads_a_job_file() {
    let dir = std::env::temp_dir().join(format!("solidcraft-cam-job-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let design = fixtures().join("front-panel.sc");
    let job = json!({"stock": {"size": [300, 300], "thickness": 3}, "parts": [{"design": design.to_string_lossy(), "body": "Panel", "quantity": 3}]});
    let file = dir.join("panels.json");
    std::fs::write(&file, job.to_string()).unwrap();
    let mut s = Session::default();
    let r = run(&mut s, "cam.job.nest", json!({"job": file.to_string_lossy()}));
    // 200 × 100 boxes in the 280 × 280 between the margins, 10 mm apart: two stack (100 + 10 +
    // 100 = 210), leaving a 70 mm strip above and an 80 mm strip beside them, both narrower than
    // the panel's 100 mm, turned or not. Only two fit.
    assert_eq!(r["nesting"]["placements"].as_array().unwrap().len(), 2, "{r:#}");
    assert_eq!(r["nesting"]["unplaced"][0]["reason"], "no room left on the sheet");
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn nest_refuses_bad_jobs() {
    let mut s = Session::default();
    let err = |s: &mut Session, job: Value| s.execute("cam.job.nest", &json!({"job": job})).unwrap_err().to_string();
    assert!(err(&mut s, json!(5)).contains("a job object or the path"));
    assert!(err(&mut s, json!({"stock": {"size": [100, 100], "thickness": 3}, "parts": []})).contains("at least one part"));
    assert!(err(&mut s, json!({"stock": {"size": [100, 100], "thickness": 3}, "parts": [{"body": "Ghost"}]})).contains("no body named `Ghost`"));
    let missing = fixtures().join("no-such-design.sc");
    assert!(
        err(&mut s, json!({"stock": {"size": [100, 100], "thickness": 3}, "parts": [{"design": missing.to_string_lossy(), "body": "B"}]}))
            .contains("no-such-design.sc")
    );
    assert!(err(&mut s, json!("/no/such/job.json")).contains("/no/such/job.json"));
}

/// The scene of two front panels (200 × 100 × 3) on a 300 × 250 × 3 sheet, work zero in its
/// top centre: the stock spans −150…150, −125…125, −3…0; the files are written; each copy's
/// matrix, applied to its part's model as written, gives the copy's box; the copies lie on the
/// sheet, from its bottom (−3) to its top (0).
#[test]
fn export_scene_writes_models_and_where_they_go() {
    let dir = std::env::temp_dir().join(format!("solidcraft-cam-scene-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let job = json!({
        "stock": {"size": [300, 250], "thickness": 3},
        "strategy": {"roughing": {"pattern": "offset"}},
        "parts": [{"design": fixtures().join("front-panel.step").to_string_lossy(), "body": "Panel", "quantity": 2}]
    });
    let mut s = Session::default();
    let r = run(&mut s, "cam.job.export_scene", json!({"job": job, "path": dir.to_string_lossy()}));
    let sc = &r["scene"];
    let v3 = |v: &Value| [v[0].as_f64().unwrap(), v[1].as_f64().unwrap(), v[2].as_f64().unwrap()];
    assert_eq!(sc["work_zero"], "top_center");
    assert_eq!((v3(&sc["stock"]["min"]), v3(&sc["stock"]["max"])), ([-150.0, -125.0, -3.0], [150.0, 125.0, 0.0]));
    for f in ["scene.json", "stock.stl", "placed.stl"] {
        assert!(dir.join(f).is_file(), "{f}");
    }
    let model = dir.join(sc["parts"][0]["model"].as_str().unwrap());
    let bytes = std::fs::read(&model).unwrap();
    let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    assert_eq!(bytes.len(), 84 + 50 * n);
    let fl = |o: usize| f64::from(f32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]));
    let copies = sc["parts"][0]["copies"].as_array().unwrap();
    assert_eq!(copies.len(), 2);
    for c in copies {
        let m: Vec<Vec<f64>> =
            c["matrix"].as_array().unwrap().iter().map(|row| row.as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect()).collect();
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for t in 0..n {
            for k in 0..3 {
                let o = 84 + 50 * t + 12 + 12 * k;
                let p = [fl(o), fl(o + 4), fl(o + 8), 1.0];
                for (i, row) in m.iter().take(3).enumerate() {
                    let x: f64 = row.iter().zip(&p).map(|(a, b)| a * b).sum();
                    lo[i] = lo[i].min(x);
                    hi[i] = hi[i].max(x);
                }
            }
        }
        let (cmin, cmax) = (v3(&c["min"]), v3(&c["max"]));
        for i in 0..3 {
            assert!((lo[i] - cmin[i]).abs() < 1e-3 && (hi[i] - cmax[i]).abs() < 1e-3, "{lo:?} {hi:?} vs {cmin:?} {cmax:?}");
        }
        assert!((cmin[2] + 3.0).abs() < 1e-3 && cmax[2].abs() < 1e-3, "on the sheet: {cmin:?} {cmax:?}");
        assert!(cmin[0] >= -150.0 && cmax[0] <= 150.0 && cmin[1] >= -125.0 && cmax[1] <= 125.0);
    }
    let placed = std::fs::read(dir.join("placed.stl")).unwrap();
    assert_eq!(u32::from_le_bytes([placed[80], placed[81], placed[82], placed[83]]) as usize, 2 * n, "both copies");
    std::fs::remove_dir_all(&dir).ok();
}
