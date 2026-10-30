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
