//! Feature recognition on the CAM test parts, read from their STEP files, checked against
//! `tests/cam/features.json`: the features each part was designed with, written down when the
//! parts were made. (The engine checks `cam.part.inspect` on the `.sc` designs.)

use std::f64::consts::PI;
use std::path::{Path, PathBuf};

use serde_json::Value;

fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/cam")
}

fn expected() -> Value {
    serde_json::from_str(&std::fs::read_to_string(fixtures().join("features.json")).unwrap()).unwrap()
}

/// The part's features, read from the STEP file beside its design.
fn inspect(part: &Value) -> Value {
    let step = fixtures().join(part["step"].as_str().unwrap());
    let read = solidcraft_kernel::step_import(&std::fs::read_to_string(step).unwrap()).unwrap();
    assert_eq!(read.bodies.len(), 1, "one solid per test part");
    let body = &read.bodies[0].body;
    serde_json::to_value(crate::part::read_part(body).unwrap()).unwrap()
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}

fn xy(v: &Value) -> [f64; 2] {
    [f(&v[0]), f(&v[1])]
}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

/// Most a curved outline drawn as chords within the CAM area tolerance changes an area by, for
/// a total curved length `len`: 2/3 · tol · len.
fn chord(len: f64) -> f64 {
    2.0 / 3.0 * crate::part::AREA_TOL * len + 1e-6
}

/// Area and curved length of a w × h rectangle with corner radius r.
fn rounded_rect(size: &Value, r: f64) -> (f64, f64) {
    (f(&size[0]) * f(&size[1]) - (4.0 - PI) * r * r, 2.0 * PI * r)
}

/// The hole stack centred at `c`.
fn stack_at(got: &Value, c: [f64; 2]) -> &Value {
    got["holes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|h| near(f(&h["center"][0]), c[0], 1e-6) && near(f(&h["center"][1]), c[1], 1e-6))
        .unwrap_or_else(|| panic!("no hole stack at {c:?}: {:#}", got["holes"]))
}

/// The step of `stack` of `kind` with radius `r` at its top and the given heights.
fn has_step(stack: &Value, kind: &str, r_top: f64, r_bottom: f64, z_top: f64, z_bottom: f64) -> bool {
    stack["steps"].as_array().unwrap().iter().any(|t| {
        t["kind"] == kind
            && near(f(&t["r_top"]), r_top, 1e-6)
            && near(f(&t["r_bottom"]), r_bottom, 1e-6)
            && near(f(&t["z_top"]), z_top, 1e-6)
            && near(f(&t["z_bottom"]), z_bottom, 1e-6)
    })
}

fn check_part(part: &Value) {
    let got = inspect(part);
    let name = part["step"].as_str().unwrap();
    let tz = f(&part["top_z"]);
    assert!(near(f(&got["top_z"]), tz, 1e-6), "{name}: top {}", got["top_z"]);
    assert!(near(f(&got["bottom_z"]), f(&part["bbox"][0][2]), 1e-6), "{name}: bottom {}", got["bottom_z"]);
    for [i, corner] in [[0usize, 0usize], [0, 1], [1, 0], [1, 1]] {
        assert!(near(f(&got["bounds"][i][corner]), f(&part["bbox"][i][corner]), 1e-6), "{name}: bounds {}", got["bounds"]);
    }
    assert!(got["unsupported"].as_array().unwrap().is_empty(), "{name}: {:#}", got["unsupported"]);

    // Upward floors at every level but the bottom.
    let mut want: Vec<f64> = part["floor_levels"].as_array().unwrap().iter().map(f).filter(|z| *z > f(&part["bbox"][0][2]) + 1e-9).collect();
    want.sort_by(f64::total_cmp);
    let mut levels: Vec<f64> = got["floors"].as_array().unwrap().iter().map(|fl| (f(&fl["z"]) * 1e6).round() / 1e6).collect();
    levels.sort_by(f64::total_cmp);
    levels.dedup();
    assert_eq!(levels, want, "{name}: floor levels");

    let mut stacks = 0;
    let mut chamfers = 0;
    let mut cutouts = 0;
    for feat in part["features"].as_array().unwrap() {
        match feat["kind"].as_str().unwrap() {
            "outline" => {
                let (area, curved) = rounded_rect(&feat["size"], f(&feat["corner_radius"]));
                assert!(near(f(&got["outline_area"]), area, chord(curved)), "{name}: outline area {} vs {area}", got["outline_area"]);
            }
            "chamfer" => {
                chamfers += 1;
                let on = if feat["edge"] == "outline_top" { "outline" } else { "opening" };
                let w = f(&feat["width"]);
                let found = got["chamfers"].as_array().unwrap().iter().any(|c| {
                    c["on"] == on
                        && near(f(&c["width"]), w, 1e-6)
                        && near(f(&c["angle_deg"]), f(&feat["angle_deg"]), 1e-6)
                        && near(f(&c["z_top"]), tz, 1e-6)
                });
                // A chamfer round the top of a round recess is part of its hole stack: a cone of
                // the same width on top of the recess.
                let in_stack = feat["edge"] == "pocket_top" && {
                    let r = f(&feat["pocket_diameter"]) / 2.0;
                    got["holes"].as_array().unwrap().iter().any(|h| has_step(h, "cone", r + w, r, tz, tz - w))
                };
                if in_stack {
                    chamfers -= 1;
                }
                assert!(found || in_stack, "{name}: no {on} chamfer {w} wide: {:#}", got["chamfers"]);
            }
            "pocket" if feat["shape"] == "circle" => {
                let s = stack_at(&got, xy(&feat["center"]));
                let r = f(&feat["diameter"]) / 2.0;
                let floor = f(&feat["floor_z"]);
                let top = s["steps"].as_array().unwrap().iter().find(|t| t["kind"] == "cylinder" && near(f(&t["r_top"]), r, 1e-6)).unwrap();
                assert!(near(f(&top["z_bottom"]), floor, 1e-6), "{name}: recess floor {}", top["z_bottom"]);
                assert!(got["floors"].as_array().unwrap().iter().any(|fl| near(f(&fl["z"]), floor, 1e-6) && fl["in_hole"] == true));
            }
            "pocket" => {
                let (area, curved) = rounded_rect(&feat["size"], f(&feat["corner_radius"]));
                let c = xy(&feat["center"]);
                let floor = got["floors"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .find(|fl| near(f(&fl["z"]), f(&feat["floor_z"]), 1e-6) && fl["in_hole"] == false)
                    .unwrap_or_else(|| panic!("{name}: no pocket floor at {}", feat["floor_z"]));
                assert!(near(f(&floor["area"]), area, chord(curved)), "{name}: pocket floor {} vs {area}", floor["area"]);
                let b = &floor["bounds"];
                assert!(near((f(&b[0][0]) + f(&b[1][0])) / 2.0, c[0], 1e-6) && near((f(&b[0][1]) + f(&b[1][1])) / 2.0, c[1], 1e-6));
            }
            "hole" => {
                stacks += 1;
                let s = stack_at(&got, xy(&feat["center"]));
                let r = f(&feat["diameter"]) / 2.0;
                assert!(
                    has_step(s, "cylinder", r, r, f(&feat["from_z"]), f(&feat["to_z"])),
                    "{name}: hole Ø{} at {}: {s:#}",
                    2.0 * r,
                    feat["center"]
                );
                assert_eq!(s["through"], feat["through"]);
                assert!(near(f(&s["diameter"]), 2.0 * r, 1e-6));
                if f(&feat["from_z"]) >= tz - 1e-9 {
                    assert_eq!(s["kind"], "hole", "{name}: a plain hole at {}", feat["center"]);
                }
            }
            "counterbore" => {
                stacks += 1;
                let s = stack_at(&got, xy(&feat["center"]));
                assert_eq!(s["kind"], "counterbore");
                let (r, rc, seat) = (f(&feat["diameter"]) / 2.0, f(&feat["cb_diameter"]) / 2.0, f(&feat["cb_floor_z"]));
                assert!(has_step(s, "cylinder", rc, rc, tz, seat) && has_step(s, "cylinder", r, r, seat, f(&feat["to_z"])), "{s:#}");
                assert!(near(f(&s["diameter"]), 2.0 * r, 1e-6) && s["through"] == true);
            }
            "countersink" => {
                stacks += 1;
                let s = stack_at(&got, xy(&feat["center"]));
                assert_eq!(s["kind"], "countersink");
                let (r, rc) = (f(&feat["diameter"]) / 2.0, f(&feat["cs_diameter"]) / 2.0);
                // a cone of the given angle from the countersink diameter down to the hole's
                let depth = (rc - r) / (f(&feat["cs_angle_deg"]) / 2.0).to_radians().tan();
                assert!(has_step(s, "cone", rc, r, tz, tz - depth), "{name}: countersink Ø{} at {}: {s:#}", 2.0 * rc, feat["center"]);
                assert!(has_step(s, "cylinder", r, r, tz - depth, f(&feat["to_z"])), "{s:#}");
                assert!(near(f(&s["diameter"]), 2.0 * r, 1e-6) && s["through"] == true);
            }
            "cutout" => {
                cutouts += 1;
                let (area, curved) = rounded_rect(&feat["size"], f(&feat["corner_radius"]));
                let c = xy(&feat["center"]);
                let found = got["cutouts"].as_array().unwrap().iter().any(|k| {
                    let b = &k["bounds"];
                    near(f(&k["area"]), area, chord(curved))
                        && near((f(&b[0][0]) + f(&b[1][0])) / 2.0, c[0], 1e-6)
                        && near((f(&b[0][1]) + f(&b[1][1])) / 2.0, c[1], 1e-6)
                });
                assert!(found, "{name}: no cutout of {area} mm² at {c:?}: {:#}", got["cutouts"]);
            }
            other => panic!("{name}: features.json has a kind this test doesn't know: {other}"),
        }
    }
    // Nothing extra: a round recess with a hole under it is one stack (counted with its hole).
    assert_eq!(got["holes"].as_array().unwrap().len(), stacks, "{name}: hole stacks {:#}", got["holes"]);
    assert_eq!(got["chamfers"].as_array().unwrap().len(), chamfers, "{name}: chamfers {:#}", got["chamfers"]);
    assert_eq!(got["cutouts"].as_array().unwrap().len(), cutouts, "{name}: cutouts {:#}", got["cutouts"]);
}

#[test]
fn nema23_flange_features_match_the_design() {
    check_part(&with_step(&expected()["parts"][0]));
}

#[test]
fn front_panel_features_match_the_design() {
    check_part(&with_step(&expected()["parts"][1]));
}

/// The STEP file exported from the part's design.
fn with_step(part: &Value) -> Value {
    let mut p = part.clone();
    p["step"] = Value::from(part["design"].as_str().unwrap().replace(".sc", ".step"));
    p
}

/// Four flanges and two front panels, read from their STEP files, nested on a 600 × 400 × 10
/// sheet: their boxes (90 × 90 and 200 × 100) fit with room to spare, so every copy goes on, and
/// the outlines keep the margin and spacing.
#[test]
fn the_test_parts_nest_on_one_sheet() {
    use crate::job::Job;
    use crate::nest::{NestItem, nest};
    let job = Job::from_json(
        r#"{"stock": {"size": [600, 400], "thickness": 10, "material": "aluminium"},
            "nest": {"margin": 10, "spacing": 8},
            "parts": [{"design": "nema23-flange.step", "body": "Flange", "quantity": 4},
                      {"design": "front-panel.step", "body": "Panel", "quantity": 2}]}"#,
    )
    .unwrap();
    let features: Vec<crate::part::PartFeatures> = job
        .parts
        .iter()
        .map(|p| {
            let read = solidcraft_kernel::step_import(&std::fs::read_to_string(fixtures().join(&p.design)).unwrap()).unwrap();
            crate::part::read_part(&read.bodies[0].body).unwrap()
        })
        .collect();
    let items: Vec<NestItem> = job
        .parts
        .iter()
        .zip(&features)
        .map(|(p, f)| NestItem { label: p.label(), quantity: p.quantity, outline: &f.outline, height: f.top_z - f.bottom_z })
        .collect();
    let n = nest(&job.stock, &job.nest, &items);
    assert_eq!(n.placements.len(), 6, "{:?}", n.unplaced);
    let placed: Vec<crate::region::Area> =
        n.placements.iter().map(|p| features[p.part].outline.placed(p.quarter_turns, p.offset[0], p.offset[1])).collect();
    for a in &placed {
        let (lo, hi) = a.bounds().unwrap();
        assert!(lo[0] >= 10.0 - 1e-6 && lo[1] >= 10.0 - 1e-6 && hi[0] <= 590.0 + 1e-6 && hi[1] <= 390.0 + 1e-6, "{lo:?} {hi:?}");
    }
    for i in 0..placed.len() {
        for j in (i + 1)..placed.len() {
            let gap = placed[i].offset(4.0 - 1e-3, 0.01).unwrap().intersection(&placed[j].offset(4.0 - 1e-3, 0.01).unwrap()).unwrap();
            assert!(gap.is_empty(), "{:?} and {:?} closer than 8 mm", n.placements[i], n.placements[j]);
        }
    }
    // The outlines' areas over the usable 580 × 380: 4 rounded 90 × 90 squares and 2 rounded
    // 200 × 100 panels (their through openings inside the outline count as part).
    let flange = 8100.0 - (4.0 - PI) * 36.0;
    let panel = 20_000.0 - (4.0 - PI) * 16.0;
    let want = (4.0 * flange + 2.0 * panel) / (580.0 * 380.0);
    assert!((n.utilisation - want).abs() < 1e-4, "utilisation {} vs {want}", n.utilisation);
}
