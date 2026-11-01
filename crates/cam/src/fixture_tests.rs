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
                // a pocket that isn't one rounded rectangle (a ring round a deeper step, a cross)
                // states its floor's area and curved length
                let (area, curved) = match (feat.get("floor_area"), feat.get("curved_length")) {
                    (Some(a), Some(l)) => (f(a), f(l)),
                    _ => rounded_rect(&feat["size"], f(&feat["corner_radius"])),
                };
                // the floor's bounds come from its outline drawn as chords, each within the
                // area tolerance of the curve
                let c = xy(&feat["center"]);
                let tol = crate::part::AREA_TOL;
                let floor = got["floors"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .find(|fl| {
                        let b = &fl["bounds"];
                        near(f(&fl["z"]), f(&feat["floor_z"]), 1e-6)
                            && fl["in_hole"] == false
                            && near((f(&b[0][0]) + f(&b[1][0])) / 2.0, c[0], tol)
                            && near((f(&b[0][1]) + f(&b[1][1])) / 2.0, c[1], tol)
                    })
                    .unwrap_or_else(|| panic!("{name}: no pocket floor at {} centred on {c:?}: {}", feat["floor_z"], got["floors"]));
                assert!(near(f(&floor["area"]), area, chord(curved)), "{name}: pocket floor {} vs {area}", floor["area"]);
                let b = &floor["bounds"];
                assert!(near((f(&b[0][0]) + f(&b[1][0])) / 2.0, c[0], tol) && near((f(&b[0][1]) + f(&b[1][1])) / 2.0, c[1], tol));
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

#[test]
fn slot_plate_features_match_the_design() {
    check_part(&with_step(&expected()["parts"][2]));
}

#[test]
fn stepped_slot_features_match_the_design() {
    check_part(&with_step(&expected()["parts"][3]));
}

#[test]
fn cross_slot_features_match_the_design() {
    check_part(&with_step(&expected()["parts"][4]));
}

#[test]
fn odd_slots_features_match_the_design() {
    check_part(&with_step(&expected()["parts"][5]));
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

/// The tools of a small aluminium job: Ø6 and Ø3 end mills, a Ø5.5 drill (M5 clearance), a
/// 90° Ø12 chamfer cutter.
const TOOLS: &str = r#"[
    {"number": 1, "kind": "flat", "diameter": 6, "flute_length": 22, "rpm": 18000, "feed": 1200, "plunge": 300, "stepdown": 3},
    {"number": 2, "kind": "flat", "diameter": 3, "flute_length": 12, "rpm": 22000, "feed": 700, "plunge": 150, "stepdown": 1.5},
    {"number": 3, "kind": "drill", "diameter": 5.5, "flute_length": 40, "rpm": 2500, "feed": 150, "plunge": 150, "peck": 2},
    {"number": 4, "kind": "chamfer", "diameter": 12, "flute_length": 6, "rpm": 12000, "feed": 600, "plunge": 200, "tip_angle_deg": 90}
]"#;

fn plan_fixture(i: usize, thickness: f64) -> (crate::part::PartFeatures, crate::plan::PartPlan) {
    let part = &expected()["parts"][i];
    let read =
        solidcraft_kernel::step_import(&std::fs::read_to_string(fixtures().join(part["design"].as_str().unwrap().replace(".sc", ".step"))).unwrap())
            .unwrap();
    let f = crate::part::read_part(&read.bodies[0].body).unwrap();
    let job = crate::job::Job::from_json(&format!(
        r#"{{"stock": {{"size": [600, 400], "thickness": {thickness}}}, "tools": {TOOLS},
            "strategy": {{"countersinks": "chamfer_mill", "cut_out": {{"method": "profile"}}}}, "parts": [{{"body": "P"}}]}}"#
    ))
    .unwrap();
    let plan = crate::plan::plan_part(&f, &job, "P").unwrap();
    (f, plan)
}

/// The front panel's four M3 countersinks (Ø3.4 holes, Ø6.6 at 90°: the cone from r 3.3 at the
/// top, z 3, to r 1.7 at z 1.4), end-milled by default with the end mill that fits the Ø3.4 hole,
/// T2 Ø3 (3 × 1.05 ≤ 3.4): roughed in its 1.5 stepdown (at 2.2 and 1.4), then finished in
/// 16 steps of 0.1 from the top: at each step z one ring of radius 1.7 + (z − 1.4) − 1.5, the
/// tool's edge on the cone; nowhere past the cone; first of all on the sheet. The holes under them are bored from the panel's top (the countersinks leave
/// their middles), with the other through holes near the end, just before the separation.
#[test]
fn countersinks_are_end_milled_first_and_holes_bored_last() {
    use crate::toolpath::{Move, OpKind};
    let part = &expected()["parts"][1];
    let read =
        solidcraft_kernel::step_import(&std::fs::read_to_string(fixtures().join(part["design"].as_str().unwrap().replace(".sc", ".step"))).unwrap())
            .unwrap();
    let f = crate::part::read_part(&read.bodies[0].body).unwrap();
    let job =
        crate::job::Job::from_json(&format!(r#"{{"stock": {{"size": [600, 400], "thickness": 3}}, "tools": {TOOLS}, "parts": [{{"body": "P"}}]}}"#))
            .unwrap();
    let plan = crate::plan::plan_part(&f, &job, "P").unwrap();
    let sinks: Vec<[f64; 2]> = part["features"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|ft| ft["kind"] == "countersink")
        .map(|ft| [ft["center"][0].as_f64().unwrap(), ft["center"][1].as_f64().unwrap()])
        .collect();
    assert_eq!(sinks.len(), 4);
    let ops = ops_of(&plan, OpKind::Countersink);
    assert_eq!(ops.len(), 1);
    assert_eq!(ops[0].tool, 2);
    let cone = |z: f64| 1.7 + (z - 1.4);
    let mut at = [f64::NAN; 3];
    let mut flat: Vec<([f64; 2], f64, f64)> = Vec::new();
    for m in &ops[0].moves {
        let to = m.end();
        if let Move::Arc { center, .. } = m {
            let r = (to[0] - center[0]).hypot(to[1] - center[1]);
            assert!(sinks.iter().any(|c| (c[0] - center[0]).hypot(c[1] - center[1]) < 1e-6), "an arc about {center:?}");
            // never past the cone: at the arc's lower end, the tool's edge within the cone there
            let low = to[2].min(at[2]);
            assert!(r + 1.5 <= cone(low) + 1e-6, "at z {low} the edge is {} out, the cone {}", r + 1.5, cone(low));
            if (to[2] - at[2]).abs() < 1e-12 {
                flat.push((*center, to[2], r));
            }
        }
        at = to;
    }
    for c in &sinks {
        let mine: Vec<(f64, f64)> =
            flat.iter().filter(|(cc, ..)| (cc[0] - c[0]).hypot(cc[1] - c[1]) < 1e-6).map(|(_, z, r)| ((z * 1e6).round() / 1e6, *r)).collect();
        // roughed in the tool's 1.5 stepdown (1.6 deep: two steps, at 2.2 and 1.4), then finished
        // every 0.1 from the top: the finishing rings, the last 16 levels, in order, each with
        // the edge on the cone
        let mut levels = mine.clone();
        levels.dedup_by(|a, b| a.0 == b.0);
        let want: Vec<f64> = [2.2, 1.4].into_iter().chain((1..=16).map(|k| ((3.0 - 0.1 * k as f64) * 1e6).round() / 1e6)).collect();
        assert_eq!(levels.iter().map(|l| l.0).collect::<Vec<_>>(), want, "the steps at {c:?}");
        for (z, r) in levels.iter().skip(2) {
            assert!((r - (cone(*z) - 1.5)).abs() < 1e-6, "at {c:?} z {z} the ring is r {r}");
        }
    }
    // the bores under them start at the panel's top
    let bores = ops_of(&plan, OpKind::ThroughHole);
    for c in &sinks {
        let starts = bores.iter().flat_map(|o| &o.moves).any(|m| match m {
            Move::Line { to, .. } => (to[0] - c[0]).hypot(to[1] - c[1]) < 0.21 && (to[2] - 3.0).abs() < 1e-9,
            _ => false,
        });
        assert!(starts, "the bore at {c:?} starts at z 3");
    }

    // on a sheet: countersinks first, through holes just before the separation
    let pl = crate::nest::Placement { part: 0, label: "P".into(), copy: 0, quarter_turns: 0, offset: [120.0, 70.0], bounds: [[0.0; 2]; 2] };
    let (all, _) = crate::plan::plan_sheet(&job, &[(&f, &plan)], &[pl]).unwrap();
    let kinds: Vec<OpKind> = all.iter().map(|o| o.kind).collect();
    let first = |k: OpKind| kinds.iter().position(|x| *x == k).unwrap();
    let last = |k: OpKind| kinds.iter().rposition(|x| *x == k).unwrap();
    assert!(last(OpKind::Countersink) < first(OpKind::Roughing), "{kinds:?}");
    assert!(last(OpKind::EdgeChamfer) < first(OpKind::ThroughHole) && last(OpKind::Profile) < first(OpKind::ThroughHole), "{kinds:?}");
    assert_eq!(last(OpKind::ThroughHole) + 1, first(OpKind::Separation), "{kinds:?}");
}

/// Pocket corners sharper than the finishing tool (T2, Ø3) are warned about, each once, down to
/// its pocket's floor, with how far material is left into it: r(1/sin(a/2) − 1) for a corner of
/// angle a and r 1.5. odd-slots has twelve: the L's five 90° corners (its sixth, at (−42, −38),
/// points into the pocket and is cut), the island pocket's four, and the triangle's (base
/// angles atan(30/15) = 63.43°, apex 53.13°). The other test parts have none: their corners are
/// rounded R2 or more, and a cross's or an island's corners point into the pocket. Roughing by
/// offsets keeps the test quick.
#[test]
fn corners_too_sharp_for_the_finishing_tool_are_warned_about() {
    let depth = |a_deg: f64| 1.5 * (1.0 / (a_deg.to_radians() / 2.0).sin() - 1.0);
    let base = (30.0f64 / 15.0).atan().to_degrees();
    let want: Vec<([f64; 2], f64, f64)> = vec![
        ([-70.0, -50.0], 5.0, depth(90.0)),
        ([-30.0, -50.0], 5.0, depth(90.0)),
        ([-30.0, -10.0], 5.0, depth(90.0)),
        ([-42.0, -10.0], 5.0, depth(90.0)),
        ([-70.0, -38.0], 5.0, depth(90.0)),
        ([40.0, -50.0], 5.0, depth(90.0)),
        ([76.0, -50.0], 5.0, depth(90.0)),
        ([76.0, -20.0], 5.0, depth(90.0)),
        ([40.0, -20.0], 5.0, depth(90.0)),
        ([0.0, -50.0], 4.0, depth(base)),
        ([30.0, -50.0], 4.0, depth(base)),
        ([15.0, -20.0], 4.0, depth(180.0 - 2.0 * base)),
    ];
    for i in 0..expected()["parts"].as_array().unwrap().len() {
        let part = &expected()["parts"][i];
        let read = solidcraft_kernel::step_import(
            &std::fs::read_to_string(fixtures().join(part["design"].as_str().unwrap().replace(".sc", ".step"))).unwrap(),
        )
        .unwrap();
        let f = crate::part::read_part(&read.bodies[0].body).unwrap();
        let thickness = part["stock_thickness"].as_f64().unwrap();
        let job = crate::job::Job::from_json(&format!(
            r#"{{"stock": {{"size": [600, 400], "thickness": {thickness}}}, "tools": {TOOLS},
                "strategy": {{"roughing": {{"pattern": "offset"}}}}, "parts": [{{"body": "P"}}]}}"#
        ))
        .unwrap();
        let plan = crate::plan::plan_part(&f, &job, "P").unwrap();
        let got: Vec<&String> = plan.warnings.iter().filter(|w| w.contains("pocket corner")).collect();
        let name = part["design"].as_str().unwrap();
        if name != "odd-slots.sc" {
            assert!(got.is_empty(), "{name}: {got:#?}");
            continue;
        }
        assert_eq!(got.len(), want.len(), "{name}: {got:#?}");
        for (at, z, d) in &want {
            let text =
                format!("T2 (Ø3) can't reach into the pocket corner at ({:.2}, {:.2}) down to z {z:.3}: it leaves material up to ", at[0], at[1]);
            // −0.00 prints for a coordinate a hair below zero
            let alt = text.replace("(0.00,", "(-0.00,");
            let w = got.iter().find(|w| w.contains(&text) || w.contains(&alt)).unwrap_or_else(|| panic!("{name}: no warning \"{text}\": {got:#?}"));
            // within the paths' tolerance (the covered region's arcs are drawn as chords inside
            // them, 0.005) and the printing's
            let left: f64 = w.split("up to ").nth(1).and_then(|r| r.split(' ').next()).unwrap().parse().unwrap();
            assert!((left - d).abs() <= 0.01, "{name}: {left} mm left at {at:?}, worked out {d:.4}");
        }
    }
}

fn ops_of(plan: &crate::plan::PartPlan, kind: crate::toolpath::OpKind) -> Vec<&crate::toolpath::Operation> {
    plan.ops.iter().filter(|o| o.kind == kind).collect()
}

/// The flange, every feature from features.json machined by the right tool:
/// - the 8 Ø5.5 through holes (4 under counterbores, 4 under countersinks) drilled with T3,
///   from 1 mm above the flange's top (nothing is milled yet) to −0.2 less the drill's point
///   (2.75 / tan 59°);
/// - the Ø38.2 recess, the Ø24 bore and the Ø10 counterbores milled with T2;
/// - the countersinks (Ø10.4, cone foot r 2.75 at z 7.55) and the recess's chamfer (foot r 19.1
///   at z 9.5) cut by T4 0.3 below their foot and 0.3 in from it: circles r 2.45 at z 7.25 and
///   r 18.8 at z 9.2;
/// - the outline chamfer (foot at z 9) by T4 0.3 outside the outline at z 8.7, after the
///   profile; the profile with T1, 3 mm outside the outline.
#[test]
fn the_flange_is_planned_feature_by_feature() {
    use crate::toolpath::{Move, OpKind};
    let (_, plan) = plan_fixture(0, 10.0);
    assert!(plan.warnings.is_empty(), "{:#?}", plan.warnings);
    assert!(ops_of(&plan, OpKind::Roughing).is_empty() && ops_of(&plan, OpKind::Facing).is_empty(), "no pockets or facing on the flange");

    let drills = ops_of(&plan, OpKind::Drilling);
    assert_eq!(drills.len(), 1);
    assert_eq!(drills[0].tool, 3);
    let tip = 2.75 / 59f64.to_radians().tan();
    let mut centres = Vec::new();
    for m in drills[0].moves.iter().filter(|m| !matches!(m, Move::Rapid { .. })) {
        let Move::Drill { at, r, bottom, back, .. } = m else { panic!("only drilling cycles and rapids: {m:?}") };
        assert!((back - 15.0).abs() < 1e-9, "each cycle returns to the clearance height (G98)");
        assert!((bottom - (-0.2 - tip)).abs() < 1e-6, "drilled to {bottom}");
        let cb = (at[0].abs() - 23.57).abs() < 1e-6;
        let cs = (at[0].abs() - 35.0).abs() < 1e-6;
        assert!(cb || cs, "a drill at {at:?}");
        // Drilling comes before the counterbores and countersinks are cut, so every cycle starts
        // 1 mm above the flange's top (10): starting above the seat (5) or the cone's foot
        // (7.55) would rapid into solid metal.
        assert!((r - 11.0).abs() < 1e-6, "retract {r} for {at:?}");
        centres.push(*at);
    }
    assert_eq!(centres.len(), 8);

    let holes = ops_of(&plan, OpKind::HoleMilling);
    assert_eq!(holes.len(), 1);
    assert_eq!(holes[0].tool, 2);

    let cones = ops_of(&plan, OpKind::Chamfer);
    assert_eq!(cones.len(), 1);
    let circles: Vec<(f64, f64)> = cones[0]
        .moves
        .iter()
        .filter_map(|m| match m {
            Move::Arc { to, center, .. } => Some((((to[0] - center[0]).hypot(to[1] - center[1]) * 1e6).round() / 1e6, (to[2] * 1e6).round() / 1e6)),
            _ => None,
        })
        .collect();
    assert_eq!(circles.iter().filter(|c| **c == (2.45, 7.25)).count(), 4, "{circles:?}");
    assert_eq!(circles.iter().filter(|c| **c == (18.8, 9.2)).count(), 1, "{circles:?}");

    let edge = ops_of(&plan, OpKind::EdgeChamfer);
    assert_eq!(edge.len(), 1);
    let flat: Vec<[f64; 3]> =
        edge[0].moves.iter().filter_map(|m| if let Move::Line { to, .. } = m { (to[2] < 9.0).then_some(*to) } else { None }).collect();
    assert!(!flat.is_empty());
    for p in &flat {
        assert!((p[2] - 8.7).abs() < 1e-9);
        let d = rounded_rect_dist([p[0], p[1]], 45.0, 45.0, 6.0);
        assert!((d - 0.3).abs() <= crate::plan::PATH_TOL + 1e-6, "edge chamfer at {p:?} is {d} off the outline");
    }
    let prof = ops_of(&plan, OpKind::Profile);
    assert_eq!(prof[0].tool, 1);
    // the outline chamfer runs after the profile, the countersinks before it
    let rank = |k: OpKind| k.rank();
    assert!(rank(OpKind::Chamfer) < rank(OpKind::Profile) && rank(OpKind::Profile) < rank(OpKind::EdgeChamfer));
}

/// Signed distance from `p` to the outside of a rectangle of half-sizes `hx`, `hy` centred on
/// the origin with corner radius `r`.
fn rounded_rect_dist(p: [f64; 2], hx: f64, hy: f64, r: f64) -> f64 {
    let qx = p[0].abs() - (hx - r);
    let qy = p[1].abs() - (hy - r);
    qx.max(0.0).hypot(qy.max(0.0)) + qx.max(qy).min(0.0) - r
}

/// The panel: the label recess roughed to 2.4 and finished at 2.2, the display cutout cleared
/// through, its chamfer and the outline's cut, the round holes milled (no drill of their size),
/// the countersinks cut by T4 at r 1.4, z 1.1 (foot r 1.7 at z 1.4).
#[test]
fn the_panel_is_planned_feature_by_feature() {
    use crate::toolpath::{Move, OpKind};
    let (_, plan) = plan_fixture(1, 3.0);
    assert!(plan.warnings.is_empty(), "{:#?}", plan.warnings);
    let lows = |kind: OpKind| -> f64 {
        ops_of(&plan, kind)
            .iter()
            .flat_map(|o| o.moves.iter())
            .filter_map(|m| if let Move::Line { to, .. } = m { Some(to[2]) } else { None })
            .fold(f64::INFINITY, f64::min)
    };
    assert!((lows(OpKind::Roughing) + 0.2).abs() < 1e-9, "the cutout is cleared through");
    let floors: Vec<f64> = ops_of(&plan, OpKind::FloorFinish)
        .iter()
        .flat_map(|o| o.moves.iter())
        .filter_map(|m| if let Move::Line { to, .. } = m { Some(to[2]) } else { None })
        .collect();
    assert!(floors.iter().any(|z| (z - 2.2).abs() < 1e-9) && floors.iter().all(|z| *z >= 2.2 - 1e-9), "the label floor at 2.2");
    assert!(ops_of(&plan, OpKind::Drilling).is_empty(), "no drill of the panel's sizes");
    let cones = ops_of(&plan, OpKind::Chamfer);
    let csk = cones
        .iter()
        .flat_map(|o| o.moves.iter())
        .filter(|m| matches!(m, Move::Arc { to, center, .. } if ((to[0] - center[0]).hypot(to[1] - center[1]) - 1.4).abs() < 1e-6 && (to[2] - 1.1).abs() < 1e-6))
        .count();
    assert_eq!(csk, 4, "four countersinks");
    assert_eq!(cones.len(), 2, "countersinks and the display's chamfer: {:?}", cones.iter().map(|o| &o.name).collect::<Vec<_>>());
    assert_eq!(ops_of(&plan, OpKind::EdgeChamfer).len(), 1);
}

/// Whatever the features, on both test parts: every rapid that moves sideways does it at the
/// clearance height (top + 5), every drilling cycle starts above the part's top, and no move
/// goes below the spoilboard allowance (−0.2) or the drill's point under it.
#[test]
fn no_operation_rapids_into_the_material() {
    use crate::toolpath::Move;
    for (i, thickness) in [(0usize, 10.0), (1, 3.0)] {
        let (f, plan) = plan_fixture(i, thickness);
        let clearance = f.top_z + 5.0;
        for op in &plan.ops {
            let mut at = [0.0, 0.0, clearance];
            for m in &op.moves {
                let to = m.end();
                match m {
                    Move::Rapid { to } if (to[0] - at[0]).hypot(to[1] - at[1]) > 1e-9 => {
                        assert!((at[2] - clearance).abs() < 1e-9 && (to[2] - clearance).abs() < 1e-9, "{}: rapid across at {at:?} → {to:?}", op.name);
                    }
                    Move::Drill { r, bottom, .. } => {
                        assert!(*r > f.top_z, "{}: a drilling cycle starts at {r}, inside the part (top {})", op.name, f.top_z);
                        assert!(*bottom >= -0.2 - 2.75 / 59f64.to_radians().tan() - 1e-9, "{}: drilled to {bottom}", op.name);
                    }
                    _ => assert!(to[2] >= -0.2 - 1e-9, "{}: {m:?} below the spoilboard allowance", op.name),
                }
                at = to;
            }
            assert!((at[2] - clearance).abs() < 1e-9, "{} ends at {at:?}, not at the clearance height", op.name);
        }
    }
}

/// Placing on the sheet: a part turned a quarter turn and moved by (100, 50) has its point
/// (x, y) at (100 − y, 50 + x); its top (z 10 on a 10 mm sheet) is the sheet's top, Z0.
#[test]
fn placed_paths_are_turned_moved_and_lowered() {
    use crate::nest::Placement;
    use crate::plan::plan_sheet;
    use crate::toolpath::Move;
    let (f, plan) = plan_fixture(0, 10.0);
    let job = crate::job::Job::from_json(&format!(
        r#"{{"stock": {{"size": [600, 400], "thickness": 10, "origin": "top_front_left"}}, "tools": {TOOLS}, "parts": [{{"body": "P"}}]}}"#
    ))
    .unwrap();
    let pl = Placement { part: 0, label: "P".into(), copy: 1, quarter_turns: 1, offset: [100.0, 50.0], bounds: [[0.0; 2]; 2] };
    let (all, _) = plan_sheet(&job, &[(&f, &plan)], &[pl]).unwrap();
    assert!(all.windows(2).all(|w| w[0].kind.rank() <= w[1].kind.rank()), "in machining order");
    // the part's own operations, besides the roughing round it
    let ops: Vec<_> = all.iter().filter(|o| !o.name.starts_with("round the parts")).collect();
    assert_eq!(ops.len(), plan.ops.len());
    assert!(ops.iter().all(|o| o.name.starts_with("P #2: ")), "{:?}", ops.iter().map(|o| &o.name).collect::<Vec<_>>());
    let drill = ops.iter().find(|o| o.kind == crate::toolpath::OpKind::Drilling).unwrap();
    for (m, orig) in drill.moves.iter().zip(&plan.ops.iter().find(|o| o.kind == crate::toolpath::OpKind::Drilling).unwrap().moves) {
        let (Move::Drill { at, r, .. }, Move::Drill { at: a0, r: r0, .. }) = (m, orig) else { continue };
        assert!((at[0] - (100.0 - a0[1])).abs() < 1e-9 && (at[1] - (50.0 + a0[0])).abs() < 1e-9);
        assert!((r - (r0 - 10.0)).abs() < 1e-9 && (r - 1.0).abs() < 1e-9, "1 mm above the sheet's top");
    }
}

/// The panel's label recess (70 × 14, floor at 2.2) is roughed at 2.4 before its floor is
/// finished: roughing passes at 2.4 inside it, 3.5 or more from its walls.
#[test]
fn the_label_recess_is_roughed_before_its_floor() {
    use crate::toolpath::{Move, OpKind};
    let (_, plan) = plan_fixture(1, 3.0);
    let rough = ops_of(&plan, OpKind::Roughing);
    let in_recess: Vec<[f64; 3]> = rough
        .iter()
        .flat_map(|o| o.moves.iter())
        .filter_map(|m| if let Move::Line { to, .. } = m { Some(*to) } else { None })
        .filter(|p| (p[2] - 2.4).abs() < 1e-9 && p[1] < -20.0)
        .collect();
    assert!(!in_recess.is_empty(), "no roughing in the label recess");
    for p in &in_recess {
        let d = rounded_rect_dist([p[0] + 40.0, p[1] + 30.0], 35.0, 7.0, 3.0);
        assert!(d <= -(3.5 - crate::plan::PATH_TOL - 1e-6), "roughing at {p:?} is {} from the recess wall", -d);
    }
}

/// The flange sheet's preview (cut out by the profile), checked pixel by pixel where the
/// geometry says what must be there: the profile's dark line 3 mm outside a flange's straight edge, a drilled hole's red
/// cross at its centre, white part face between features; and the SVG names every operation.
#[test]
fn the_preview_draws_the_toolpaths_where_they_are() {
    use crate::preview::{Scene, colour, png, svg};
    use crate::toolpath::OpKind;
    let part = &expected()["parts"][0];
    let read =
        solidcraft_kernel::step_import(&std::fs::read_to_string(fixtures().join(part["design"].as_str().unwrap().replace(".sc", ".step"))).unwrap())
            .unwrap();
    let job = crate::job::Job::from_json(&format!(
        r#"{{"stock": {{"size": [220, 220], "thickness": 10, "origin": "top_front_left"}}, "nest": {{"margin": 10, "spacing": 10}}, "tools": {TOOLS},
            "strategy": {{"countersinks": "chamfer_mill", "cut_out": {{"method": "profile"}}}}, "parts": [{{"body": "F", "quantity": 4}}]}}"#
    ))
    .unwrap();
    let plan = crate::plan::plan_job(&job, &[read.bodies[0].body.clone()]).unwrap();
    assert!(plan.warnings.is_empty(), "{:?}", plan.warnings);
    assert_eq!(plan.nesting.placements.len(), 4);
    let outlines = plan.placed_outlines();
    let scene = Scene { sheet: [220.0, 220.0], origin: [0.0, 0.0], margin: 10.0, outlines: &outlines, ops: &plan.ops, clearance: 5.0 };

    // 1100 px wide: the sheet plus 4 % of 220 each side (8.8 mm), 4.545... px per mm
    let bytes = png(&scene, 1100).unwrap();
    let decoder = png::Decoder::new(std::io::Cursor::new(&bytes));
    let mut reader = decoder.read_info().unwrap();
    let mut buf = vec![0; reader.output_buffer_size().unwrap()];
    let info = reader.next_frame(&mut buf).unwrap();
    assert_eq!((info.width, info.height), (1100, 1100));
    let scale = 1100.0 / (220.0 + 2.0 * 8.8);
    let pixel = |x: f64, y: f64| {
        let (px, py) = (((x + 8.8) * scale) as usize, (((220.0 - y) + 8.8) * scale) as usize);
        let i = (py * 1100 + px) * 4;
        [buf[i], buf[i + 1], buf[i + 2]]
    };
    let near = |x: f64, y: f64, hex: &str| {
        let want = [&hex[1..3], &hex[3..5], &hex[5..7]].map(|h| u8::from_str_radix(h, 16).unwrap() as i32);
        (-2..=2).any(|dx| {
            (-2..=2).any(|dy| {
                let p = pixel(x + dx as f64 / scale, y + dy as f64 / scale);
                (0..3).all(|k| (p[k] as i32 - want[k]).abs() < 60)
            })
        })
    };
    // The first placement: its outline's box and centre.
    let pl = &plan.nesting.placements[0];
    let [lo, hi] = pl.bounds;
    let c = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
    assert!(near(c[0], lo[1] - 3.0, colour(OpKind::Profile)), "the profile 3 mm outside the bottom edge at {:?}", [c[0], lo[1] - 3.0]);
    let drilled = plan.ops.iter().find(|o| o.kind == OpKind::Drilling).unwrap();
    let crate::toolpath::Move::Drill { at, .. } = drilled.moves[1] else { panic!("a drilling cycle") };
    assert!(near(at[0], at[1], colour(OpKind::Drilling)), "a drilled hole's cross at {at:?}");
    // halfway between the centre bore (r 19.6 with its chamfer) and a counterbore (centre at
    // 23.57, 23.57 from the middle, r 5): plain part face
    let face = pixel(c[0] + 12.0, c[1] - 30.0);
    assert!(face.iter().all(|v| *v > 240), "part face at {:?} is {face:?}", [c[0] + 12.0, c[1] - 30.0]);

    let s = svg(&scene);
    assert!(s.starts_with("<svg") && s.ends_with("</svg>"));
    for op in &plan.ops {
        assert!(s.contains(&format!("<title>{}</title>", op.name)), "the SVG names {}", op.name);
    }
    for k in [OpKind::Drilling, OpKind::HoleMilling, OpKind::Chamfer, OpKind::Profile, OpKind::EdgeChamfer] {
        assert!(s.contains(colour(k)), "{k:?} drawn");
    }
    assert!(!s.contains(colour(OpKind::Roughing)), "the flange has no pockets to rough");
    assert!(s.contains(">drilling<") && s.contains(">profile<"), "the legend names what is drawn");
}

/// Tracing: a full clockwise circle of radius 4 traced every 3° gives 120 points on the circle,
/// ending where it began; a drilling cycle is a vertical stroke from its retract height down.
#[test]
fn traces_follow_arcs_and_drilling_cycles() {
    use crate::toolpath::{Move, OpKind, Operation, StrokeKind, trace};
    let op = Operation {
        kind: OpKind::HoleMilling,
        name: "t".into(),
        tool: 1,
        rpm: 1.0,
        moves: vec![
            Move::Rapid { to: [14.0, 10.0, 5.0] },
            Move::Line { to: [14.0, 10.0, -1.0], feed: 1.0 },
            Move::Arc { to: [14.0, 10.0, -1.0], center: [10.0, 10.0], ccw: false, feed: 1.0 },
            Move::Rapid { to: [30.0, 0.0, 5.0] },
            Move::Drill { at: [30.0, 0.0], back: 5.0, r: 1.0, bottom: -6.0, peck: None, feed: 1.0 },
        ],
    };
    let strokes = trace(&op, [f64::NAN, f64::NAN, 5.0], 3f64.to_radians());
    let cut: Vec<&[f64; 3]> = strokes.iter().filter(|s| s.kind == StrokeKind::Cut).flat_map(|s| s.pts.iter()).collect();
    let on_circle = cut.iter().filter(|p| ((p[0] - 10.0).hypot(p[1] - 10.0) - 4.0).abs() < 1e-9 && (p[2] + 1.0).abs() < 1e-12).count();
    assert!(on_circle >= 121, "{on_circle} points on the circle");
    // clockwise: the second point after the start (14, 10) is below the X axis
    let after = cut.iter().position(|p| (p[2] + 1.0).abs() < 1e-12 && p[0] == 14.0).unwrap();
    assert!(cut[after + 1][1] < 10.0, "clockwise");
    let drill = strokes.iter().find(|s| s.kind == StrokeKind::Cut && s.pts.first() == Some(&[30.0, 0.0, 1.0])).unwrap();
    assert_eq!(drill.pts.last(), Some(&[30.0, 0.0, -6.0]));
}

/// One move as read back from a program: what kind, where it ends, the feed, the arc's centre
/// and turn, the tool loaded and the spindle's speed.
#[derive(Debug, Clone, PartialEq)]
enum Read {
    Rapid([f64; 3]),
    Feed([f64; 3], f64),
    Arc([f64; 3], [f64; 2], bool, f64),
    Drill([f64; 2], f64, f64, Option<f64>, f64),
}

/// A small RS274 reader, independent of the post-processor: modal motion and feed, absolute
/// positions, arcs' I and J from their start, G98 G81/G83 cycles, tools and spindle. Returns
/// each move with the tool and spindle speed it ran with, and the program's other facts.
fn read_program(p: &str) -> (Vec<(Read, u32, f64)>, Vec<String>) {
    let mut out = Vec::new();
    let mut facts = Vec::new();
    let mut pos = [f64::NAN; 3];
    let (mut motion, mut feed, mut tool, mut h, mut spindle): (u32, f64, u32, u32, f64) = (0, 0.0, 0, 0, 0.0);
    let mut cycle: Option<u32> = None;
    for raw in p.lines() {
        assert!(raw.len() <= 256, "line too long: {raw}");
        // comments out
        let mut line = String::new();
        let mut depth = 0;
        for c in raw.chars() {
            match c {
                '(' => depth += 1,
                ')' => depth -= 1,
                c if depth == 0 => line.push(c),
                _ => {}
            }
        }
        let line = line.trim();
        if line.is_empty() || line == "%" {
            continue;
        }
        let mut words: Vec<(char, f64)> = Vec::new();
        let mut cur: Option<(char, String)> = None;
        for c in line.chars().filter(|c| !c.is_whitespace()) {
            if c.is_ascii_alphabetic() {
                if let Some((l, v)) = cur.take() {
                    words.push((l, v.parse().unwrap_or_else(|_| panic!("bad number in {raw}"))));
                }
                cur = Some((c.to_ascii_uppercase(), String::new()));
            } else if let Some((_, v)) = cur.as_mut() {
                v.push(c);
            } else {
                panic!("stray {c} in {raw}");
            }
        }
        if let Some((l, v)) = cur.take() {
            words.push((l, v.parse().unwrap_or_else(|_| panic!("bad number in {raw}"))));
        }
        let word = |l: char| words.iter().find(|w| w.0 == l).map(|w| w.1);
        let mut axes = false;
        for (l, v) in &words {
            match l {
                'G' => match (*v * 10.0).round() as u32 {
                    0 => motion = 0,
                    10 => motion = 1,
                    20 => motion = 2,
                    30 => motion = 3,
                    800 => cycle = None,
                    810 => cycle = Some(81),
                    830 => cycle = Some(83),
                    430 => h = word('H').unwrap() as u32,
                    490 => h = 0,
                    980 | 210 | 900 | 170 | 400 | 940 | 540 | 640 | 610 | 40 => facts.push(format!("G{v}")),
                    g => panic!("unexpected G{} in {raw}", g as f64 / 10.0),
                },
                'M' => match *v as u32 {
                    6 => tool = word('T').unwrap() as u32,
                    3 => spindle = word('S').unwrap(),
                    5 => spindle = 0.0,
                    7..=9 => {}
                    30 => facts.push("M30".into()),
                    m => panic!("unexpected M{m} in {raw}"),
                },
                'F' => feed = *v,
                'X' | 'Y' | 'Z' => axes = true,
                _ => {}
            }
        }
        if !axes {
            continue;
        }
        let mut to = pos;
        for (k, l) in ['X', 'Y', 'Z'].into_iter().enumerate() {
            if let Some(v) = word(l) {
                to[k] = v;
            }
        }
        if let Some(c) = cycle {
            assert!(words.iter().any(|w| w.0 == 'G' && (w.1 - 98.0).abs() < 1e-9), "cycles return to the start height (G98): {raw}");
            let q = word('Q');
            assert_eq!(c == 83, q.is_some(), "{raw}");
            out.push((Read::Drill([to[0], to[1]], to[2], word('R').unwrap(), q, feed), tool, spindle));
            // G98: back to the height it started from
            pos = [to[0], to[1], pos[2]];
            continue;
        }
        assert_eq!(h, tool, "the tool's length offset (G43 H) is the loaded tool's: {raw}");
        let r = match motion {
            0 => Read::Rapid(to),
            1 => Read::Feed(to, feed),
            m => Read::Arc(to, [pos[0] + word('I').unwrap(), pos[1] + word('J').unwrap()], m == 3, feed),
        };
        if motion != 0 {
            assert!(spindle > 0.0, "cutting with the spindle off: {raw}");
        }
        out.push((r, tool, spindle));
        pos = to;
    }
    (out, facts)
}

/// Two NEMA 23 flanges on a 220 × 120 × 10 sheet (drilled holes, helical bores, end-milled
/// countersinks, contours, separation), written for LinuxCNC and read back by `read_program`:
/// every move of every operation comes back in order (the same kind, end, feed, arc centre and
/// turn, drilling depth, retract and peck), each with its operation's tool loaded and the spindle
/// at its speed; the program is metric, absolute and ends with M30.
#[test]
fn the_linuxcnc_program_reads_back_as_planned() {
    use crate::toolpath::Move;
    let part = &expected()["parts"][0];
    let read =
        solidcraft_kernel::step_import(&std::fs::read_to_string(fixtures().join(part["design"].as_str().unwrap().replace(".sc", ".step"))).unwrap())
            .unwrap();
    let job = crate::job::Job::from_json(&format!(
        r#"{{"stock": {{"size": [220, 120], "thickness": 10}}, "tools": {TOOLS}, "post": {{"coolant": "mist", "fit_arcs": false}}, "parts": [{{"body": "F", "quantity": 2}}]}}"#
    ))
    .unwrap();
    let plan = crate::plan::plan_job(&job, &[read.bodies[0].body.clone()]).unwrap();
    assert_eq!(plan.nesting.placements.len(), 2);
    let program = crate::post::linuxcnc(&job, &plan.ops, "flanges", &plan.warnings).unwrap();
    assert!(program.starts_with("%\n") && program.trim_end().ends_with("%"));
    // the plan's warnings in the header, a long one over several comment lines
    let warned = crate::post::linuxcnc(&job, &plan.ops, "flanges", &["a long warning ".repeat(30), "short".into()]).unwrap();
    let head: Vec<&str> = warned.lines().take_while(|l| !l.starts_with('G')).filter(|l| l.starts_with("(WARNING")).collect();
    assert!(head.len() >= 4 && head.iter().all(|l| l.len() <= 256), "{head:?}");
    assert!(head.last().unwrap().contains("short"));
    assert!(program.contains("M7"), "mist with each tool");
    let (got, facts) = read_program(&program);
    for f in ["G21", "G90", "G17", "G94", "G54", "G98", "M30"] {
        assert!(facts.iter().any(|x| x == f), "{f} in {facts:?}");
    }
    // what the plan says, move by move, with its operation's tool and speed; moves that go
    // nowhere at the program's resolution (4 decimals) are left out
    let mut want: Vec<(Read, u32, f64)> = Vec::new();
    let mut at = [f64::NAN; 3];
    for op in &plan.ops {
        for m in &op.moves {
            let r = match m {
                Move::Rapid { to } => Read::Rapid(*to),
                Move::Line { to, feed } => Read::Feed(*to, *feed),
                Move::Arc { to, center, ccw, feed } => Read::Arc(*to, *center, *ccw, *feed),
                Move::Drill { at: c, back, r, bottom, peck, feed } => {
                    at = [c[0], c[1], *back];
                    want.push((Read::Drill(*c, *bottom, *r, *peck, *feed), op.tool, op.rpm));
                    continue;
                }
            };
            let to = m.end();
            // the same to the program's 4 decimals: no move at all for the control
            let same = |a: f64, b: f64| crate::post::num(a, 4) == crate::post::num(b, 4);
            if !matches!(r, Read::Arc(..)) && (0..3).all(|k| same(to[k], at[k])) {
                continue;
            }
            at = to;
            want.push((r, op.tool, op.rpm));
        }
    }
    // the program also goes straight up to clearance after each tool change, and over each hole
    // before its cycle: rapids the plan has as part of its own moves
    let got: Vec<(Read, u32, f64)> = got.into_iter().filter(|(r, ..)| !matches!(r, Read::Rapid(p) if p[0].is_nan() || p[1].is_nan())).collect();
    let near = |a: &[f64], b: &[f64]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 2e-4);
    let mut gi = 0;
    for (k, (w, tool, rpm)) in want.iter().enumerate() {
        // skip the program's own rapids that the plan does not have (to clearance, over a hole)
        while let Some((Read::Rapid(p), ..)) = got.get(gi)
            && !matches!(w, Read::Rapid(q) if near(p, q))
        {
            gi += 1;
        }
        let Some((g, gt, gs)) = got.get(gi) else { panic!("the program stops before move {k}: {w:?}") };
        let same = match (w, g) {
            (Read::Rapid(a), Read::Rapid(b)) => near(a, b),
            (Read::Feed(a, f), Read::Feed(b, h)) => near(a, b) && (f - h).abs() < 1e-3,
            (Read::Arc(a, c, cc, f), Read::Arc(b, d, dd, h)) => near(a, b) && near(c, d) && cc == dd && (f - h).abs() < 1e-3,
            (Read::Drill(a, z, r, q, f), Read::Drill(b, y, s, p, h)) => {
                near(a, b)
                    && (z - y).abs() < 2e-4
                    && (r - s).abs() < 2e-4
                    && q.zip(*p).is_none_or(|(x, v)| (x - v).abs() < 2e-4)
                    && (f - h).abs() < 1e-3
            }
            _ => false,
        };
        assert!(same, "move {k}: planned {w:?}, the program has {g:?}");
        assert_eq!((gt, gs), (tool, rpm), "move {k}: tool and speed");
        gi += 1;
    }
    assert!(want.len() > 1000);
    assert!(got[gi..].iter().all(|(r, ..)| matches!(r, Read::Rapid(_))), "only the last move up after the plan's moves");
}

/// Numbers: at most the decimals asked for, no trailing zeros, no negative zero.
#[test]
fn gcode_numbers_are_short_and_exact() {
    use crate::post::num;
    assert_eq!(num(1.0, 4), "1");
    assert_eq!(num(-0.00001, 4), "0");
    assert_eq!(num(2.345_67, 4), "2.3457");
    assert_eq!(num(-12.5, 4), "-12.5");
    assert_eq!(num(1200.0, 4), "1200");
}

/// Comments: in parentheses, none inside, plain ASCII.
#[test]
fn gcode_comments_are_plain_ascii() {
    use crate::post::comment;
    assert_eq!(comment("Ø6 90° chamfer mill (T9)"), "(D6 90 deg chamfer mill [T9])");
    assert_eq!(comment("Fräs\nteil"), "(Fr?s teil)");
}

/// Two NEMA 23 flanges simulated on 0.25 mm cells (drilled holes, bores, end-milled countersinks,
/// recess and chamfers, contours, separation): nothing cut into them, no rapid through
/// material, nothing below the spoilboard allowance, both cut free.
#[test]
fn simulated_flanges_are_cut_clean_and_free() {
    let part = &expected()["parts"][0];
    let read =
        solidcraft_kernel::step_import(&std::fs::read_to_string(fixtures().join(part["design"].as_str().unwrap().replace(".sc", ".step"))).unwrap())
            .unwrap();
    let body = read.bodies[0].body.clone();
    let job = crate::job::Job::from_json(&format!(
        r#"{{"stock": {{"size": [220, 120], "thickness": 10}}, "tools": {TOOLS}, "parts": [{{"body": "F", "quantity": 2}}]}}"#
    ))
    .unwrap();
    let plan = crate::plan::plan_job(&job, std::slice::from_ref(&body)).unwrap();
    let (r, _) = crate::sim::simulate(&job, &[body], &plan, 0.25).unwrap();
    assert!(r.gouges.is_empty(), "{:?}", r.gouges);
    assert!(r.rapid_hits.is_empty(), "{:?}", r.rapid_hits);
    assert!(r.spoilboard.is_empty(), "{:?}", r.spoilboard);
    assert!(r.parts.len() == 2 && r.parts.iter().all(|p| p.free), "{:?}", r.parts);
    assert!(r.removed_volume > 0.0);
}

/// The program's tool table: one `(TOOL T<n> key=value ... name=...)` line per tool, in order of
/// first use, before any G-code; each line's values are the job's tool's (kind, diameter,
/// flutes, flute length, stepdown and stepover or tip angle and peck, rpm, feeds) and `z_min` the
/// lowest point of that tool's moves in the plan, worked out here from the moves themselves.
#[test]
fn the_program_opens_with_a_tool_table() {
    use crate::toolpath::Move;
    let part = &expected()["parts"][0];
    let read =
        solidcraft_kernel::step_import(&std::fs::read_to_string(fixtures().join(part["design"].as_str().unwrap().replace(".sc", ".step"))).unwrap())
            .unwrap();
    let job =
        crate::job::Job::from_json(&format!(r#"{{"stock": {{"size": [120, 120], "thickness": 10}}, "tools": {TOOLS}, "parts": [{{"body": "F"}}]}}"#))
            .unwrap();
    let plan = crate::plan::plan_job(&job, &[read.bodies[0].body.clone()]).unwrap();
    let program = crate::post::linuxcnc(&job, &plan.ops, "flange", &plan.warnings).unwrap();
    let first_g = program.lines().position(|l| l.starts_with('G')).unwrap();
    let table: Vec<&str> = program.lines().take(first_g).filter(|l| l.starts_with("(TOOL ")).collect();
    let mut order: Vec<u32> = Vec::new();
    for op in &plan.ops {
        if !order.contains(&op.tool) {
            order.push(op.tool);
        }
    }
    assert_eq!(table.len(), order.len(), "{table:?}");
    for (line, n) in table.iter().zip(&order) {
        let body = line.trim_start_matches('(').trim_end_matches(')');
        let (head, name) = body.split_once(" name=").unwrap();
        let mut words = head.split(' ');
        assert_eq!(words.next(), Some("TOOL"));
        assert_eq!(words.next(), Some(format!("T{n}").as_str()));
        let kv: std::collections::HashMap<&str, &str> = words.map(|w| w.split_once('=').unwrap()).collect();
        let t = job.tool(*n).unwrap();
        let num = |k: &str| kv[k].parse::<f64>().unwrap();
        assert_eq!(
            kv["kind"],
            match t.kind {
                crate::job::ToolKind::Flat => "flat",
                crate::job::ToolKind::Drill => "drill",
                crate::job::ToolKind::Chamfer => "chamfer",
            }
        );
        assert_eq!((num("D"), num("flutes") as u32, num("flute_length")), (t.diameter, t.flutes, t.flute_length));
        assert_eq!((num("rpm"), num("feed"), num("plunge")), (t.rpm, t.feed, t.plunge));
        match t.kind {
            crate::job::ToolKind::Flat => assert!((num("stepdown") - t.stepdown()).abs() < 1e-4 && (num("stepover") - t.stepover()).abs() < 1e-4),
            _ => {
                assert_eq!(num("tip_angle"), t.tip_angle());
                assert_eq!(kv.get("peck").map(|p| p.parse::<f64>().unwrap()), t.peck);
            }
        }
        let low = plan
            .ops
            .iter()
            .filter(|o| o.tool == *n)
            .flat_map(|o| &o.moves)
            .map(|m| match m {
                Move::Drill { bottom, .. } => *bottom,
                Move::Rapid { .. } => f64::INFINITY,
                _ => m.end()[2],
            })
            .fold(f64::INFINITY, f64::min);
        assert!((num("z_min") - low).abs() < 1e-4, "T{n}: z_min {} vs {low}", kv["z_min"]);
        assert!(!name.is_empty());
    }
}

/// Points along `moves` from `start`, arcs and helices in pieces of at most half a degree and
/// within 1e-5 mm of the curve.
fn path_points(start: [f64; 3], moves: &[crate::toolpath::Move]) -> Vec<[f64; 3]> {
    use crate::toolpath::Move;
    let mut out = vec![start];
    let mut at = start;
    for m in moves {
        if let Move::Arc { to, center: c, ccw, .. } = m {
            let a0 = (at[1] - c[1]).atan2(at[0] - c[0]);
            let r = (at[0] - c[0]).hypot(at[1] - c[1]);
            let mut sweep = (to[1] - c[1]).atan2(to[0] - c[0]) - a0;
            if *ccw {
                sweep = sweep.rem_euclid(std::f64::consts::TAU);
            } else {
                sweep = -(-sweep).rem_euclid(std::f64::consts::TAU);
            }
            if sweep.abs() < 1e-12 {
                sweep = if *ccw { std::f64::consts::TAU } else { -std::f64::consts::TAU };
            }
            // pieces short enough to stay within 1e-5 of the arc
            let step = 0.5f64.to_radians().min(2.0 * (1.0 - 1e-5 / r.max(1e-3)).clamp(-1.0, 1.0).acos());
            let n = (sweep.abs() / step).ceil().clamp(1.0, 1e6) as usize;
            for k in 1..=n {
                let t = k as f64 / n as f64;
                let a = a0 + sweep * t;
                out.push([c[0] + r * a.cos(), c[1] + r * a.sin(), at[2] + (to[2] - at[2]) * t]);
            }
        } else if let Move::Drill { at: c, bottom, .. } = m {
            out.push([c[0], c[1], *bottom]);
        }
        at = m.end();
        out.push(at);
    }
    out
}

/// The largest distance from a point of `a` to the path through `b`.
fn furthest(a: &[[f64; 3]], b: &[[f64; 3]]) -> f64 {
    let seg = |p: &[f64; 3], s: &[f64; 3], e: &[f64; 3]| {
        let d: Vec<f64> = (0..3).map(|k| e[k] - s[k]).collect();
        let l2: f64 = d.iter().map(|x| x * x).sum();
        let t = if l2 > 0.0 { ((0..3).map(|k| (p[k] - s[k]) * d[k]).sum::<f64>() / l2).clamp(0.0, 1.0) } else { 0.0 };
        (0..3).map(|k| (s[k] + t * d[k] - p[k]).powi(2)).sum::<f64>().sqrt()
    };
    // pieces of `b` by 1 mm cell of their box
    let mut cells: std::collections::HashMap<(i64, i64), Vec<usize>> = std::collections::HashMap::new();
    for i in 1..b.len() {
        let (s, e) = (b[i - 1], b[i]);
        for x in (s[0].min(e[0]) - 0.1).floor() as i64..=(s[0].max(e[0]) + 0.1).floor() as i64 {
            for y in (s[1].min(e[1]) - 0.1).floor() as i64..=(s[1].max(e[1]) + 0.1).floor() as i64 {
                cells.entry((x, y)).or_default().push(i);
            }
        }
    }
    a.iter()
        .map(|p| {
            cells
                .get(&(p[0].floor() as i64, p[1].floor() as i64))
                .map(|ix| ix.iter().map(|&i| seg(p, &b[i - 1], &b[i])).fold(f64::INFINITY, f64::min))
                .unwrap_or(f64::INFINITY)
        })
        .fold(0.0, f64::max)
}

#[test]
fn fitted_arcs_stay_on_the_planned_path_with_far_fewer_moves() {
    use crate::toolpath::Move;
    let part = &expected()["parts"][0];
    let read =
        solidcraft_kernel::step_import(&std::fs::read_to_string(fixtures().join(part["design"].as_str().unwrap().replace(".sc", ".step"))).unwrap())
            .unwrap();
    let job = crate::job::Job::from_json(&format!(
        r#"{{"stock": {{"size": [220, 120], "thickness": 10}}, "tools": {TOOLS}, "post": {{"fit_tolerance": 0.005}}, "parts": [{{"body": "F", "quantity": 2}}]}}"#
    ))
    .unwrap();
    assert!(job.post.fit_arcs, "on by default");
    let plan = crate::plan::plan_job(&job, &[read.bodies[0].body.clone()]).unwrap();
    let tol = job.post.fit_tolerance;
    let (mut planned, mut fitted, mut short_planned, mut short_fitted, mut arcs) = (0, 0, 0, 0, 0);
    let short = |s: [f64; 3], ms: &[Move]| {
        let mut at = s;
        let mut n = 0;
        for m in ms {
            let e = m.end();
            if !matches!(m, Move::Drill { .. }) && (0..3).map(|k| (e[k] - at[k]).powi(2)).sum::<f64>().sqrt() < 0.05 {
                n += 1;
            }
            at = e;
        }
        n
    };
    let mut start = [0.0, 0.0, job.strategy.clearance];
    for op in &plan.ops {
        let fit = crate::post::fit(&op.moves, tol);
        // the same ends: the operation starts and stops where planned
        assert_eq!(fit.last().map(|m| m.end()), op.moves.last().map(|m| m.end()), "{}", op.name);
        let (p, q) = (path_points(start, &op.moves), path_points(start, &fit));
        let (there, back) = (furthest(&q, &p), furthest(&p, &q));
        assert!(there <= tol + 1e-4 && back <= tol + 1e-4, "{}: {there} {back} from the planned path", op.name);
        planned += op.moves.len();
        fitted += fit.len();
        short_planned += short(start, &op.moves);
        short_fitted += short(start, &fit);
        arcs += fit.iter().filter(|m| matches!(m, Move::Arc { .. })).count() - op.moves.iter().filter(|m| matches!(m, Move::Arc { .. })).count();
        start = op.moves.last().map(|m| m.end()).unwrap_or(start);
    }
    assert!(arcs > 50, "{arcs} arcs fitted");
    assert!(fitted * 2 < planned, "{fitted} of {planned} moves");
    assert!(short_fitted * 4 < short_planned.max(1), "{short_fitted} of {short_planned} short moves");
    // and the program carries them: as many feed and arc moves as the fitted operations
    let program = crate::post::linuxcnc(&job, &plan.ops, "flanges", &plan.warnings).unwrap();
    let (got, _) = read_program(&program);
    assert!(got.len() <= fitted + 4 * plan.ops.len() + 100, "{} read back, {fitted} fitted", got.len());
    let (unfitted, _) = read_program(
        &crate::post::linuxcnc(
            &crate::job::Job { post: crate::job::Post { fit_arcs: false, ..job.post.clone() }, ..job.clone() },
            &plan.ops,
            "flanges",
            &plan.warnings,
        )
        .unwrap(),
    );
    assert!(got.len() * 2 < unfitted.len());
}
