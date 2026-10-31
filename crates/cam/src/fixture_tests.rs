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
        r#"{{"stock": {{"size": [600, 400], "thickness": {thickness}}}, "tools": {TOOLS}, "parts": [{{"body": "P"}}]}}"#
    ))
    .unwrap();
    let plan = crate::plan::plan_part(&f, &job, "P").unwrap();
    (f, plan)
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
    let job =
        crate::job::Job::from_json(&format!(r#"{{"stock": {{"size": [600, 400], "thickness": 10}}, "tools": {TOOLS}, "parts": [{{"body": "P"}}]}}"#))
            .unwrap();
    let pl = Placement { part: 0, label: "P".into(), copy: 1, quarter_turns: 1, offset: [100.0, 50.0], bounds: [[0.0; 2]; 2] };
    let ops = plan_sheet(&job, &[(&f, &plan)], &[pl]);
    assert_eq!(ops.len(), plan.ops.len());
    assert!(ops.iter().all(|o| o.name.starts_with("P #2: ")), "{:?}", ops.iter().map(|o| &o.name).collect::<Vec<_>>());
    assert!(ops.windows(2).all(|w| w[0].kind.rank() <= w[1].kind.rank()), "in machining order");
    let drill = ops.iter().find(|o| o.kind == crate::toolpath::OpKind::Drilling).unwrap();
    for (m, orig) in drill.moves.iter().zip(&plan.ops.iter().find(|o| o.kind == crate::toolpath::OpKind::Drilling).unwrap().moves) {
        let (Move::Drill { at, r, .. }, Move::Drill { at: a0, r: r0, .. }) = (m, orig) else { continue };
        assert!((at[0] - (100.0 - a0[1])).abs() < 1e-9 && (at[1] - (50.0 + a0[0])).abs() < 1e-9);
        assert!((r - (r0 - 10.0)).abs() < 1e-9 && (r - 1.0).abs() < 1e-9, "1 mm above the sheet's top");
    }
}

/// The panel's label recess (70 × 14, floor at 2.2) is roughed at 2.4 before its floor is
/// finished: roughing passes at 2.4 inside it, 3.2 or more from its walls.
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
        assert!(d <= -(3.2 - crate::plan::PATH_TOL - 1e-6), "roughing at {p:?} is {} from the recess wall", -d);
    }
}

/// The flange sheet's preview, checked pixel by pixel where the geometry says what must be
/// there: the profile's dark line 3 mm outside a flange's straight edge, a drilled hole's red
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
        r#"{{"stock": {{"size": [220, 220], "thickness": 10}}, "nest": {{"margin": 10, "spacing": 10}}, "tools": {TOOLS}, "parts": [{{"body": "F", "quantity": 4}}]}}"#
    ))
    .unwrap();
    let plan = crate::plan::plan_job(&job, &[read.bodies[0].body.clone()]).unwrap();
    assert!(plan.warnings.is_empty(), "{:?}", plan.warnings);
    assert_eq!(plan.nesting.placements.len(), 4);
    let outlines = plan.placed_outlines();
    let scene = Scene { sheet: [220.0, 220.0], margin: 10.0, outlines: &outlines, ops: &plan.ops, clearance: 5.0 };

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
