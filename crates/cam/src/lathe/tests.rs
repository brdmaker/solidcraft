use solidcraft_geom::Vec3;
use solidcraft_geom::{Loop2, Plane, Region2, Vec2};
use solidcraft_kernel::{Body, BoolOp, boolean, cylinder};

use super::job::LatheJob;
use super::plan::{LatheOpKind, TipMove, plan};
use super::profile::read;
use super::sim::simulate;

fn v(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, y, z)
}

fn union(a: Body, b: Body) -> Body {
    boolean(&a, &b, BoolOp::Union).unwrap().unwrap()
}

fn cut(a: Body, b: Body) -> Body {
    boolean(&a, &b, BoolOp::Cut).unwrap().unwrap()
}

/// The solid turned from the half section `pts` ((r, z) corners, the axis among them) about Z.
fn turned(pts: &[(f64, f64)]) -> Body {
    let outer = Loop2::polygon(&pts.iter().map(|(r, z)| Vec2::new(*r, *z)).collect::<Vec<_>>());
    let mut b =
        solidcraft_kernel::revolve(&Plane::YZ, &[Region2 { outer, holes: vec![] }], Vec2::new(0.0, 0.0), Vec2::new(0.0, 1.0), std::f64::consts::TAU)
            .unwrap();
    b.remove(0)
}

/// A cylinder of radius `r` from z `a` to `b` about Z.
fn cyl(r: f64, a: f64, b: f64) -> Body {
    cylinder(v(0.0, 0.0, a), v(0.0, 0.0, 1.0), r, b - a).unwrap()
}

fn job(diameter: f64, extra: &str) -> LatheJob {
    LatheJob::from_json(&format!(r#"{{"stock": {{"diameter": {diameter}, "material": "aluminium"}}, "part": {{"body": "P"}} {extra}}}"#)).unwrap()
}

/// The Emco Compact 5's library: T1 roughing, an 80° insert at 95°: its leading edge at 180 − 95
/// = 85° from +Z, its trailing edge 80° round at 5°, so it falls at most 5 − 2 = 3° toward the
/// chuck; T2 finishing, 35° at 93°: 87° and 52°, falling at most 50°. Roughing takes T1,
/// finishing the tool reaching down the steepest, T2. The machine: 3200 rpm, stations T1–T6,
/// 20 in the chuck.
#[test]
fn the_emco_library_and_its_tools_reach() {
    let j = job(22.0, "");
    let (t1, t2) = (j.tool(1).unwrap(), j.tool(2).unwrap());
    assert_eq!((t1.lead_angle(), t1.trail_angle(), t1.descent_deg()), (85.0, 5.0, 3.0));
    assert_eq!((t2.lead_angle(), t2.trail_angle(), t2.descent_deg()), (87.0, 52.0, 50.0));
    assert_eq!((j.rough_tool().unwrap().number, j.finish_tool().unwrap().number), (1, 2));
    assert_eq!((j.machine.max_rpm, j.machine.turret.clone(), j.stock.in_chuck), (3200.0, vec![1, 2, 3, 4, 5, 6], 20.0));
    assert!(LatheJob::from_json(r#"{"stock": {"diameter": -1}, "part": {"body": "P"}}"#).is_err());
    assert!(LatheJob::from_json(r#"{"stock": {"diameter": 20, "material": "wood"}, "part": {"body": "P"}}"#).is_err(), "no speed for wood");
}

/// A Ø20 × 30 shaft with a Ø12 × 20 spigot in front reads as radius 10 to z 30 and 6 to z 50,
/// solid to the axis; turned the other way round (`-z` the front), the spigot is at the back.
#[test]
fn the_profile_is_read_from_the_solid() {
    let b = union(cyl(10.0, 0.0, 30.0), cyl(6.0, 30.0, 50.0));
    let p = read(&b, super::job::Front::PlusZ).unwrap();
    assert!((p.length - 50.0).abs() < 1e-9);
    for (z, r) in [(1.0, 10.0), (29.9, 10.0), (30.1, 6.0), (49.9, 6.0)] {
        assert!((p.od_at(z) - r).abs() < 0.002, "r at z {z}: {}", p.od_at(z));
    }
    assert!(p.id.iter().all(|r| *r == 0.0));
    let q = read(&b, super::job::Front::MinusZ).unwrap();
    assert!((q.od_at(5.0) - 6.0).abs() < 0.002 && (q.od_at(45.0) - 10.0).abs() < 0.002);
    // a bore: a Ø8 hole through
    let tube = cut(cyl(10.0, 0.0, 30.0), cyl(4.0, -1.0, 31.0));
    let t = read(&tube, super::job::Front::PlusZ).unwrap();
    assert!((t.id[t.index(15.0)] - 4.0).abs() < 0.002, "{}", t.id[t.index(15.0)]);
}

/// The tip positions of an operation's feeds.
fn feeds(ops: &[super::plan::LatheOp], kind: LatheOpKind) -> Vec<[f64; 2]> {
    ops.iter().filter(|o| o.kind == kind).flat_map(|o| o.moves.iter()).filter(|&m| matches!(m, TipMove::Feed(_))).map(|m| m.to()).collect()
}

/// The shaft above from a Ø22 bar (aluminium): faced in one pass to 0.1 in front of the finished
/// face (0.5 allowance less 0.1 left, within the 0.5 depth); roughed from the bar's radius 11 down
/// to the Ø12 spigot's 6 + 0.2 left, the nose centre (R0.4) from 11.4 to 6.6 in ten even steps
/// of 0.48; finished at Ø20 and Ø12 exactly (tip radius 10 and 6). Simulated, nothing is cut
/// into the part, nothing left on it, no rapid through the bar, nothing at the jaws. The bar: 50
/// + 0.5 + 3 + 2 + 20 = 75.5, rounded up to 76, 56 out of the chuck.
#[test]
fn a_stepped_shaft_is_faced_roughed_and_finished() {
    let b = union(cyl(10.0, 0.0, 30.0), cyl(6.0, 30.0, 50.0));
    let j = job(22.0, "");
    let pl = plan(&j, read(&b, super::job::Front::PlusZ).unwrap()).unwrap();
    assert!(pl.warnings.is_empty(), "{:?}", pl.warnings);
    assert_eq!((pl.setup.length, pl.setup.stick_out, pl.setup.jaws), (76.0, 56.0, 0.5 - 56.0));
    let face = feeds(&pl.ops, LatheOpKind::Facing);
    assert!(face.iter().any(|p| (p[1] - 0.1).abs() < 1e-9 && p[0] < 0.0), "faced to Z0.1 past the centre: {face:?}");
    let rough = feeds(&pl.ops, LatheOpKind::Roughing);
    let mut levels: Vec<f64> = rough.iter().map(|p| (p[0] * 1e6).round() / 1e6).collect();
    levels.sort_by(f64::total_cmp);
    levels.dedup();
    for k in 1..=10 {
        let tip = 11.4 - 0.48 * k as f64 - 0.4;
        assert!(levels.iter().any(|l| (l - tip).abs() < 1e-6), "a pass at tip radius {tip:.3}: {levels:?}");
    }
    assert!(rough.iter().all(|p| p[0] >= 6.2 - 1e-6), "nothing below 6 + 0.2");
    let fin = feeds(&pl.ops, LatheOpKind::Finishing);
    assert!(fin.iter().any(|p| (p[0] - 10.0).abs() < 1e-6 && p[1] < -25.0), "Ø20");
    assert!(fin.iter().any(|p| (p[0] - 6.0).abs() < 1e-6 && p[1] > -15.0), "Ø12");
    assert!(fin.iter().all(|p| p[0] >= 6.0 - 1e-6 || p[1] > -0.2 - 1e-6), "nothing under Ø12 behind the face and its corner");
    let (r, _) = simulate(&j, &pl).unwrap();
    assert!(r.gouges.is_empty(), "{:?}", r.gouges);
    assert!(r.left.is_empty(), "{:?}", r.left);
    assert!(r.rapid_hits.is_empty() && r.jaw_hits == 0, "{:?} {}", r.rapid_hits, r.jaw_hits);
    // the program: diameters, the tools on their tips, constant surface speed under 3200 rpm
    let prog = super::post::linuxcnc(&j, &pl, "shaft").unwrap();
    for w in ["G18 G7 G21 G90 G40 G80 G95", "T1 M6 G43", "T2 M6 G43", "G96 S150 D3200 M3", "G96 S180 D3200 M3", "F0.1", "F0.05", "M30"] {
        assert!(prog.contains(w), "{w}");
    }
    assert!(prog.lines().any(|l| l.contains("X20 ") || l.ends_with("X20") || l.contains("X20.0")), "Ø20 as a diameter");
    assert!(prog.lines().any(|l| l.starts_with("X12 ") || l.contains(" X12 ") || l.contains("G1 X12 ")), "Ø12 as a diameter");
    assert!(!prog.split_whitespace().any(|w| w == "G2" || w == "G3"), "lines only");
}

/// A Ø20 × 10 head on a Ø14 shank, joined by a cone narrowing toward the chuck: from r 10 at z 30
/// to r 7 at z 27 (45°). T1 can't
/// follow a 45° fall (it falls 3°), T2 can (50°): finished, nothing left, no warning. At 60°
/// (r 10 to 7 over 3 tan 30° = 1.732) T2 can't either: the part is warned of and the simulation
/// finds material left there, and cuts nothing into it.
#[test]
fn slopes_falling_toward_the_chuck_take_the_finishing_tool_or_are_reported() {
    for (run, steep) in [(3.0, false), (3.0 * (30f64).to_radians().tan(), true)] {
        let b = turned(&[(0.0, 0.0), (7.0, 0.0), (7.0, 30.0 - run), (10.0, 30.0), (10.0, 40.0), (0.0, 40.0)]);
        let j = job(22.0, "");
        let pl = plan(&j, read(&b, super::job::Front::PlusZ).unwrap()).unwrap();
        let (r, _) = simulate(&j, &pl).unwrap();
        assert!(r.gouges.is_empty(), "{steep}: {:?}", r.gouges);
        if steep {
            assert!(pl.warnings.iter().any(|w| w.contains("can't reach")), "{:?}", pl.warnings);
            assert!(!r.left.is_empty());
        } else {
            assert!(pl.warnings.is_empty(), "{:?}", pl.warnings);
            assert!(r.left.is_empty(), "{:?}", r.left);
        }
    }
}

/// A Ø20 shaft with a ring groove (Ø18 × 1.1): the turning tools can't cut it: it is reported
/// (with the groove's Z span) and left, never cut into.
#[test]
fn a_ring_groove_is_reported_not_cut() {
    let b = cut(cyl(10.0, 0.0, 40.0), cut(cyl(11.0, 20.0, 21.1), cyl(9.0, 19.0, 22.0)));
    let j = job(22.0, "");
    let pl = plan(&j, read(&b, super::job::Front::PlusZ).unwrap()).unwrap();
    let w: Vec<&String> = pl.warnings.iter().filter(|w| w.contains("can't reach")).collect();
    assert_eq!(w.len(), 1, "{:?}", pl.warnings);
    let (r, _) = simulate(&j, &pl).unwrap();
    assert!(r.gouges.is_empty(), "{:?}", r.gouges);
    assert!(r.left.iter().any(|p| p.z[0] > -20.5 && p.z[1] < -18.4), "{:?}", r.left);
}

/// The six lathe test parts (tests/cam/lathe) from their bars: each planned and simulated with
/// nothing cut into it, no rapid through material, nothing at the jaws; every piece of material
/// left meets a span a warning names (the ring grooves, the undercut, the V-groove, the grip
/// groove, the bores: a wedge the insert can't reach joins the round its nose leaves in the
/// groove's far corner).
#[test]
fn the_lathe_test_parts_turn_clean() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/cam/lathe");
    let parts: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(dir.join("parts.json")).unwrap()).unwrap();
    for p in parts["parts"].as_array().unwrap() {
        let name = p["design"].as_str().unwrap();
        let read_step = solidcraft_kernel::step_import(&std::fs::read_to_string(dir.join(name.replace(".sc", ".step"))).unwrap()).unwrap();
        let body = &read_step.bodies[0].body;
        let material = p["material"].as_str().unwrap();
        let j = LatheJob::from_json(&format!(
            r#"{{"stock": {{"diameter": {}, "material": "{material}"}}, "part": {{"body": "P"}}}}"#,
            p["stock_bar_diameter"]
        ))
        .unwrap();
        let pl = plan(&j, read(body, super::job::Front::PlusZ).unwrap()).unwrap();
        let (r, _) = simulate(&j, &pl).unwrap();
        assert!(r.gouges.is_empty(), "{name}: {:?}", r.gouges);
        assert!(r.rapid_hits.is_empty() && r.jaw_hits == 0, "{name}: {:?} {}", r.rapid_hits, r.jaw_hits);
        // a part whose profile never meets the axis is hollow: its bore is warned of once
        let hollow = !p["profile"].as_array().unwrap().iter().any(|seg| seg.as_array().unwrap()[1..].iter().any(|q| q[0].as_f64().unwrap() == 0.0));
        let bores = pl.warnings.iter().filter(|w| w.contains("a bore")).count();
        assert_eq!(bores, usize::from(hollow), "{name}: {:?}", pl.warnings);
        let spans: Vec<(f64, f64)> = pl
            .warnings
            .iter()
            .filter_map(|w| {
                let rest = w.strip_prefix("Z ")?;
                let (a, rest) = rest.split_once(" to ")?;
                let (b, _) = rest.split_once(':')?;
                Some((a.parse::<f64>().ok()?, b.parse::<f64>().ok()?))
            })
            .collect();
        for piece in &r.left {
            assert!(
                spans.iter().any(|(a, b)| piece.z[1] >= a - 0.5 && piece.z[0] <= b + 0.5),
                "{name}: material left {piece:?} with no warning: {:?}",
                pl.warnings
            );
        }
    }
}
