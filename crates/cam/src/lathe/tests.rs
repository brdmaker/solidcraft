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
/// 20 in the chuck. T3 a 1.0 blade reaching 12, T4 a 60° threading tool, T5 a Ø8 drill, T6 a
/// boring bar from Ø7, and Ø5 and Ø10 drills (T7, T8) changed by hand. Threads: M12 is coarse,
/// 1.75; M12x1.25 fine; M13 has no coarse pitch, and a pitch of half the diameter is no thread.
#[test]
fn the_emco_library_and_its_tools_reach() {
    let j = job(22.0, "");
    let (t1, t2) = (j.tool(1).unwrap(), j.tool(2).unwrap());
    assert_eq!((t1.lead_angle(), t1.trail_angle(), t1.descent_deg()), (85.0, 5.0, 3.0));
    assert_eq!((t2.lead_angle(), t2.trail_angle(), t2.descent_deg()), (87.0, 52.0, 50.0));
    assert_eq!((j.rough_tool().unwrap().number, j.finish_tool().unwrap().number), (1, 2));
    assert_eq!((j.machine.max_rpm, j.machine.turret.clone(), j.stock.in_chuck), (3200.0, vec![1, 2, 3, 4, 5, 6], 20.0));
    use super::job::LatheToolKind::*;
    let kinds: Vec<_> = j.tools.iter().map(|t| (t.number, t.kind)).collect();
    assert_eq!(kinds, [(1, Turning), (2, Turning), (3, Grooving), (4, Threading), (5, Drill), (6, Boring), (7, Drill), (8, Drill)]);
    let t = |n| j.tool(n).unwrap();
    assert_eq!(
        (t(3).blade_width(), t(3).reach(), t(4).point_angle(), t(5).diameter, t(5).point_angle(), t(6).min_bore),
        (1.0, 12.0, 60.0, Some(8.0), 118.0, Some(7.0))
    );
    let size = |d: &str| super::job::ThreadSpec { designation: d.into(), z_from: 0.0, z_to: 1.0 }.size();
    assert_eq!((size("M12").unwrap(), size("M12x1.25").unwrap()), ((12.0, 1.75), (12.0, 1.25)));
    assert!(size("M13").is_err() && size("M4x2").is_err() && size("12").is_err());
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
/// to r 7 at z 27 (45°). T1 can't follow a 45° fall (it falls 3°), T2 can (50°): finished,
/// nothing left, no warning. At 60° (r 10 to 7 over 3 tan 30° = 1.732) T2 can't either, nor the
/// square grooving blade: the part is warned of and the simulation finds material left there,
/// and cuts nothing into it.
#[test]
fn slopes_falling_toward_the_chuck_take_the_finishing_tool_or_are_reported() {
    for (run, steep) in [(3.0, false), (3.0 * (30f64).to_radians().tan(), true)] {
        let b = turned(&[(0.0, 0.0), (7.0, 0.0), (7.0, 30.0 - run), (10.0, 30.0), (10.0, 40.0), (0.0, 40.0)]);
        let j = job(22.0, "");
        let pl = plan(&j, read(&b, super::job::Front::PlusZ).unwrap()).unwrap();
        let (r, _) = simulate(&j, &pl).unwrap();
        assert!(r.gouges.is_empty(), "{steep}: {:?}", r.gouges);
        if steep {
            assert!(pl.warnings.iter().any(|w| w.contains("reach")), "{:?}", pl.warnings);
            assert!(!r.left.is_empty());
        } else {
            assert!(pl.warnings.is_empty(), "{:?}", pl.warnings);
            assert!(r.left.is_empty(), "{:?}", r.left);
        }
    }
}

/// A Ø20 × 40 shaft with a ring groove Ø18 × 1.1 at z 20 to 21.1 (work Z −20 to −18.9): the
/// turning tools can't get into it, so the 1.0 blade (T3) plunges it: wider than the blade, at
/// the groove's front (its corner at Z −18.9 − 1 = −19.9) and then at its back (Z −20), each
/// in one peck of 1.0 from r 10 to the groove's floor at r 9. Nothing is left, nothing warned,
/// and it is parted off behind the part (the blade's corner at Z −41) down past the axis.
#[test]
fn a_ring_groove_is_cut_with_the_blade() {
    let b = cut(cyl(10.0, 0.0, 40.0), cut(cyl(11.0, 20.0, 21.1), cyl(9.0, 19.0, 22.0)));
    let j = job(22.0, "");
    let pl = plan(&j, read(&b, super::job::Front::PlusZ).unwrap()).unwrap();
    assert!(pl.warnings.is_empty(), "{:?}", pl.warnings);
    let g = feeds(&pl.ops, LatheOpKind::Grooving);
    let mut at: Vec<f64> = g.iter().filter(|p| (p[0] - 9.0).abs() < 0.003).map(|p| p[1]).collect();
    at.sort_by(f64::total_cmp);
    assert!(at.len() >= 2 && (at[0] + 20.0).abs() < 0.01 && (at[at.len() - 1] + 19.9).abs() < 0.01, "plunges at {at:?}: {g:?}");
    assert!(g.iter().all(|p| p[0] >= 9.0 - 0.003 && p[1] >= -20.0 - 0.01 && p[1] + 1.0 <= -18.9 + 0.01), "the blade in the groove: {g:?}");
    let part = feeds(&pl.ops, LatheOpKind::Parting);
    assert!(part.iter().all(|p| (p[1] + 41.0).abs() < 1e-9) && part.iter().any(|p| (p[0] + 0.2).abs() < 1e-9), "{part:?}");
    let (r, _) = simulate(&j, &pl).unwrap();
    assert!(r.gouges.is_empty(), "{:?}", r.gouges);
    assert!(r.left.is_empty(), "{:?}", r.left);
    assert!(r.rapid_hits.is_empty() && r.jaw_hits == 0, "{:?} {}", r.rapid_hits, r.jaw_hits);
}

/// A Ø20 × 30 tube with a Ø12 bore through, from a Ø22 aluminium bar. The largest drill the
/// turret holds that fits the bore is T5's Ø8 (T8's Ø10 is changed by hand): 50 m/min on Ø8 is
/// 50000 / 8π = 1989 rpm, under its 2000; its 118° point stands 4 / tan 59° = 2.403 ahead of
/// its corners, so it drills through the part (30) and the parting allowance (3) to Z −35.903 in
/// pecks of 4 (Z −4, −8 … −32). T6 (Ø7 and up) bores it to Ø12 (its tip at radius 6), the bar
/// is parted off down to 0.5 inside the bore (r 5.5), and the simulation leaves the tube exact.
#[test]
fn a_bore_is_drilled_bored_and_parted_into() {
    let b = cut(cyl(10.0, 0.0, 30.0), cyl(6.0, -1.0, 31.0));
    let j = job(22.0, "");
    let pl = plan(&j, read(&b, super::job::Front::PlusZ).unwrap()).unwrap();
    assert!(pl.warnings.is_empty(), "{:?}", pl.warnings);
    let kinds: Vec<LatheOpKind> = pl.ops.iter().map(|o| o.kind).collect();
    use LatheOpKind::*;
    assert_eq!(kinds, vec![Facing, Drilling, Roughing, Boring, Finishing, Parting]);
    let drill = pl.ops.iter().find(|o| o.kind == Drilling).unwrap();
    assert_eq!((drill.tool, drill.spindle), (5, super::plan::Spindle::Rpm(1989.0)));
    let d = feeds(&pl.ops, Drilling);
    let bottom = -30.0 - 3.0 - 4.0 / 59f64.to_radians().tan() - 0.5;
    assert!((bottom + 35.903).abs() < 1e-3);
    assert_eq!(d.len(), 9, "{d:?}");
    for (k, p) in d.iter().enumerate() {
        let z = if k < 8 { -4.0 * (k + 1) as f64 } else { bottom };
        assert!(p[0] == 0.0 && (p[1] - z).abs() < 1e-9, "peck {k}: {p:?}");
    }
    let bore = feeds(&pl.ops, Boring);
    assert!(bore.iter().any(|p| (p[0] - 6.0).abs() < 0.003 && p[1] < -25.0), "bored to Ø12: {bore:?}");
    // (but round the bore's mouth: the nose's centre wraps its corner, the tip a radius out)
    assert!(bore.iter().all(|p| p[0] <= 6.0 + 0.003 || p[1] > -0.2), "nothing past Ø12 behind the mouth");
    let part = feeds(&pl.ops, Parting);
    assert!(part.iter().all(|p| (p[1] + 31.0).abs() < 1e-9));
    let low = part.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    assert!((low - 5.5).abs() < 0.003, "parted to r {low}");
    let (r, _) = simulate(&j, &pl).unwrap();
    assert!(r.gouges.is_empty(), "{:?}", r.gouges);
    assert!(r.left.is_empty(), "{:?}", r.left);
    assert!(r.rapid_hits.is_empty() && r.jaw_hits == 0, "{:?} {}", r.rapid_hits, r.jaw_hits);
    let prog = super::post::linuxcnc(&j, &pl, "tube").unwrap();
    for w in ["T5 M6 G43", "G97 S1989 M3", "G0 X0 Z1.5", "G1 Z-4", "T6 M6 G43", "G96 S120 D3200 M3", "T3 M6 G43", "G96 S100 D3200 M3"] {
        assert!(prog.lines().any(|l| l == w || l.starts_with(&format!("{w} "))), "{w}");
    }
}

/// An M12 thread over a Ø12 spigot (z 23 to 40, the front) of a Ø20 × 20 shaft, with a Ø9.6 × 3
/// relief groove before the shoulder. ISO coarse: P 1.75, the depth 0.61343 P = 1.0735, the root
/// at radius 4.9265 (Ø9.853). In passes of at most 0.25 deep at first, degressive: (1.0735 /
/// 0.25)² = 18.4, so 19 passes at depth 1.0735 √(k / 19) and a spring pass, each a G33 from two
/// pitches in front of the face (Z3.5) to half a pitch into the relief (Z 23 − 0.875 − 40 =
/// −17.875), at T4's fixed 400 rpm (60 m/min on Ø12 would be 1592); going in in the air, rapid.
#[test]
fn a_thread_is_cut_in_passes() {
    let b = union(union(cyl(10.0, 0.0, 20.0), cyl(4.8, 19.0, 24.0)), cyl(6.0, 23.0, 40.0));
    let j = LatheJob::from_json(r#"{"stock": {"diameter": 22, "material": "aluminium"}, "part": {"body": "P", "threads": [{"designation": "M12", "z_from": 23, "z_to": 40}]}}"#).unwrap();
    let pl = plan(&j, read(&b, super::job::Front::PlusZ).unwrap()).unwrap();
    assert!(!pl.warnings.iter().any(|w| w.contains("thread")), "{:?}", pl.warnings);
    let th = pl.ops.iter().find(|o| o.kind == LatheOpKind::Threading).unwrap();
    assert_eq!((th.tool, th.spindle), (4, super::plan::Spindle::Rpm(400.0)));
    let depth = 0.61343 * 1.75;
    let passes: Vec<(f64, f64, f64)> = th
        .moves
        .iter()
        .filter_map(|m| match m {
            TipMove::Thread { to, pitch } => Some((to[0], to[1], *pitch)),
            _ => None,
        })
        .collect();
    assert_eq!(passes.len(), 20, "{passes:?}");
    for (k, (x, z, p)) in passes.iter().enumerate() {
        let d = depth * ((k + 1).min(19) as f64 / 19.0).sqrt();
        assert!((x - (6.0 - d)).abs() < 1e-9 && (z + 17.875).abs() < 0.01 && *p == 1.75, "pass {k}: {x} {z} {p}");
    }
    assert!((depth * (1.0f64 / 19.0).sqrt() - 0.2463).abs() < 1e-4, "the first pass under 0.25");
    let t = pl.threads.first().unwrap();
    assert!((t.z[1] - 3.5).abs() < 1e-9 && (2.0 * t.minor - 9.853).abs() < 1e-3, "{t:?}");
    let (r, _) = simulate(&j, &pl).unwrap();
    assert!(r.gouges.is_empty(), "{:?}", r.gouges);
    assert!(r.rapid_hits.is_empty() && r.jaw_hits == 0, "{:?} {}", r.rapid_hits, r.jaw_hits);
    let prog = super::post::linuxcnc(&j, &pl, "thread").unwrap();
    assert!(prog.contains("G97 S400 M3"));
    assert_eq!(prog.lines().filter(|l| *l == "G33 Z-17.875 K1.75").count(), 20);
    assert!(prog.lines().any(|l| l == "X9.853" || l == "G0 X9.853"), "the last pass at the root");
}

/// The six lathe test parts (tests/cam/lathe) from their bars: each planned and simulated with
/// nothing cut into it, no rapid through material, nothing at the jaws; every piece of material
/// left meets a span a warning names (the V-groove's flanks, the grip groove's, the internal
/// ring groove, the bores' chamfers at the chuck end, the bore past the boring bar's reach). The
/// hollow parts are drilled and the threaded ones threaded.
#[test]
fn the_lathe_test_parts_turn_clean() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/cam/lathe");
    let parts: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(dir.join("parts.json")).unwrap()).unwrap();
    for p in parts["parts"].as_array().unwrap() {
        let name = p["design"].as_str().unwrap();
        let read_step = solidcraft_kernel::step_import(&std::fs::read_to_string(dir.join(name.replace(".sc", ".step"))).unwrap()).unwrap();
        let body = &read_step.bodies[0].body;
        let material = p["material"].as_str().unwrap();
        let threads: Vec<serde_json::Value> = p["threads"]
            .as_array()
            .unwrap()
            .iter()
            .map(|t| serde_json::json!({"designation": t["designation"], "z_from": t["z_from"], "z_to": t["z_to"]}))
            .collect();
        let j = LatheJob::from_json(&format!(
            r#"{{"stock": {{"diameter": {}, "material": "{material}"}}, "part": {{"body": "P", "threads": {}}}}}"#,
            p["stock_bar_diameter"],
            serde_json::Value::from(threads.clone())
        ))
        .unwrap();
        let pl = plan(&j, read(body, super::job::Front::PlusZ).unwrap()).unwrap();
        let (r, _) = simulate(&j, &pl).unwrap();
        assert!(r.gouges.is_empty(), "{name}: {:?}", r.gouges);
        assert!(r.rapid_hits.is_empty() && r.jaw_hits == 0, "{name}: {:?} {}", r.rapid_hits, r.jaw_hits);
        // a part whose profile never meets the axis is hollow: it is drilled; a thread is cut
        let hollow = !p["profile"].as_array().unwrap().iter().any(|seg| seg.as_array().unwrap()[1..].iter().any(|q| q[0].as_f64().unwrap() == 0.0));
        let has = |k: LatheOpKind| pl.ops.iter().any(|o| o.kind == k);
        assert_eq!(has(LatheOpKind::Drilling), hollow, "{name}");
        assert_eq!(has(LatheOpKind::Threading), !threads.is_empty(), "{name}");
        assert_eq!(pl.threads.len(), threads.len(), "{name}");
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

/// The volume a binary STL's triangles enclose (positive when they face out).
fn stl_volume(b: &[u8]) -> f64 {
    let n = u32::from_le_bytes(b[80..84].try_into().unwrap()) as usize;
    let f = |o: usize| f64::from(f32::from_le_bytes(b[o..o + 4].try_into().unwrap()));
    (0..n)
        .map(|i| {
            let o = 84 + 50 * i + 12;
            let v = |k: usize| [f(o + 12 * k), f(o + 12 * k + 4), f(o + 12 * k + 8)];
            let (a, b, c) = (v(0), v(1), v(2));
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
        })
        .sum()
}

/// The tube above as a scene: a zip with `scene.json`, the bar, the part, the turned bar and the
/// program. The bar: Ø22 × 56 (30 + 0.5 + 3 + 2 + 20 = 55.5, rounded up) from Z0.5 back to
/// −55.5, the jaws at 0.5 − 36 = −35.5; revolved as a 120-gon its volume is 60 sin 3° · 11² · 56.
/// The part from Z0 to −30, π (10² − 6²) 30 = 6031.9 (its mesh within 0.02 of the surfaces).
/// The turned bar, by Pappus over the simulation's half section, 2π ∬ r dA times the 120-gon's
/// (60 / π) sin 3°. The tools in the order the program loads them, each with its kind and data.
#[test]
fn the_scene_holds_the_bar_the_part_the_tools_and_the_program() {
    let b = cut(cyl(10.0, 0.0, 30.0), cyl(6.0, -1.0, 31.0));
    let j = job(22.0, "");
    let pl = plan(&j, read(&b, super::job::Front::PlusZ).unwrap()).unwrap();
    let (report, left) = simulate(&j, &pl).unwrap();
    let scene = super::scene::export(&j, &b, &pl, &report, &left).unwrap();
    let bytes = scene.zip().unwrap();
    let mut z = zip::ZipArchive::new(std::io::Cursor::new(bytes)).unwrap();
    let mut names: Vec<String> = z.file_names().map(|n| n.unwrap().to_string()).collect();
    names.sort();
    assert_eq!(names, ["job.ngc", "part.stl", "scene.json", "stock.stl", "turned.stl"]);
    let mut read_file = |name: &str| {
        let mut out = Vec::new();
        std::io::Read::read_to_end(&mut z.by_name(name).unwrap(), &mut out).unwrap();
        out
    };
    let doc: serde_json::Value = serde_json::from_slice(&read_file("scene.json")).unwrap();
    assert_eq!(doc["machine_type"], "lathe");
    let st = &doc["stock"];
    assert_eq!((st["diameter"].as_f64(), st["length"].as_f64()), (Some(22.0), Some(56.0)));
    assert_eq!((st["front_z"].as_f64(), st["back_z"].as_f64(), st["jaws_z"].as_f64()), (Some(0.5), Some(-55.5), Some(-35.5)));
    let tools: Vec<(u64, &str)> =
        doc["tools"].as_array().unwrap().iter().map(|t| (t["number"].as_u64().unwrap(), t["kind"].as_str().unwrap())).collect();
    assert_eq!(tools, [(1, "turning"), (5, "drill"), (6, "boring"), (2, "turning"), (3, "grooving")]);
    let t5 = &doc["tools"][1];
    assert_eq!(
        (t5["diameter"].as_f64(), t5["tip_angle_deg"].as_f64(), t5["peck"].as_f64(), t5["in_turret"].as_bool()),
        (Some(8.0), Some(118.0), Some(4.0), Some(true))
    );
    assert!((t5["z_min"].as_f64().unwrap() + 35.903).abs() < 1e-3);
    let t3 = &doc["tools"][4];
    assert_eq!((t3["width"].as_f64(), t3["reach"].as_f64()), (Some(1.0), Some(12.0)));
    let ops: Vec<&str> = doc["operations"].as_array().unwrap().iter().map(|o| o["kind"].as_str().unwrap()).collect();
    assert_eq!(ops, ["facing", "drilling", "roughing", "boring", "finishing", "parting"]);
    assert_eq!((doc["operations"][1]["spindle"].as_str(), doc["operations"][1]["speed"].as_f64()), (Some("rpm"), Some(1989.0)));
    let bore = doc["part"]["bore"].as_array().unwrap();
    assert!(bore.iter().all(|q| (q[1].as_f64().unwrap() - 12.0).abs() < 0.003), "{bore:?}");
    let prog = String::from_utf8(read_file("job.ngc")).unwrap();
    assert_eq!(prog, super::post::linuxcnc(&j, &pl, "P").unwrap());

    let poly = 60.0 * 3f64.to_radians().sin();
    let bar = stl_volume(&read_file("stock.stl"));
    assert!((bar / (poly * 121.0 * 56.0) - 1.0).abs() < 1e-4, "bar {bar}");
    let part = stl_volume(&read_file("part.stl"));
    assert!((part / (std::f64::consts::PI * 64.0 * 30.0) - 1.0).abs() < 0.005, "part {part}");
    let turned = stl_volume(&read_file("turned.stl"));
    let moment: f64 = left
        .simplified(super::plan::PATH_TOL)
        .unwrap()
        .rings()
        .map(|r| {
            let n = r.len();
            (0..n)
                .map(|i| {
                    let (p, q) = (r[i], r[(i + 1) % n]);
                    (p[0] * q[1] - q[0] * p[1]) * (p[1] + q[1]) / 6.0
                })
                .sum::<f64>()
        })
        .sum::<f64>()
        .abs();
    let want = 2.0 * std::f64::consts::PI * moment * poly / std::f64::consts::PI;
    assert!((turned / want - 1.0).abs() < 1e-3, "turned {turned}, by Pappus {want}");
    // the part parted off: the turned bar is the stub in the chuck and the tube
    assert!(turned > part && turned < bar);
}
