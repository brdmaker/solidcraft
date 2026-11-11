use std::path::{Path, PathBuf};

use serde_json::Value;

use super::job::CtxJob;

fn dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/cam/ctx")
}

fn parts() -> Vec<Value> {
    let v: Value = serde_json::from_str(&std::fs::read_to_string(dir().join("parts.json")).unwrap()).unwrap();
    v["parts"].as_array().unwrap().clone()
}

fn body(p: &Value) -> solidcraft_kernel::Body {
    let step = dir().join(p["design"].as_str().unwrap().replace(".sc", ".step"));
    let read = solidcraft_kernel::step_import(&std::fs::read_to_string(step).unwrap()).unwrap();
    read.bodies[0].body.clone()
}

/// The job for test part `p` on the twin's simulation.
fn job(p: &Value) -> CtxJob {
    CtxJob::from_json(&format!(
        r#"{{"machine": {{"name": "CTX twin (sim)", "max_rpm": 5000, "live_max_rpm": 6000, "sync_max_rpm": 600, "x_min": -5}},
            "stock": {{"diameter": {}, "material": "{}"}},
            "part": {{"body": "{}"}},
            "post": {{"work_z_1": -100, "work_z_2": -450}}}}"#,
        p["stock_bar_diameter"],
        p["material"].as_str().unwrap(),
        p["body"].as_str().unwrap()
    ))
    .unwrap()
}

#[test]
#[ignore]
fn print_ctx() {
    for p in parts() {
        let j = job(&p);
        let plan = super::plan::plan(&j, &body(&p)).unwrap();
        let (r, _, _) = super::sim::simulate(&j, &plan, &body(&p)).unwrap();
        println!("== {} grip {}", p["design"], plan.grip);
        for w in &plan.warnings {
            println!("W {w}");
        }
        println!("{}", serde_json::to_string(&r).unwrap());
        for (side, live) in [(1, &plan.live1), (2, &plan.live2)] {
            for o in live {
                println!("live side {side}: {} T{} {} holes {:?} moves {}", o.name, o.tool, o.rpm, o.holes, o.moves.len());
            }
        }
        if let Ok(out) = std::env::var("CTX_OUT") {
            let stem = p["design"].as_str().unwrap().trim_end_matches(".sc").replace('-', "_");
            std::fs::write(format!("{out}/cam_{stem}_1.ngc"), super::post::side1(&j, &plan, &stem).unwrap()).unwrap();
            std::fs::write(format!("{out}/cam_{stem}_2.ngc"), super::post::side2(&j, &plan, &stem).unwrap()).unwrap();
        }
        if std::env::var("CTX_PRINT").is_ok() {
            println!("{}", super::post::side1(&j, &plan, "t").unwrap());
            println!("{}", super::post::side2(&j, &plan, "t").unwrap());
        }
    }
}

/// Each test part on the twin: both sides' turning simulates clean (nothing cut from the part,
/// nothing left of its turned shape, no rapid through material, no tool or holder at the jaws),
/// chuck 2 grips part of it, side 2 starts from what side 1 cut off, and the live tools do each
/// feature `parts.json` lists on its side without cutting into the part. The only warnings are
/// the blind holes' drill points.
#[test]
fn the_test_parts_are_machined_clean_on_both_sides() {
    use super::live::LiveKind;
    for p in parts() {
        let name = p["design"].as_str().unwrap();
        let j = job(&p);
        let b = body(&p);
        let plan = super::plan::plan(&j, &b).unwrap();
        let unexpected: Vec<&String> = plan.warnings.iter().filter(|w| !w.contains("point at its floor")).collect();
        assert!(unexpected.is_empty(), "{name}: {unexpected:?}");
        let l = p["length"].as_f64().unwrap();
        assert!(plan.grip >= 5.0 && plan.grip <= 0.8 * l + 1e-9, "{name}: grip {}", plan.grip);
        let (r, _, left2) = super::sim::simulate(&j, &plan, &b).unwrap();
        for (side, s) in [(1, &r.side1), (2, &r.side2)] {
            assert!(s.gouges.is_empty() && s.left.is_empty(), "{name} side {side}: gouges {:?} left {:?}", s.gouges, s.left);
            assert!(s.rapid_hits.is_empty() && s.jaw_hits == 0 && s.holder_hits.is_empty(), "{name} side {side}: {s:?}");
        }
        assert!(r.live_gouges.is_empty(), "{name}: {:?}", r.live_gouges);
        // side 2 faced the back allowance off: the part is its length
        let (lo, hi) = left2.bounds().unwrap();
        assert!((hi[0] - 0.0).abs() < 0.01 && (lo[0] + l).abs() < 0.01, "{name}: side 2 leaves z {} to {}", lo[0], hi[0]);
        // each feature by the live tools, on its side
        for f in p["features"].as_array().unwrap() {
            let kind = f["kind"].as_str().unwrap();
            let side = if f["side"] == "back" { 2 } else { 1 };
            let ops = if side == 1 { &plan.live1 } else { &plan.live2 };
            let count = |k: LiveKind| ops.iter().filter(|o| o.kind == k).map(|o| o.holes.len().max(1)).sum::<usize>();
            let ok = match kind {
                "cross_hole" => count(LiveKind::RadialDrill) == if f["through"] == true { 2 } else { 1 },
                "keyway" => count(LiveKind::Keyway) == 1,
                "axial_holes" => count(LiveKind::AxialDrill) == f["count"].as_u64().unwrap() as usize,
                "hex" | "slot" | "flats" | "pocket" => count(LiveKind::FaceMill) == 1,
                other => panic!("{name}: unknown feature {other}"),
            };
            assert!(ok, "{name}: {kind} on side {side}: {:?}", ops.iter().map(|o| (o.kind, o.holes.len())).collect::<Vec<_>>());
        }
    }
}

/// The programs follow the twin's programming manual: the modes, o<twin_init>, chuck 1 clamped,
/// T M6 G43 on turret stations, G96 with D; side 1 brings the parting blade before the transfer,
/// meets side 2 at sync point 3, grips, parts off at a fixed speed under the transfer's limit,
/// then o<twin_cut_done>; side 2 waits at sync point 3, catches, then machines the back. Every
/// coordinate has a decimal point.
#[test]
fn the_programs_follow_the_twins_manual() {
    let p = &parts()[0];
    let j = job(p);
    let plan = super::plan::plan(&j, &body(p)).unwrap();
    let one = super::post::side1(&j, &plan, "pin").unwrap();
    let two = super::post::side2(&j, &plan, "pin").unwrap();
    let lines = |g: &str| g.lines().map(str::to_string).collect::<Vec<_>>();
    let (a, b) = (lines(&one), lines(&two));
    let at = |v: &[String], s: &str| v.iter().position(|l| l.starts_with(s)).unwrap_or_else(|| panic!("no {s} in\n{}", v.join("\n")));
    for v in [&a, &b] {
        assert_eq!(v.first().map(String::as_str), Some("%"));
        assert_eq!(v.last().map(String::as_str), Some("%"));
        assert!(at(v, "G18 G7 G21 G90 G40 G80 G95") < at(v, "o<twin_init> call"));
        assert!(v.iter().any(|l| l == "M30"));
        for l in v.iter().filter(|l| !l.starts_with('(')) {
            for w in l.split_whitespace() {
                let (c, n) = w.split_at(1);
                if "XZRF".contains(c) && n.parse::<f64>().is_ok() {
                    assert!(n.contains('.'), "{w} has no decimal point: {l}");
                }
                if c == "T" {
                    let t: u32 = n.parse().unwrap();
                    assert!((1..=12).contains(&t), "{l}");
                    assert!(l.ends_with("M6 G43"), "{l}");
                }
            }
            if l.starts_with("G96") {
                assert!(l.contains(" D"), "{l}");
            }
        }
    }
    // side 1: chuck, turning, the blade to the cut, sync, grip, parting, cut done
    let (clamp, sync, grip, done) = (at(&a, "M10"), at(&a, "o<twin_sync> call [3]"), at(&a, "o<twin_grip> call"), at(&a, "o<twin_cut_done> call"));
    assert!(clamp < at(&a, "(facing)") && at(&a, "(roughing)") < sync && sync < grip && grip < done, "{one}");
    let blade = a[..sync].iter().rposition(|l| l.starts_with('T')).unwrap();
    assert!(a[blade].starts_with("T9 "), "the parting blade before the transfer: {}", a[blade]);
    assert!(!a[sync..].iter().any(|l| l.contains(" M6")), "no tool change near chuck 2");
    let speed = &a[grip + 1];
    assert!(speed.starts_with("G97 S"), "{speed}");
    let rpm: f64 = speed["G97 S".len()..].split_whitespace().next().unwrap().parse().unwrap();
    assert!(rpm > 0.0 && rpm <= j.machine.sync_max_rpm, "{speed}");
    assert!(a[grip..done].iter().any(|l| l.starts_with("G1 X") || l.starts_with('X')), "parting moves after the grip");
    // side 2: sync, catch, then its own tools
    let (sync2, catch) = (at(&b, "o<twin_sync> call [3]"), at(&b, "o<twin_catch> call"));
    assert!(sync2 < catch && catch < at(&b, "T1 M6"), "{two}");
    assert!(!b.iter().any(|l| l.starts_with("M10")), "chuck 2 is clamped by the transfer");
}

/// A job reads the lathe's program settings beside the twin's own, refuses a field it doesn't
/// know, and fills its tools from the simulation's library: turning tools and live tools on
/// different stations.
#[test]
fn a_job_reads_its_settings_and_the_library() {
    let j = CtxJob::from_json(r#"{"stock": {"diameter": 30}, "part": {"body": "P"}, "post": {"decimals": 3, "work_z_2": -400}}"#).unwrap();
    assert_eq!(j.post.lathe.decimals, 3);
    assert_eq!(j.post.work_z_2, Some(-400.0));
    assert!(CtxJob::from_json(r#"{"stock": {"diameter": 30}, "part": {"body": "P"}, "post": {"decimal": 3}}"#).is_err());
    assert!(CtxJob::from_json(r#"{"stock": {"diameter": 30}, "part": {"body": "P"}, "grip": 3}"#).is_err());
    assert!(!j.tools.is_empty() && !j.live_tools.is_empty());
    for t in &j.live_tools {
        assert!(j.tool(t.number).is_none(), "T{} twice", t.number);
    }
    // a job's own tool takes its station from the library's
    let own = CtxJob::from_json(
        r#"{"stock": {"diameter": 30}, "part": {"body": "P"},
            "live_tools": [{"number": 1, "kind": "drill", "mount": "axial", "diameter": 3, "flute_length": 20, "rpm": 3000, "feed": 100, "plunge": 100}]}"#,
    );
    // T1 was the library's roughing tool: the job now has none at T1, but another turning tool
    let own = own.unwrap();
    assert!(own.tool(1).is_none() && own.live(1).is_some());
}

/// Every feed move in every test part's programs has a feed set since the feed mode last changed
/// (G94 per minute for live tools and polar interpolation, G95 per revolution for turning): a
/// polar mill's first move once ran on the turning feed (0.06 per minute: the twin's sim crawled)
/// or none ("Cannot do g1 with zero feed rate").
#[test]
fn every_feed_move_has_a_feed_for_its_mode() {
    for p in parts() {
        let j = job(&p);
        let plan = super::plan::plan(&j, &body(&p)).unwrap();
        for prog in [super::post::side1(&j, &plan, "t").unwrap(), super::post::side2(&j, &plan, "t").unwrap()] {
            let mut fed = false;
            let mut motion = "";
            for l in prog.lines().filter(|l| !l.starts_with('(') && !l.starts_with('o')) {
                let words: Vec<&str> = l.split_whitespace().collect();
                if words.iter().any(|w| *w == "G94" || *w == "G95" || *w == "G12.1" || *w == "G13.1") {
                    fed = false;
                }
                if words.iter().any(|w| w.starts_with('F')) {
                    fed = true;
                }
                for w in &words {
                    if ["G0", "G1", "G33", "G81", "G83"].contains(w) {
                        motion = w;
                    }
                }
                let moves = words.iter().any(|w| w.starts_with('X') || w.starts_with('Y') || w.starts_with('Z'));
                if moves && ["G1", "G81", "G83"].contains(&motion) {
                    assert!(fed, "{}: no feed for its mode: {l}", p["design"]);
                }
            }
        }
    }
}

/// In polar interpolation no move turns C faster than the machine's speed for polar work (its
/// position loop follows only well under its top speed): C turns at feed / radius at a move's
/// nearest point to the centre, and no cutting move comes within 1 mm of it. Going round the
/// centre at C's top speed once stopped the twin's sim with "joint 2 following error".
#[test]
fn polar_moves_keep_c_under_its_polar_speed() {
    use super::live::{LiveKind, LiveMove, R_MIN};
    for p in parts() {
        let j = job(&p);
        let plan = super::plan::plan(&j, &body(&p)).unwrap();
        let c_max = j.machine.polar_c_rpm * 6.0;
        for op in plan.live1.iter().chain(&plan.live2).filter(|o| o.kind == LiveKind::FaceMill) {
            let mut at: Option<[f64; 3]> = None;
            for m in &op.moves {
                let to = m.to();
                if let (Some(a), LiveMove::Feed(_, f)) = (at, m) {
                    let (dx, dy) = (to[0] - a[0], to[1] - a[1]);
                    let l2 = dx * dx + dy * dy;
                    if l2 > 1e-18 {
                        let t = (-(a[0] * dx + a[1] * dy) / l2).clamp(0.0, 1.0);
                        let r = (a[0] + t * dx).hypot(a[1] + t * dy);
                        let deg_s = (f / 60.0 / r).to_degrees();
                        assert!(deg_s <= c_max * 1.001, "{}: C at {deg_s:.0}°/s from {a:?} to {to:?} at F{f}", p["design"]);
                        if a[2] < 1.0 && to[2] < 1.0 {
                            assert!(r >= R_MIN - 1e-6, "{}: cutting {r:.3} from the centre", p["design"]);
                        }
                    }
                }
                at = Some(to);
            }
        }
    }
}

/// No cutting path in polar interpolation folds back on itself near the centre (where C would
/// reverse at once): the face pattern's back pocket round its bore once ran its ring along Y−2
/// to X0 and straight back, and the twin's sim stopped there with "joint 2 following error". The
/// pocket is now cut with the open bore beside it.
#[test]
fn polar_paths_dont_fold_back_near_the_centre() {
    use super::live::{LiveKind, LiveMove};
    for p in parts() {
        let j = job(&p);
        let plan = super::plan::plan(&j, &body(&p)).unwrap();
        for op in plan.live1.iter().chain(&plan.live2).filter(|o| o.kind == LiveKind::FaceMill) {
            let pts: Vec<[f64; 3]> = op.moves.iter().map(LiveMove::to).collect();
            for w in pts.windows(3) {
                let (a, b, c) = (w[0], w[1], w[2]);
                if a[2] > 0.0 || b[2] > 0.0 || c[2] > 0.0 || (a[2] - b[2]).abs() > 1e-9 || (b[2] - c[2]).abs() > 1e-9 {
                    continue;
                }
                let (u, v) = ([b[0] - a[0], b[1] - a[1]], [c[0] - b[0], c[1] - b[1]]);
                let (lu, lv) = (u[0].hypot(u[1]), v[0].hypot(v[1]));
                if lu < 1e-6 || lv < 1e-6 || b[0].hypot(b[1]) > 4.0 {
                    continue;
                }
                let turn = ((u[0] * v[0] + u[1] * v[1]) / (lu * lv)).clamp(-1.0, 1.0).acos().to_degrees();
                assert!(turn < 150.0, "{}: turns {turn:.0}° at {b:?}", p["design"]);
            }
        }
    }
}
