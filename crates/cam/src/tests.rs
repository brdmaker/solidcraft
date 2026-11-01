//! CAM tests. Every expected value is worked out by hand or in closed form, never read back
//! from the code under test.

mod region_tests {
    use std::f64::consts::PI;

    use crate::region::{Area, GRID, circle_ring, ring_length, segments_for, signed_area};

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    /// Closing by 3 µm fills a 4 µm slit between two 10 × 10 squares (one shape of 20.004 × 10)
    /// and leaves an L of 10 × 10 less a 5 × 5 corner as it was: area 75, six sharp corners.
    #[test]
    fn closing_fills_hairline_gaps_and_keeps_corners() {
        let two = Area::rect(0.0, 0.0, 10.0, 10.0).unwrap().union(&Area::rect(10.004, 0.0, 20.004, 10.0).unwrap()).unwrap();
        assert_eq!(two.shapes.len(), 2);
        let c = two.closed(0.003).unwrap();
        assert_eq!(c.shapes.len(), 1, "one shape");
        assert!(close(c.area(), 200.04, 1e-3), "area {}", c.area());
        let l = Area::from_rings(&[vec![[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [5.0, 5.0], [5.0, 10.0], [0.0, 10.0]]]).unwrap();
        let c = l.closed(0.003).unwrap();
        assert!(close(c.area(), 75.0, 1e-6), "area {}", c.area());
        let ring = c.rings().next().unwrap();
        assert_eq!(ring.len(), 6, "{ring:?}");
        for p in [[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [5.0, 5.0], [5.0, 10.0], [0.0, 10.0]] {
            assert!(ring.iter().any(|q| close(q[0], p[0], GRID) && close(q[1], p[1], GRID)), "corner {p:?} kept: {ring:?}");
        }
    }

    #[test]
    fn rectangle_area_and_bounds() {
        let a = Area::rect(10.0, 0.0, -10.0, 5.0).unwrap();
        assert!(close(a.area(), 100.0, 1e-6));
        assert_eq!(a.bounds(), Some(([-10.0, 0.0], [10.0, 5.0])));
        assert!(a.contains([0.0, 2.5]));
        assert!(!a.contains([0.0, 6.0]));
    }

    /// A ring inside a ring is a hole, whatever direction either runs.
    #[test]
    fn nested_rings_make_a_hole() {
        let outer = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let inner = vec![[2.0, 2.0], [8.0, 2.0], [8.0, 8.0], [2.0, 8.0]]; // same direction as outer
        let a = Area::from_rings(&[outer, inner]).unwrap();
        assert!(close(a.area(), 100.0 - 36.0, 1e-6));
        assert!(!a.contains([5.0, 5.0]));
        assert!(a.contains([1.0, 5.0]));
        assert_eq!(a.shapes.len(), 1);
        assert_eq!(a.shapes[0].len(), 2);
        assert!(signed_area(&a.shapes[0][0]) > 0.0, "outer ring counter-clockwise");
        assert!(signed_area(&a.shapes[0][1]) < 0.0, "hole clockwise");
    }

    #[test]
    fn booleans_of_two_squares() {
        let a = Area::rect(0.0, 0.0, 10.0, 10.0).unwrap();
        let b = Area::rect(5.0, 5.0, 15.0, 15.0).unwrap();
        assert!(close(a.union(&b).unwrap().area(), 175.0, 1e-6));
        assert!(close(a.difference(&b).unwrap().area(), 75.0, 1e-6));
        assert!(close(a.intersection(&b).unwrap().area(), 25.0, 1e-6));
        let apart = Area::rect(20.0, 0.0, 30.0, 10.0).unwrap();
        assert!(a.intersection(&apart).unwrap().is_empty());
        assert_eq!(a.union(&apart).unwrap().shapes.len(), 2);
    }

    /// Growing a w × h rectangle by d gives w·h + 2d(w + h) + πd² (round corners); the chords
    /// of the corners lie inside the true arcs, by at most the tolerance.
    #[test]
    fn outward_offset_of_a_rectangle_has_round_corners() {
        let (w, h, d, tol) = (40.0, 20.0, 3.0, 0.001);
        let a = Area::rect(0.0, 0.0, w, h).unwrap().offset(d, tol).unwrap();
        let exact = w * h + 2.0 * d * (w + h) + PI * d * d;
        assert!(a.area() <= exact + 1e-6);
        assert!(exact - a.area() < 2.0 * PI * d * tol + 1e-3, "area {} vs {exact}", a.area());
        let (lo, hi) = a.bounds().unwrap();
        assert!(close(lo[0], -d, 2.0 * GRID) && close(hi[0], w + d, 2.0 * GRID));
        assert!(close(lo[1], -d, 2.0 * GRID) && close(hi[1], h + d, 2.0 * GRID));
    }

    /// Shrinking keeps square corners: (w − 2d)(h − 2d).
    #[test]
    fn inward_offset_of_a_rectangle() {
        let a = Area::rect(0.0, 0.0, 40.0, 20.0).unwrap().offset(-3.0, 0.001).unwrap();
        assert!(close(a.area(), 34.0 * 14.0, 1e-4), "area {}", a.area());
        let gone = Area::rect(0.0, 0.0, 40.0, 20.0).unwrap().offset(-10.5, 0.001).unwrap();
        assert!(gone.is_empty(), "a 20 mm wide area shrunk by 10.5 mm is empty");
    }

    /// Offsetting a circle changes its radius: a tool of radius 3 in a Ø20 hole runs on r = 7.
    #[test]
    fn circle_offset_changes_the_radius() {
        let hole = Area::circle(5.0, -2.0, 10.0, 0.0005).unwrap();
        let path = hole.offset(-3.0, 0.0005).unwrap();
        let r = (path.area() / PI).sqrt();
        assert!(close(r, 7.0, 0.002), "radius {r}");
        let ring = &path.shapes[0][0];
        for p in ring {
            let d = (p[0] - 5.0).hypot(p[1] + 2.0);
            assert!(close(d, 7.0, 0.002), "point {p:?} at {d}");
        }
    }

    #[test]
    fn circle_chords_stay_within_tolerance() {
        let (r, tol) = (12.0, 0.01);
        let n = segments_for(r, tol, 2.0 * PI);
        let half = PI / n as f64;
        assert!(r * (1.0 - half.cos()) <= tol + 1e-12, "sag of {n} chords");
        let ring = circle_ring(0.0, 0.0, r, tol);
        assert_eq!(ring.len(), n);
        assert!(close(ring_length(&ring), 2.0 * n as f64 * r * half.sin(), 1e-9));
    }

    #[test]
    fn placing_turns_then_moves() {
        let a = Area::rect(0.0, 0.0, 10.0, 4.0).unwrap();
        let (lo, hi) = a.placed(1, 100.0, 0.0).bounds().unwrap();
        assert_eq!((lo, hi), ([96.0, 0.0], [100.0, 10.0]));
        assert!(close(a.placed(3, 0.0, 0.0).area(), 40.0, 1e-9), "turning keeps the area");
    }

    #[test]
    fn hostile_input_is_an_error_not_a_panic() {
        assert!(Area::from_rings(&[vec![[f64::NAN, 0.0], [1.0, 0.0], [1.0, 1.0]]]).is_err());
        assert!(Area::from_rings(&[vec![[1e9, 0.0], [1.0, 0.0], [1.0, 1.0]]]).is_err());
        assert!(Area::circle(0.0, 0.0, -1.0, 0.01).is_err());
        assert!(Area::rect(0.0, 0.0, 1.0, 1.0).unwrap().offset(f64::INFINITY, 0.01).is_err());
        assert!(Area::from_rings(&[vec![[0.0, 0.0], [1.0, 1.0]]]).unwrap().is_empty(), "two points enclose nothing");
    }
}

mod part_tests {
    use std::f64::consts::PI;

    use solidcraft_geom::Vec3;
    use solidcraft_kernel::{Body, BoolOp, boolean, box_solid, cylinder};

    use crate::part::read_part;

    fn v(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z)
    }

    fn cut(a: Body, b: Body) -> Body {
        boolean(&a, &b, BoolOp::Cut).unwrap().unwrap()
    }

    /// A 40 × 30 × 10 plate with a 20 × 10 pocket 4 deep cut into its bottom, as if modelled
    /// upside down: as modelled the pocket's floor faces down (unreachable from the top); turned
    /// over (−Z up) it is an ordinary pocket, 4 deep from the top, nothing unreachable, resting on
    /// the 40 × 30 face. `auto` picks that; `+z` keeps it as modelled.
    #[test]
    fn auto_orientation_turns_an_upside_down_part_over() {
        use crate::job::{Orient, Up};
        use crate::part::oriented;
        let b = cut(box_solid(v(-20.0, -15.0, 0.0), v(20.0, 15.0, 10.0)).unwrap(), box_solid(v(-10.0, -5.0, -1.0), v(10.0, 5.0, 4.0)).unwrap());
        let as_modelled = read_part(&b).unwrap();
        assert!(!as_modelled.unsupported.is_empty(), "the pocket's floor faces down");
        let (up, _, f) = oriented(&b, Orient::Auto, 10.0).unwrap();
        assert_eq!(up, Up::MinusZ);
        assert!(f.unsupported.is_empty(), "{:?}", f.unsupported);
        assert!((f.top_z - f.bottom_z - 10.0).abs() < 1e-6);
        let floor = f.floors.iter().find(|fl| !fl.top).unwrap();
        assert!((f.top_z - floor.z - 4.0).abs() < 1e-6, "the pocket 4 deep from the top");
        assert!((f.bottom_area - 1200.0).abs() < 1e-6, "resting on 40 × 30: {}", f.bottom_area);
        let (up, _, f) = oriented(&b, Orient::PlusZ, 10.0).unwrap();
        assert_eq!(up, Up::PlusZ);
        assert!(!f.unsupported.is_empty());
    }

    /// A plain 10 × 20 × 30 block: every way up is reachable, so `auto` rests it on its largest
    /// face, 20 × 30 (X up, 10 tall; +X before −X); given an axis it turns as told: +Y up is 20
    /// tall, −Z up 30, each resting on the face across that axis.
    #[test]
    fn auto_orientation_rests_a_part_on_its_largest_face() {
        use crate::job::{Orient, Up};
        use crate::part::oriented;
        let b = box_solid(v(0.0, 0.0, 0.0), v(10.0, 20.0, 30.0)).unwrap();
        let height = |f: &crate::part::PartFeatures| f.top_z - f.bottom_z;
        let (up, _, f) = oriented(&b, Orient::Auto, 40.0).unwrap();
        assert_eq!(up, Up::PlusX);
        assert!((height(&f) - 10.0).abs() < 1e-6 && (f.bottom_area - 600.0).abs() < 1e-6, "{} {}", height(&f), f.bottom_area);
        for (o, h, area) in [(Orient::PlusY, 20.0, 300.0), (Orient::MinusZ, 30.0, 200.0), (Orient::MinusX, 10.0, 600.0)] {
            let (_, _, f) = oriented(&b, o, 40.0).unwrap();
            assert!((height(&f) - h).abs() < 1e-6 && (f.bottom_area - area).abs() < 1e-6, "{o:?}: {} tall on {}", height(&f), f.bottom_area);
        }
        // the job's `orient` field
        let j = crate::job::Job::from_json(
            r#"{"stock": {"size": [300, 300], "thickness": 10}, "parts": [{"body": "A"}, {"body": "B", "orient": "-y"}]}"#,
        )
        .unwrap();
        assert_eq!((j.parts[0].orient, j.parts[1].orient), (Orient::Auto, Orient::MinusY));
        assert!(crate::job::Job::from_json(r#"{"stock": {"size": [300, 300], "thickness": 10}, "parts": [{"body": "A", "orient": "up"}]}"#).is_err());
        // planned as a job: the axis it chose is reported
        let plan = crate::plan::plan_job(
            &crate::job::Job::from_json(r#"{"stock": {"size": [300, 300], "thickness": 12}, "parts": [{"body": "A"}]}"#).unwrap(),
            &[b],
        )
        .unwrap();
        assert_eq!(plan.ups, vec![Up::PlusX]);
    }

    /// A 40 × 30 × 10 plate: a Ø6 through hole at (10, 10); a counterbore (Ø10 × 4 deep over a
    /// Ø5 through hole) at (−10, 5); a 10 × 8 pocket 3 deep centred on (0, −8).
    fn plate() -> Body {
        let mut b = box_solid(v(-20.0, -15.0, 0.0), v(20.0, 15.0, 10.0)).unwrap();
        b = cut(b, cylinder(v(10.0, 10.0, -1.0), v(0.0, 0.0, 1.0), 3.0, 12.0).unwrap());
        b = cut(b, cylinder(v(-10.0, 5.0, 6.0), v(0.0, 0.0, 1.0), 5.0, 5.0).unwrap());
        b = cut(b, cylinder(v(-10.0, 5.0, -1.0), v(0.0, 0.0, 1.0), 2.5, 12.0).unwrap());
        cut(b, box_solid(v(-5.0, -12.0, 7.0), v(5.0, -4.0, 11.0)).unwrap())
    }

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    /// Most a circle of radius `r` drawn as chords within AREA_TOL loses in area: 2/3 · tol · 2πr.
    fn chord_loss(r: f64) -> f64 {
        2.0 / 3.0 * crate::part::AREA_TOL * 2.0 * PI * r
    }

    #[test]
    fn plate_heights_outline_and_silhouette() {
        let p = read_part(&plate()).unwrap();
        assert!(close(p.top_z, 10.0, 1e-6) && close(p.bottom_z, 0.0, 1e-6));
        assert!(close(p.outline_area, 1200.0, 1e-3), "outline {}", p.outline_area);
        // Seen from above, only the two through holes are open.
        let open = PI * 9.0 + PI * 6.25;
        assert!(close(p.silhouette_area, 1200.0 - open, 0.05), "silhouette {} vs {}", p.silhouette_area, 1200.0 - open);
        let b = p.bounds;
        assert!(close(b[0][0], -20.0, 1e-9) && close(b[0][1], -15.0, 1e-9) && close(b[1][0], 20.0, 1e-9) && close(b[1][1], 15.0, 1e-9), "{b:?}");
        assert!(p.unsupported.is_empty(), "{:?}", p.unsupported);
        assert!(p.chamfers.is_empty() && p.cutouts.is_empty());
    }

    #[test]
    fn plate_holes() {
        let p = read_part(&plate()).unwrap();
        assert_eq!(p.holes.len(), 2, "{:#?}", p.holes);
        let cb = p.holes.iter().find(|h| close(h.center[0], -10.0, 1e-6) && close(h.center[1], 5.0, 1e-6)).unwrap();
        assert_eq!(cb.kind, "counterbore");
        assert!(cb.through && close(cb.diameter, 5.0, 1e-6));
        assert_eq!(cb.steps.len(), 2);
        assert!(close(cb.steps[0].r_top, 5.0, 1e-6) && close(cb.steps[0].z_bottom, 6.0, 1e-6) && close(cb.steps[0].z_top, 10.0, 1e-6));
        assert!(close(cb.steps[1].r_top, 2.5, 1e-6) && close(cb.steps[1].z_top, 6.0, 1e-6) && close(cb.steps[1].z_bottom, 0.0, 1e-6));
        let h = p.holes.iter().find(|h| close(h.center[0], 10.0, 1e-6)).unwrap();
        assert_eq!(h.kind, "hole");
        assert!(h.through && close(h.diameter, 6.0, 1e-6) && close(h.z_top, 10.0, 1e-6));
    }

    #[test]
    fn plate_floors() {
        let p = read_part(&plate()).unwrap();
        let mut z: Vec<f64> = p.floors.iter().map(|f| (f.z * 1e6).round() / 1e6).collect();
        z.sort_by(f64::total_cmp);
        assert_eq!(z, vec![6.0, 7.0, 10.0]);
        let top = p.floors.iter().find(|f| f.top).unwrap();
        // The holes in the top face are drawn as chords inside their circles (AREA_TOL), which
        // understates their area by at most 2/3 · tol · circumference: 2/3 · 0.002 · 2π(3 + 5).
        let top_area = 1200.0 - PI * 9.0 - PI * 25.0 - 80.0;
        let chord = 2.0 / 3.0 * crate::part::AREA_TOL * 2.0 * PI * 8.0;
        assert!(top.area >= top_area - 1e-6 && top.area <= top_area + chord + 1e-6, "top {} vs {top_area}", top.area);
        let seat = p.floors.iter().find(|f| close(f.z, 6.0, 1e-6)).unwrap();
        assert!(seat.in_hole && close(seat.area, PI * (25.0 - 6.25), chord_loss(5.0) + 1e-6), "seat {}", seat.area);
        let pocket = p.floors.iter().find(|f| close(f.z, 7.0, 1e-6)).unwrap();
        assert!(!pocket.in_hole && close(pocket.area, 80.0, 1e-3));
        let b = pocket.bounds;
        assert!(close(b[0][0], -5.0, 1e-9) && close(b[0][1], -12.0, 1e-9) && close(b[1][0], 5.0, 1e-9) && close(b[1][1], -4.0, 1e-9), "{b:?}");
    }

    /// A rectangular through cutout is a cutout, not a hole; a blind round recess is a hole stack
    /// that does not go through.
    #[test]
    fn cutouts_and_blind_holes() {
        let mut b = box_solid(v(0.0, 0.0, 0.0), v(50.0, 40.0, 5.0)).unwrap();
        b = cut(b, box_solid(v(10.0, 10.0, -1.0), v(30.0, 20.0, 6.0)).unwrap());
        b = cut(b, cylinder(v(40.0, 30.0, 2.0), v(0.0, 0.0, 1.0), 4.0, 4.0).unwrap());
        let p = read_part(&b).unwrap();
        assert_eq!(p.cutouts.len(), 1);
        assert!(close(p.cutouts[0].area, 200.0, 1e-3));
        let b = p.cutouts[0].bounds;
        assert!(close(b[0][0], 10.0, 1e-9) && close(b[0][1], 10.0, 1e-9) && close(b[1][0], 30.0, 1e-9) && close(b[1][1], 20.0, 1e-9), "{b:?}");
        assert_eq!(p.holes.len(), 1);
        let h = &p.holes[0];
        assert!(!h.through && close(h.z_bottom, 2.0, 1e-6) && close(h.diameter, 8.0, 1e-6));
        assert!(p.floors.iter().any(|f| close(f.z, 2.0, 1e-6) && f.in_hole && close(f.area, PI * 16.0, chord_loss(4.0) + 1e-6)));
    }

    /// A ball (or any face a 3-axis job from the top cannot finish with flat tools) is reported,
    /// not skipped silently.
    #[test]
    fn unsupported_faces_are_reported() {
        let b = solidcraft_kernel::sphere(v(0.0, 0.0, 0.0), 5.0).unwrap();
        let p = read_part(&b).unwrap();
        assert!(!p.unsupported.is_empty());
        assert!(p.unsupported.iter().all(|n| !n.reason.is_empty()));
    }
}

mod job_tests {
    use crate::job::{CutOutMethod, Job, ToolKind, ToolLibrary};

    const MINIMAL: &str = r#"{"stock": {"size": [600, 400], "thickness": 10}, "parts": [{"body": "Flange"}]}"#;

    #[test]
    fn a_minimal_job_takes_the_defaults() {
        let j = Job::from_json(MINIMAL).unwrap();
        assert_eq!(j.version, 1);
        assert_eq!(j.stock.spoilboard_allowance, 0.2);
        assert_eq!((j.nest.margin, j.nest.spacing, j.nest.rotate), (10.0, 8.0, true));
        assert_eq!(j.parts[0].quantity, 1);
        assert_eq!(j.parts[0].label(), "Flange");
        let st = &j.strategy;
        assert_eq!((st.stock_to_leave, st.floor_stock_to_leave, st.wall_passes.clone()), (0.5, 0.2, vec![0.3, 0.1]));
        let c = &j.strategy.cut_out;
        assert_eq!((c.method, c.skin, c.skin_stepdown), (CutOutMethod::Skin, 0.5, 0.1));
        // no tools listed and no library named: the built-in library's
        assert_eq!(j.tools.iter().map(|t| t.number).collect::<Vec<_>>(), vec![1, 2, 3, 6, 9, 10]);
    }

    /// The built-in library: single-flute end mills Ø1, Ø2, Ø3 and Ø6 as T1, T2, T3 and T6, and
    /// 90° chamfer mills Ø6 and Ø12 as T9 and T10. With it, roughing takes the Ø6, finishing
    /// the Ø3 (the largest smaller than the roughing tool), cutting out the Ø6 (its 22 mm flutes
    /// reach through 10 mm and the 0.2 below), chamfers the first chamfer mill.
    #[test]
    fn the_builtin_library_holds_the_usual_tools() {
        let lib = ToolLibrary::builtin().unwrap();
        let got: Vec<(u32, ToolKind, f64, u32)> = lib.tools.iter().map(|t| (t.number, t.kind, t.diameter, t.flutes)).collect();
        assert_eq!(
            got,
            vec![
                (1, ToolKind::Flat, 1.0, 1),
                (2, ToolKind::Flat, 2.0, 1),
                (3, ToolKind::Flat, 3.0, 1),
                (6, ToolKind::Flat, 6.0, 1),
                (9, ToolKind::Chamfer, 6.0, 1),
                (10, ToolKind::Chamfer, 12.0, 1)
            ]
        );
        assert!(lib.tools.iter().filter(|t| t.kind == ToolKind::Chamfer).all(|t| t.tip_angle() == 90.0));
        let j = Job::from_json(MINIMAL).unwrap();
        let n = |t: Option<&crate::job::Tool>| t.map(|t| t.number);
        assert_eq!((n(j.rough_tool()), n(j.finish_tool()), n(j.profile_tool()), n(j.chamfer_tool())), (Some(6), Some(3), Some(6), Some(9)));
    }

    /// Holes no drill fits are milled with the finishing tool when the hole's radius is at least
    /// 1.05 times the tool's, else with the largest end mill it is for: Ø3.2 takes the Ø3
    /// (3.15 ≤ 3.2), Ø2.5 the Ø2 (2.1 ≤ 2.5 < 3.15), Ø1.5 the Ø1 (1.05 ≤ 1.5 < 2.1); Ø1 none.
    #[test]
    fn holes_take_the_largest_end_mill_that_fits() {
        let j = Job::from_json(MINIMAL).unwrap();
        let n = |d: f64| j.hole_tool(d).map(|t| t.number);
        assert_eq!((n(3.2), n(2.5), n(1.5), n(1.0)), (Some(3), Some(2), Some(1), None));
    }

    /// A library file adds its tools to the job's; a job tool of the same number replaces the
    /// library's (T6 here, fed at 1500 not 1000), and the strategy may name a library tool. A
    /// job naming a library read where there is nothing to read it with is refused.
    #[test]
    fn a_library_file_adds_tools_and_the_job_replaces_by_number() {
        let lib = r#"{"version": 1, "name": "shop", "tools": [
            {"number": 4, "kind": "flat", "diameter": 4, "flute_length": 12, "rpm": 20000, "feed": 800, "plunge": 200},
            {"number": 6, "kind": "flat", "diameter": 6, "flute_length": 22, "rpm": 18000, "feed": 1000, "plunge": 250}]}"#;
        let text = r#"{"stock": {"size": [600, 400], "thickness": 10}, "library": "shop.json",
            "tools": [{"number": 6, "kind": "flat", "diameter": 6, "flute_length": 22, "rpm": 18000, "feed": 1500, "plunge": 250}],
            "strategy": {"finish_tool": 4}, "parts": [{"body": "B"}]}"#;
        let asked = std::cell::RefCell::new(Vec::new());
        let load = |path: &str| {
            asked.borrow_mut().push(path.to_string());
            Ok(lib.to_string())
        };
        let j = Job::from_json_with(text, &load).unwrap();
        assert_eq!(asked.borrow().as_slice(), ["shop.json"]);
        assert_eq!(j.tools.iter().map(|t| (t.number, t.feed)).collect::<Vec<_>>(), vec![(4, 800.0), (6, 1500.0)]);
        assert_eq!(j.finish_tool().map(|t| t.number), Some(4));
        let err = Job::from_json(text).unwrap_err().to_string();
        assert!(err.contains("shop.json"), "{err}");
        let bad = Job::from_json_with(text, &|_| {
            Ok(r#"{"version": 1, "tools": [{"number": 0, "kind": "flat", "diameter": 6, "flute_length": 22, "rpm": 1, "feed": 1, "plunge": 1}]}"#
                .into())
        });
        assert!(bad.unwrap_err().to_string().contains("library.tools[0].number"));
    }

    /// Wall passes each leave less than the one before, starting below the roughing's stock to
    /// leave, and more than nothing (the last pass leaves that).
    #[test]
    fn wall_passes_are_checked() {
        let with = |extra: &str| {
            Job::from_json(&format!(r#"{{"stock": {{"size": [600, 400], "thickness": 3}}, "strategy": {{{extra}}}, "parts": [{{"body": "B"}}]}}"#))
        };
        assert!(with(r#""wall_passes": [0.4, 0.2, 0.05]"#).is_ok());
        assert!(with(r#""wall_passes": []"#).is_ok());
        for bad in [
            r#""wall_passes": [0.5]"#,
            r#""wall_passes": [0.1, 0.3]"#,
            r#""wall_passes": [0.3, 0]"#,
            r#""stock_to_leave": 0.2, "wall_passes": [0.3]"#,
        ] {
            assert!(with(bad).unwrap_err().to_string().contains("wall_passes"), "{bad}");
        }
    }

    /// The skin must be thinner than the sheet; its passes at least 0.01 mm.
    #[test]
    fn the_skin_is_checked() {
        let with = |extra: &str| {
            Job::from_json(&format!(
                r#"{{"stock": {{"size": [600, 400], "thickness": 3}}, "strategy": {{"cut_out": {{{extra}}}}}, "parts": [{{"body": "B"}}]}}"#
            ))
        };
        assert!(with(r#""skin": 0.5, "skin_stepdown": 0.1"#).is_ok());
        assert!(with(r#""skin": 3"#).unwrap_err().to_string().contains("skin"));
        assert!(with(r#""skin_stepdown": 0"#).unwrap_err().to_string().contains("skin_stepdown"));
        assert_eq!(with(r#""method": "profile""#).unwrap().strategy.cut_out.method, CutOutMethod::Profile);
    }

    #[test]
    fn tools_take_their_defaults() {
        let j = Job::from_json(
            r#"{"stock": {"size": [600, 400], "thickness": 10}, "parts": [{"body": "B"}],
                "tools": [{"number": 3, "kind": "drill", "diameter": 5, "flute_length": 30, "rpm": 3000, "feed": 200, "plunge": 100},
                          {"number": 7, "kind": "flat", "diameter": 6, "flute_length": 20, "rpm": 18000, "feed": 1200, "plunge": 300, "stepover": 0.1},
                          {"number": 9, "kind": "chamfer", "diameter": 12, "flute_length": 6, "rpm": 12000, "feed": 600, "plunge": 200}]}"#,
        )
        .unwrap();
        let drill = j.tool(3).unwrap();
        assert_eq!((drill.kind, drill.tip_angle(), drill.stepdown(), drill.flutes), (ToolKind::Drill, 118.0, 5.0, 2));
        let flat = j.tool(7).unwrap();
        assert!((flat.stepover() - 0.6).abs() < 1e-12, "10 % of 6 mm");
        assert_eq!(j.tool(9).unwrap().tip_angle(), 90.0);
        assert!(j.tool(4).is_none());
    }

    #[test]
    fn hostile_jobs_are_refused_with_a_reason() {
        let bad = [
            (r#"{"stock": {"size": [-1, 400], "thickness": 10}, "parts": [{"body": "B"}]}"#, "stock.size[0]"),
            (r#"{"stock": {"size": [600, 400], "thickness": 0}, "parts": [{"body": "B"}]}"#, "stock.thickness"),
            (r#"{"stock": {"size": [600, 400], "thickness": 10}, "parts": []}"#, "at least one part"),
            (r#"{"stock": {"size": [600, 400], "thickness": 10}, "parts": [{"body": "B", "quantity": 0}]}"#, "quantity"),
            (r#"{"stock": {"size": [600, 400], "thickness": 10}, "parts": [{"body": "B", "quantity": 20000}]}"#, "at most 10000"),
            (r#"{"stock": {"size": [600, 400], "thickness": 10}, "nest": {"margin": 300}, "parts": [{"body": "B"}]}"#, "no room"),
            (r#"{"stock": {"size": [600, 400], "thickness": 10, "spoilboard_allowance": 9}, "parts": [{"body": "B"}]}"#, "spoilboard"),
            (r#"{"stock": {"size": [600, 400], "thickness": 10}, "parts": [{"body": "B"}], "colour": "red"}"#, "unknown field"),
            (r#"{"version": 2, "stock": {"size": [600, 400], "thickness": 10}, "parts": [{"body": "B"}]}"#, "version 2"),
            (
                r#"{"stock": {"size": [600, 400], "thickness": 10}, "parts": [{"body": "B"}],
                    "tools": [{"number": 1, "kind": "flat", "diameter": 6, "flute_length": 20, "rpm": 1, "feed": 1, "plunge": 1},
                              {"number": 1, "kind": "drill", "diameter": 5, "flute_length": 20, "rpm": 1, "feed": 1, "plunge": 1}]}"#,
                "listed twice",
            ),
            (
                r#"{"stock": {"size": [600, 400], "thickness": 10}, "parts": [{"body": "B"}],
                    "tools": [{"number": 1, "kind": "flat", "diameter": 6, "flute_length": 20, "rpm": 1, "feed": 0, "plunge": 1}]}"#,
                "feed",
            ),
            (
                r#"{"stock": {"size": [600, 400], "thickness": 10}, "parts": [{"body": "B"}],
                    "tools": [{"number": 1, "kind": "chamfer", "diameter": 6, "flute_length": 20, "rpm": 1, "feed": 1, "plunge": 1, "tip_angle_deg": 180}]}"#,
                "tip_angle_deg",
            ),
            ("[1, 2", "job:"),
        ];
        for (text, why) in bad {
            let e = Job::from_json(text).unwrap_err().to_string();
            assert!(e.contains(why), "{text}\n  → {e}\n  (expected it to mention `{why}`)");
        }
    }
}

mod nest_tests {
    use crate::job::{NestOptions, Stock};
    use crate::nest::{NestItem, Nesting, nest};
    use crate::region::Area;

    fn stock(w: f64, h: f64, t: f64) -> Stock {
        Stock { size: [w, h], thickness: t, material: String::new(), spoilboard_allowance: 0.2, origin: crate::job::Origin::TopFrontLeft }
    }

    fn opts(margin: f64, spacing: f64, rotate: bool) -> NestOptions {
        NestOptions { margin, spacing, rotate }
    }

    /// Independent of how the layout was found: every placed outline lies inside the margins,
    /// and outlines grown by half the spacing never meet.
    fn check_layout(n: &Nesting, st: &Stock, o: &NestOptions, outlines: &[&Area]) {
        let placed: Vec<Area> = n.placements.iter().map(|p| outlines[p.part].placed(p.quarter_turns, p.offset[0], p.offset[1])).collect();
        for (p, a) in n.placements.iter().zip(&placed) {
            let (lo, hi) = a.bounds().unwrap();
            assert!(lo[0] >= o.margin - 1e-6 && lo[1] >= o.margin - 1e-6, "{p:?} below the margin: {lo:?}");
            assert!(hi[0] <= st.size[0] - o.margin + 1e-6 && hi[1] <= st.size[1] - o.margin + 1e-6, "{p:?} past the margin: {hi:?}");
            for k in 0..2 {
                assert!((lo[k] - p.bounds[0][k]).abs() < 1e-6 && (hi[k] - p.bounds[1][k]).abs() < 1e-6, "{p:?} reports bounds {:?}", (lo, hi));
            }
        }
        let grown: Vec<Area> = placed.iter().map(|a| a.offset(o.spacing / 2.0 - 1e-3, 0.01).unwrap()).collect();
        for i in 0..grown.len() {
            for j in (i + 1)..grown.len() {
                assert!(
                    grown[i].intersection(&grown[j]).unwrap().is_empty(),
                    "{:?} and {:?} are closer than the spacing",
                    n.placements[i],
                    n.placements[j]
                );
            }
        }
    }

    /// Four 100 mm squares on a 230 mm sheet, margin 10 and spacing 10: 100 + 10 + 100 fills
    /// the 210 mm between the margins exactly, so they go 2 × 2.
    #[test]
    fn four_squares_fill_the_sheet_exactly() {
        let sq = Area::rect(0.0, 0.0, 100.0, 100.0).unwrap();
        let (st, o) = (stock(230.0, 230.0, 10.0), opts(10.0, 10.0, true));
        let n = nest(&st, &o, &[NestItem { label: "sq", quantity: 4, outline: &sq, height: 10.0 }]);
        assert_eq!(n.placements.len(), 4);
        assert!(n.unplaced.is_empty());
        let mut corners: Vec<[f64; 2]> = n.placements.iter().map(|p| p.bounds[0]).collect();
        corners.sort_by(|a, b| a[1].total_cmp(&b[1]).then(a[0].total_cmp(&b[0])));
        assert_eq!(corners, vec![[10.0, 10.0], [120.0, 10.0], [10.0, 120.0], [120.0, 120.0]]);
        assert!((n.utilisation - 40_000.0 / (210.0 * 210.0)).abs() < 1e-9);
        check_layout(&n, &st, &o, &[&sq]);
        // A fifth has no room and says so.
        let n = nest(&st, &o, &[NestItem { label: "sq", quantity: 5, outline: &sq, height: 10.0 }]);
        assert_eq!((n.placements.len(), n.unplaced.len()), (4, 1));
        assert_eq!(n.unplaced[0].reason, "no room left on the sheet");
    }

    /// A 150 × 40 strip on a 60 mm wide sheet fits only turned; three stack 3 × 150 = 450 ≤ 460.
    #[test]
    fn strips_turn_to_fit() {
        let strip = Area::rect(0.0, 0.0, 150.0, 40.0).unwrap();
        let (st, o) = (stock(60.0, 460.0, 6.0), opts(0.0, 0.0, true));
        let n = nest(&st, &o, &[NestItem { label: "strip", quantity: 3, outline: &strip, height: 6.0 }]);
        assert_eq!(n.placements.len(), 3, "{:?}", n.unplaced);
        assert!(n.placements.iter().all(|p| p.quarter_turns == 1));
        check_layout(&n, &st, &o, &[&strip]);
        let n = nest(&st, &opts(0.0, 0.0, false), &[NestItem { label: "strip", quantity: 3, outline: &strip, height: 6.0 }]);
        assert_eq!(n.placements.len(), 0, "without turning nothing fits");
    }

    #[test]
    fn a_part_thicker_than_the_sheet_is_not_placed() {
        let sq = Area::rect(0.0, 0.0, 10.0, 10.0).unwrap();
        let n = nest(&stock(100.0, 100.0, 5.0), &opts(0.0, 0.0, true), &[NestItem { label: "tall", quantity: 2, outline: &sq, height: 8.0 }]);
        assert!(n.placements.is_empty());
        assert_eq!(n.unplaced.len(), 2);
        assert!(n.unplaced[0].reason.contains("8.000 mm tall"), "{}", n.unplaced[0].reason);
    }

    /// Mixed shapes away from the origin (round, L-shaped, long), many copies: whatever the
    /// packer chooses, the layout keeps the margins and spacing.
    #[test]
    fn mixed_parts_keep_margins_and_spacing() {
        let disc = Area::circle(-40.0, 25.0, 30.0, 0.01).unwrap();
        let ell = Area::rect(0.0, 0.0, 80.0, 20.0).unwrap().union(&Area::rect(0.0, 0.0, 20.0, 70.0).unwrap()).unwrap().placed(0, 500.0, -300.0);
        let bar = Area::rect(-100.0, -5.0, 100.0, 5.0).unwrap();
        let (st, o) = (stock(500.0, 300.0, 3.0), opts(8.0, 6.5, true));
        let items = [
            NestItem { label: "disc", quantity: 5, outline: &disc, height: 3.0 },
            NestItem { label: "ell", quantity: 6, outline: &ell, height: 2.0 },
            NestItem { label: "bar", quantity: 4, outline: &bar, height: 3.0 },
        ];
        let n = nest(&st, &o, &items);
        assert_eq!(n.placements.len() + n.unplaced.len(), 15);
        assert!(n.placements.len() >= 12, "{} placed, {:?}", n.placements.len(), n.unplaced);
        check_layout(&n, &st, &o, &[&disc, &ell, &bar]);
        // Deterministic: the same input gives the same layout.
        assert_eq!(nest(&st, &o, &items).placements, n.placements);
    }
}

mod plan_tests {
    use solidcraft_geom::Vec3;
    use solidcraft_kernel::{Body, BoolOp, boolean, box_solid, cylinder};

    use crate::job::Job;
    use crate::part::read_part;
    use crate::plan::{PATH_TOL, plan_part};
    use crate::region::{GRID, signed_area};
    use crate::toolpath::{Move, OpKind, Operation};

    fn v(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z)
    }

    fn cut(a: Body, b: Body) -> Body {
        boolean(&a, &b, BoolOp::Cut).unwrap().unwrap()
    }

    /// A tool position along the moves, with the move that made it.
    #[derive(Clone, Copy, Debug)]
    struct At {
        p: [f64; 3],
        rapid: bool,
        drill: bool,
    }

    /// Every move as tool positions: lines every 0.25 mm, arcs every 2°, drill cycles at their
    /// bottom. Independent of how the planner built them.
    fn sample(op: &Operation, start: [f64; 3]) -> Vec<At> {
        let mut out = vec![At { p: start, rapid: true, drill: false }];
        let mut cur = start;
        for m in &op.moves {
            match m {
                Move::Rapid { to } | Move::Line { to, .. } => {
                    let rapid = matches!(m, Move::Rapid { .. });
                    let d = ((to[0] - cur[0]).powi(2) + (to[1] - cur[1]).powi(2) + (to[2] - cur[2]).powi(2)).sqrt();
                    let n = (d / 0.25).ceil().max(1.0) as usize;
                    for i in 1..=n {
                        let t = i as f64 / n as f64;
                        out.push(At {
                            p: [cur[0] + (to[0] - cur[0]) * t, cur[1] + (to[1] - cur[1]) * t, cur[2] + (to[2] - cur[2]) * t],
                            rapid,
                            drill: false,
                        });
                    }
                    cur = *to;
                }
                Move::Arc { to, center, ccw, .. } => {
                    let r = (cur[0] - center[0]).hypot(cur[1] - center[1]);
                    let r_end = (to[0] - center[0]).hypot(to[1] - center[1]);
                    assert!((r - r_end).abs() < 1e-6, "arc radius changes: {r} → {r_end}");
                    let a0 = (cur[1] - center[1]).atan2(cur[0] - center[0]);
                    let a1 = (to[1] - center[1]).atan2(to[0] - center[0]);
                    let tau = std::f64::consts::TAU;
                    let mut sweep = if *ccw { (a1 - a0).rem_euclid(tau) } else { -(a0 - a1).rem_euclid(tau) };
                    if sweep.abs() < 1e-9 {
                        sweep = if *ccw { tau } else { -tau };
                    }
                    let n = (sweep.abs().to_degrees() / 2.0).ceil().max(1.0) as usize;
                    for i in 1..=n {
                        let t = i as f64 / n as f64;
                        let a = a0 + sweep * t;
                        out.push(At {
                            p: [center[0] + r * a.cos(), center[1] + r * a.sin(), cur[2] + (to[2] - cur[2]) * t],
                            rapid: false,
                            drill: false,
                        });
                    }
                    cur = *to;
                }
                Move::Drill { at, back, r, bottom, .. } => {
                    assert!((cur[0] - at[0]).hypot(cur[1] - at[1]) < 1e-9 && (cur[2] - back).abs() < 1e-9, "a cycle starts where the tool is");
                    out.push(At { p: [at[0], at[1], *r], rapid: true, drill: false });
                    out.push(At { p: [at[0], at[1], *bottom], rapid: false, drill: true });
                    cur = [at[0], at[1], *back];
                    out.push(At { p: cur, rapid: true, drill: false });
                }
            }
        }
        out
    }

    /// Every operation of `kind` joined into one, in order (roughing, walls and contours come
    /// one operation per level).
    fn op(ops: &[Operation], kind: OpKind) -> Operation {
        let of: Vec<&Operation> = ops.iter().filter(|o| o.kind == kind).collect();
        let first = of.first().unwrap_or_else(|| panic!("no {kind:?} operation: {:?}", ops.iter().map(|o| o.kind).collect::<Vec<_>>()));
        Operation {
            kind,
            name: first.name.clone(),
            tool: first.tool,
            rpm: first.rpm,
            moves: of.iter().flat_map(|o| o.moves.iter().cloned()).collect(),
        }
    }

    /// Distance from (x, y) to the outside of the axis-aligned box [x0, x1] × [y0, y1]: negative inside.
    fn rect_dist(p: [f64; 3], x0: f64, y0: f64, x1: f64, y1: f64) -> f64 {
        let dx = (x0 - p[0]).max(p[0] - x1);
        let dy = (y0 - p[1]).max(p[1] - y1);
        if dx <= 0.0 && dy <= 0.0 { dx.max(dy) } else { dx.max(0.0).hypot(dy.max(0.0)) }
    }

    fn job(tools: &str, thickness: f64, extra: &str) -> Job {
        Job::from_json(&format!(
            r#"{{"stock": {{"size": [300, 300], "thickness": {thickness}}}, "tools": [{tools}], "parts": [{{"body": "P"}}] {extra}}}"#
        ))
        .unwrap()
    }

    const FLAT6: &str =
        r#"{"number": 1, "kind": "flat", "diameter": 6, "flute_length": 22, "rpm": 18000, "feed": 1200, "plunge": 300, "stepdown": 3}"#;

    /// A 60 × 40 × 10 plate with a 30 × 20 pocket 4 deep in the middle (floor at z 6).
    fn pocket_plate() -> Body {
        let b = box_solid(v(-30.0, -20.0, 0.0), v(30.0, 20.0, 10.0)).unwrap();
        cut(b, box_solid(v(-15.0, -10.0, 6.0), v(15.0, 10.0, 11.0)).unwrap())
    }

    /// Roughing stays 3 + 0.5 mm off the pocket's walls and 0.2 mm above its floor; feeds below
    /// the top only inside the pocket; rapids that move sideways only at the clearance height.
    /// The only warnings are the pocket's four sharp corners, where the Ø6 tool (also the
    /// finishing tool) leaves 3(√2 − 1) = 1.24 mm.
    #[test]
    fn roughing_keeps_the_stock_to_leave() {
        let f = read_part(&pocket_plate()).unwrap();
        let plan = plan_part(&f, &job(FLAT6, 10.0, ""), "P").unwrap();
        assert_eq!(plan.warnings.len(), 4, "{:?}", plan.warnings);
        for (x, y) in [(-15.0, -10.0), (-15.0, 10.0), (15.0, -10.0), (15.0, 10.0)] {
            let text = format!("corner at ({x:.2}, {y:.2}) down to z 6.000: it leaves material up to 1.24 mm");
            assert!(plan.warnings.iter().any(|w| w.contains(&text)), "{text}: {:?}", plan.warnings);
        }
        let rough = &op(&plan.ops, OpKind::Roughing);
        let pts = sample(rough, [0.0, 0.0, 15.0]);
        assert!(pts.iter().all(|a| !a.drill), "roughing has no drilling cycles");
        let cutting: Vec<&At> = pts.iter().filter(|a| !a.rapid && a.p[2] < 10.0 - 1e-9).collect();
        assert!(!cutting.is_empty());
        for a in &cutting {
            let d = rect_dist(a.p, -15.0, -10.0, 15.0, 10.0);
            assert!(d <= -(3.5 - PATH_TOL - 1e-6), "roughing at {:?} is {} from the pocket wall", a.p, -d);
        }
        let low = cutting.iter().map(|a| a.p[2]).fold(f64::INFINITY, f64::min);
        assert!((low - 6.2).abs() < 1e-9, "roughing reaches {low}");
        // levels: 10 → 6.2 in steps of at most 3 (3.8 / 2 = 1.9 each)
        let mut zs: Vec<f64> = cutting.iter().map(|a| (a.p[2] * 1e6).round() / 1e6).collect();
        zs.dedup();
        assert!(zs.contains(&8.1) && zs.contains(&6.2), "{zs:?}");
        check_rapids(rough, 15.0);
    }

    /// Sideways rapids only at the clearance height; down at rapid never below the retract height
    /// above the sheet's top (10 + 1 here, or the level just cut).
    fn check_rapids(op: &Operation, clearance: f64) {
        let mut at = [0.0, 0.0, clearance];
        for m in &op.moves {
            if let Move::Rapid { to } = m {
                let sideways = (to[0] - at[0]).hypot(to[1] - at[1]) > 1e-9;
                if sideways {
                    assert!((to[2] - clearance).abs() < 1e-9 && (at[2] - clearance).abs() < 1e-9, "rapid across at z {} → {}: {m:?}", at[2], to[2]);
                }
            }
            at = m.end();
        }
    }

    /// The floor pass runs exactly on the floor and reaches every point of it a Ø6 tool can
    /// reach: the pocket less its four corner squares (3 × 3, where the round tool can't go).
    #[test]
    fn the_floor_is_finished_at_its_height_and_covered() {
        let f = read_part(&pocket_plate()).unwrap();
        let plan = plan_part(&f, &job(FLAT6, 10.0, ""), "P").unwrap();
        let pts = sample(&op(&plan.ops, OpKind::FloorFinish), [0.0, 0.0, 15.0]);
        let floor: Vec<[f64; 3]> = pts.iter().filter(|a| !a.rapid && (a.p[2] - 6.0).abs() < 1e-9).map(|a| a.p).collect();
        assert!(!floor.is_empty());
        for p in &floor {
            assert!(rect_dist(*p, -15.0, -10.0, 15.0, 10.0) <= -(3.0 - PATH_TOL - 1e-6), "floor pass at {p:?} cuts the wall");
        }
        assert!(pts.iter().all(|a| a.p[2] >= 6.0 - 1e-9), "nothing below the floor");
        let mut x: f64 = -15.0 + 0.05;
        while x < 15.0 {
            let mut y: f64 = -10.0 + 0.05;
            while y < 10.0 {
                let corner = (x.abs() > 12.0) && (y.abs() > 7.0);
                if !corner {
                    let near = floor.iter().any(|p| (p[0] - x).hypot(p[1] - y) <= 3.0 + 0.01);
                    assert!(near, "the floor point ({x:.2}, {y:.2}) is left uncut");
                }
                y += 0.5;
            }
            x += 0.5;
        }
    }

    /// The wall passes run 3.3, 3.1 and 3 mm inside each wall (the tool's radius and the 0.3,
    /// 0.1 and nothing each leaves), in that order at each level, clockwise round the pocket
    /// (wall on the left: climb milling), down to the floor.
    #[test]
    fn walls_are_finished_climbing_at_the_tool_radius() {
        let f = read_part(&pocket_plate()).unwrap();
        let plan = plan_part(&f, &job(FLAT6, 10.0, ""), "P").unwrap();
        let walls = &op(&plan.ops, OpKind::WallFinish);
        let pts = sample(walls, [0.0, 0.0, 15.0]);
        let at6: Vec<[f64; 2]> = pts.iter().filter(|a| !a.rapid && (a.p[2] - 6.0).abs() < 1e-9).map(|a| [a.p[0], a.p[1]]).collect();
        assert!(!at6.is_empty(), "the walls are finished down to the floor");
        let mut order: Vec<usize> = Vec::new();
        for p in &at6 {
            let d = -rect_dist([p[0], p[1], 0.0], -15.0, -10.0, 15.0, 10.0);
            let k = [3.3, 3.1, 3.0].iter().position(|w| (d - w).abs() <= PATH_TOL + 1e-6);
            let k = k.unwrap_or_else(|| panic!("wall pass at {p:?} is {d} from the wall"));
            if order.last() != Some(&k) {
                order.push(k);
            }
        }
        assert_eq!(order, vec![0, 1, 2], "0.3 left, then 0.1, then none");
        let last: Vec<[f64; 2]> =
            at6.iter().copied().filter(|p| (-rect_dist([p[0], p[1], 0.0], -15.0, -10.0, 15.0, 10.0) - 3.0).abs() <= PATH_TOL + 1e-6).collect();
        assert!(signed_area(&last) < 0.0, "clockwise round the pocket");
        for side in [[-12.0, 0.0], [12.0, 0.0], [0.0, -7.0], [0.0, 7.0]] {
            assert!(
                last.iter().any(|p| (p[0] - side[0]).abs() < 0.3 && (p[1] - side[1]).abs() < 0.3
                    || (side[0] == 0.0 && (p[1] - side[1]).abs() < 1e-6)
                    || (side[1] == 0.0 && (p[0] - side[0]).abs() < 1e-6)),
                "no pass along the wall at {side:?}"
            );
        }
    }

    const FLAT3: &str =
        r#"{"number": 2, "kind": "flat", "diameter": 3, "flute_length": 12, "rpm": 22000, "feed": 700, "plunge": 150, "stepdown": 1.5}"#;

    /// Distance from `p` to the nearer of the two plates (sheet coordinates).
    fn to_plates(p: [f64; 3]) -> f64 {
        rect_dist(p, 10.0, 10.0, 50.0, 40.0).min(rect_dist(p, 58.0, 10.0, 98.0, 40.0))
    }

    /// Work zero in the sheet's top centre (the default) moves every path by half the sheet:
    /// the same two plates planned with the corner as work zero, less (55, 25), move by move.
    #[test]
    fn work_zero_defaults_to_the_top_centre() {
        use crate::nest::Placement;
        use crate::plan::plan_sheet;
        let plate = box_solid(v(-20.0, -15.0, 0.0), v(20.0, 15.0, 6.0)).unwrap();
        let f = read_part(&plate).unwrap();
        let sheet = |origin: &str| {
            Job::from_json(&format!(
                r#"{{"stock": {{"size": [110, 50], "thickness": 6 {origin}}}, "tools": [{FLAT6}, {FLAT3}], "parts": [{{"body": "P"}}]}}"#
            ))
            .unwrap()
        };
        let pls = |copy: u32, x: f64| Placement { part: 0, label: "P".into(), copy, quarter_turns: 0, offset: [x, 25.0], bounds: [[0.0; 2]; 2] };
        let centre = sheet("");
        assert_eq!(centre.stock.work_zero(), [55.0, 25.0]);
        let corner = sheet(r#", "origin": "top_front_left""#);
        assert_eq!(corner.stock.work_zero(), [0.0, 0.0]);
        let plan = plan_part(&f, &centre, "P").unwrap();
        let (a, _) = plan_sheet(&centre, &[(&f, &plan)], &[pls(0, 30.0), pls(1, 78.0)]).unwrap();
        let (b, _) = plan_sheet(&corner, &[(&f, &plan)], &[pls(0, 30.0), pls(1, 78.0)]).unwrap();
        assert_eq!(a.len(), b.len());
        let mut n = 0;
        for (oa, ob) in a.iter().zip(&b) {
            assert_eq!(oa.moves.len(), ob.moves.len());
            for (ma, mb) in oa.moves.iter().zip(&ob.moves) {
                let (pa, pb) = (ma.end(), mb.end());
                assert!(
                    (pa[0] - (pb[0] - 55.0)).abs() < 1e-9 && (pa[1] - (pb[1] - 25.0)).abs() < 1e-9 && (pa[2] - pb[2]).abs() < 1e-12,
                    "{pa:?} vs {pb:?}"
                );
                n += 1;
            }
        }
        assert!(n > 1000);
    }

    /// Two 40 × 30 plates 6 thick on a 110 × 50 sheet 6 thick, 8 apart (x 10…50 and 58…98, y
    /// 10…40), with the `skin` cut-out (Ø6 roughing and contour, Ø3 finishing):
    /// - roughing clears a band round them down to the skin's top, −6 + 0.5 = −5.5, in levels of
    ///   at most 3 (−2.75, −5.5), its centre never nearer the plates than 3 + 0.5, through the
    ///   gap between them (centres between x 53.5 and 54.5) and all the way round at −5.5;
    /// - the contour's wall passes run 3.3 and 3.1 outside each plate at −2.75 and −5.5;
    /// - last of all the separation: its pass 3 outside each plate at −2.75 and −5.5, then
    ///   through the skin at −5.6, −5.7 … −6.2 (to the spoilboard allowance), both plates at each
    ///   level before the next, the tool only ever going down while it cuts (no tabs);
    /// - each kind of work goes level by level over the sheet, the deepest last.
    ///
    /// With the `profile` cut-out, none of that: a slot round each part instead.
    #[test]
    fn the_skin_cut_out_clears_round_the_parts_and_separates_them_last() {
        use crate::nest::Placement;
        use crate::plan::plan_sheet;
        let plate = box_solid(v(-20.0, -15.0, 0.0), v(20.0, 15.0, 6.0)).unwrap();
        let f = read_part(&plate).unwrap();
        let sheet = |extra: &str| {
            Job::from_json(&format!(
                r#"{{"stock": {{"size": [110, 50], "thickness": 6, "origin": "top_front_left"}}, "tools": [{FLAT6}, {FLAT3}], "parts": [{{"body": "P"}}] {extra}}}"#
            ))
            .unwrap()
        };
        let pls = |copy: u32, x: f64| Placement { part: 0, label: "P".into(), copy, quarter_turns: 0, offset: [x, 25.0], bounds: [[0.0; 2]; 2] };
        let j = sheet("");
        let plan = plan_part(&f, &j, "P").unwrap();
        let (ops, warnings) = plan_sheet(&j, &[(&f, &plan)], &[pls(0, 30.0), pls(1, 78.0)]).unwrap();
        assert!(warnings.is_empty(), "{warnings:#?}");

        let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
        // the band's roughing
        let band: Vec<&Operation> = ops.iter().filter(|o| o.name.starts_with("round the parts")).collect();
        let levels: Vec<f64> = band.iter().map(|o| o.deepest()).collect();
        assert!(levels.len() == 2 && near(levels[0], -2.75) && near(levels[1], -5.5), "{levels:?}");
        let mut in_gap = 0;
        for o in &band {
            for a in sample(o, [0.0, 0.0, 5.0]).iter().filter(|a| !a.rapid && a.p[2] < 0.0) {
                assert!(a.p[2] >= -5.5 - 1e-9, "below the skin at {:?}", a.p);
                assert!(to_plates(a.p) >= 3.5 - 0.006, "{:?} is {:.4} from a plate", a.p, to_plates(a.p));
                if near(a.p[2], -5.5) && a.p[0] > 53.5 - 0.01 && a.p[0] < 54.5 + 0.01 && a.p[1] > 15.0 && a.p[1] < 35.0 {
                    in_gap += 1;
                }
            }
        }
        assert!(in_gap > 50, "the gap is cut at the skin's top ({in_gap} samples)");
        // and all the way round: beside the middle of every side of both plates, 3.5 to 5 out
        let low: Vec<[f64; 3]> =
            band.iter().flat_map(|o| sample(o, [0.0, 0.0, 5.0])).filter(|a| !a.rapid && near(a.p[2], -5.5)).map(|a| a.p).collect();
        for (x0, x1) in [(10.0, 50.0), (58.0, 98.0)] {
            let xm = (x0 + x1) / 2.0;
            for (side, q) in [("front", [xm, 10.0 - 4.25]), ("back", [xm, 40.0 + 4.25]), ("left", [x0 - 4.25, 25.0]), ("right", [x1 + 4.25, 25.0])] {
                assert!(
                    low.iter().any(|p| (p[0] - q[0]).hypot(p[1] - q[1]) < 1.0),
                    "nothing cut at −5.5 beside the {side} of the plate at x {x0}…{x1}"
                );
            }
        }

        // the contour: the wall passes leaving 0.3 and 0.1, 3.3 and 3.1 outside each plate,
        // level by level down to the skin's top
        let contour: Vec<&Operation> = ops.iter().filter(|o| o.kind == OpKind::Profile).collect();
        let levels: Vec<f64> = contour.iter().map(|o| o.deepest()).collect();
        assert_eq!(levels.len(), 4, "{levels:?}");
        assert!(near(levels[0], -2.75) && near(levels[1], -2.75) && near(levels[2], -5.5) && near(levels[3], -5.5), "{levels:?}");
        // the separation: the last pass, 3 outside each plate, at −2.75 and −5.5, then through the
        // skin; level by level, both plates at each
        let sep: Vec<&Operation> = ops.iter().filter(|o| o.kind == OpKind::Separation).collect();
        let want: Vec<f64> = [-2.75, -5.5].into_iter().chain((1..=7).map(|k| -5.5 - 0.1 * k as f64)).flat_map(|z| [z; 2]).collect();
        let got: Vec<f64> = sep.iter().map(|o| o.deepest()).collect();
        assert!(got.len() == want.len() && got.iter().zip(&want).all(|(a, b)| (a - b).abs() < 1e-9), "{got:?}");
        for o in contour.iter().chain(&sep) {
            assert_eq!(o.tool, 1, "the Ø6");
            let level = o.deepest();
            let offsets: &[f64] = if o.kind == OpKind::Profile { &[3.3, 3.1] } else { &[3.0] };
            let mut seen = vec![false; offsets.len()];
            // between rapids the tool only ever goes down (no tabs to climb over)
            let mut prev = f64::INFINITY;
            for a in sample(o, [0.0, 0.0, 5.0]) {
                if a.rapid {
                    prev = f64::INFINITY;
                    continue;
                }
                assert!(a.p[2] <= prev + 1e-9, "{:?} rises", a.p);
                prev = a.p[2];
                if a.p[2] < 0.0 {
                    let k = offsets.iter().position(|w| (to_plates(a.p) - w).abs() <= 0.006);
                    let k = k.unwrap_or_else(|| panic!("{:?} is {:.4} from a plate", a.p, to_plates(a.p)));
                    seen[k] = true;
                    assert!(a.p[2] >= level - 1e-9);
                }
            }
            assert!(seen.iter().all(|s| *s), "{:?} at {level}: every pass", o.kind);
        }
        // kind by kind, each level by level over the sheet; the separation last
        let first_sep = ops.iter().position(|o| o.kind == OpKind::Separation).unwrap();
        assert!(ops[first_sep..].iter().all(|o| o.kind == OpKind::Separation));
        assert!(ops.windows(2).all(|w| w[0].kind.rank() <= w[1].kind.rank()));
        for kind in [OpKind::Roughing, OpKind::Profile, OpKind::Separation] {
            let d: Vec<f64> = ops.iter().filter(|o| o.kind == kind).map(|o| o.deepest()).collect();
            assert!(d.windows(2).all(|w| w[1] <= w[0] + 1e-9), "{kind:?} by depth: {d:?}");
        }

        let j = sheet(r#", "strategy": {"cut_out": {"method": "profile"}}"#);
        let plan = plan_part(&f, &j, "P").unwrap();
        let (ops, _) = plan_sheet(&j, &[(&f, &plan)], &[pls(0, 30.0), pls(1, 78.0)]).unwrap();
        assert!(ops.iter().all(|o| o.kind != OpKind::Separation && !o.name.starts_with("round the parts")));
        assert!(ops.iter().any(|o| o.kind == OpKind::Profile && o.deepest() < -6.0));
    }

    /// A 60 × 40 plate 6 thick with a 20 × 14 cutout through its middle, `skin` cut-out: the
    /// cutout is roughed and its walls finished down to the skin's top (z 0.5), no deeper; the
    /// separation's last pass runs 3 round the outside down to the skin, then the skin is cut
    /// through inside the cutout climbing (clockwise) on the 14 × 8 rectangle 3 in from its
    /// walls, as well as round the outside, down to −0.2.
    #[test]
    fn the_skin_is_cut_through_inside_a_cutout_too() {
        let plate = box_solid(v(-30.0, -20.0, 0.0), v(30.0, 20.0, 6.0)).unwrap();
        let b = cut(plate, box_solid(v(-10.0, -7.0, -1.0), v(10.0, 7.0, 7.0)).unwrap());
        let f = read_part(&b).unwrap();
        let j = Job::from_json(&format!(
            r#"{{"stock": {{"size": [300, 300], "thickness": 6}}, "tools": [{FLAT6}, {FLAT3}], "parts": [{{"body": "P"}}]}}"#
        ))
        .unwrap();
        let plan = plan_part(&f, &j, "P").unwrap();
        for kind in [OpKind::Roughing, OpKind::WallFinish] {
            let d = plan.ops.iter().filter(|o| o.kind == kind).map(|o| o.deepest()).fold(f64::INFINITY, f64::min);
            assert!((d - 0.5).abs() < 1e-9, "{kind:?} down to {d}");
        }
        // the separation's last pass round the outline at 3.25 and 0.5 (6 down to the skin in
        // levels of at most 3), then the skin, 0.5 to −0.2 in 0.1 steps, inside the cutout too
        let all: Vec<f64> = plan.ops.iter().filter(|o| o.kind == OpKind::Separation).map(|o| o.deepest()).collect();
        assert_eq!(all.len(), 9, "{all:?}");
        assert!((all[0] - 3.25).abs() < 1e-9 && (all[1] - 0.5).abs() < 1e-9, "{all:?}");
        let sep: Vec<&Operation> = plan.ops.iter().filter(|o| o.kind == OpKind::Separation && o.deepest() < 0.5 - 1e-9).collect();
        assert_eq!(sep.len(), 7, "0.5 to −0.2 in 0.1 steps");
        for o in &sep {
            let inside: Vec<[f64; 3]> =
                sample(o, [0.0, 0.0, 11.0]).iter().filter(|a| !a.rapid && a.p[0].abs() < 10.0 && a.p[1].abs() < 7.0).map(|a| a.p).collect();
            assert!(inside.len() > 100);
            for p in &inside {
                let on = (p[0].abs() - 7.0).max(p[1].abs() - 4.0);
                assert!(on.abs() < 1e-6, "{p:?} is not on the 14 × 8 rectangle");
            }
            let ring: Vec<[f64; 2]> = inside.iter().map(|p| [p[0], p[1]]).collect();
            assert!(signed_area(&ring) < 0.0, "clockwise inside the cutout");
            let outside = sample(o, [0.0, 0.0, 11.0]).iter().filter(|a| !a.rapid && (a.p[0].abs() > 25.0 || a.p[1].abs() > 15.0)).count();
            assert!(outside > 100, "and round the outside");
        }
    }

    /// The profile runs 3 mm outside the outline, anticlockwise (part on the left), down to the
    /// spoilboard allowance (−0.2), except over four tabs 1 mm high and 6 + 6 mm long.
    #[test]
    fn the_profile_cuts_through_round_four_tabs() {
        let f = read_part(&pocket_plate()).unwrap();
        let plan = plan_part(&f, &job(FLAT6, 10.0, r#", "strategy": {"cut_out": {"method": "profile"}}"#), "P").unwrap();
        let prof = &op(&plan.ops, OpKind::Profile);
        let pts = sample(prof, [0.0, 0.0, 15.0]);
        let cutting: Vec<[f64; 3]> = pts.iter().filter(|a| !a.rapid && a.p[2] < 10.0 - 1e-9).map(|a| a.p).collect();
        for p in &cutting {
            let d = rect_dist(*p, -30.0, -20.0, 30.0, 20.0);
            assert!((d - 3.0).abs() <= PATH_TOL + 1e-6, "profile at {p:?} is {d} from the outline");
        }
        let low = cutting.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
        assert!((low + 0.2).abs() < 1e-9, "through to {low}");
        // the deepest pass: the stretches still at tab height
        let deepest: Vec<[f64; 3]> = cutting.iter().copied().filter(|p| p[2] < 1.0 + 1e-9).collect();
        let ring: Vec<[f64; 2]> = deepest.iter().map(|p| [p[0], p[1]]).collect();
        assert!(signed_area(&ring) > 0.0, "anticlockwise round the part");
        let mut tabs = 0;
        let mut on = false;
        let mut tab_len = 0.0;
        let mut lens = Vec::new();
        for w in deepest.windows(2) {
            let up = (w[1][2] - 1.0).abs() < 1e-9 && (w[0][2] - 1.0).abs() < 1e-9;
            if up {
                tab_len += (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]);
            }
            if up && !on {
                tabs += 1;
            }
            if !up && on {
                lens.push(tab_len);
                tab_len = 0.0;
            }
            on = up;
        }
        assert_eq!(tabs, 4, "four tabs");
        for l in &lens {
            assert!((l - 12.0).abs() < 0.3, "a tab {l} long (6 + the 6 mm cutter)");
        }
        check_rapids(prof, 15.0);
    }

    /// A Ø10 through hole: with a Ø10 drill it is one drilling cycle at its centre, down past the
    /// spoilboard allowance by the drill's point (5 / tan 59°); without one, the Ø6 mill cuts it
    /// with circles no wider than 10 − 6 across, through to −0.2.
    #[test]
    fn a_through_hole_is_drilled_or_milled() {
        let b = cut(box_solid(v(0.0, 0.0, 0.0), v(40.0, 40.0, 8.0)).unwrap(), cylinder(v(20.0, 20.0, -1.0), v(0.0, 0.0, 1.0), 5.0, 10.0).unwrap());
        let f = read_part(&b).unwrap();
        let drill = r#"{"number": 5, "kind": "drill", "diameter": 10, "flute_length": 40, "rpm": 1500, "feed": 120, "plunge": 120, "peck": 2}"#;
        let plan = plan_part(&f, &job(&format!("{FLAT6}, {drill}"), 8.0, ""), "P").unwrap();
        let d = &op(&plan.ops, OpKind::Drilling);
        assert_eq!(d.tool, 5);
        let tip = 5.0 / 59f64.to_radians().tan();
        match d.moves.as_slice() {
            [Move::Rapid { .. }, Move::Drill { at, r, bottom, peck, .. }] => {
                assert!((at[0] - 20.0).abs() < 1e-6 && (at[1] - 20.0).abs() < 1e-6);
                assert!((r - 9.0).abs() < 1e-9 && (bottom - (-0.2 - tip)).abs() < 1e-9 && *peck == Some(2.0), "{:?}", d.moves);
            }
            m => panic!("a rapid over the hole and one drilling cycle, not {m:?}"),
        }
        assert!(!plan.ops.iter().any(|o| matches!(o.kind, OpKind::HoleMilling | OpKind::ThroughHole)));

        // milled through: a through hole of its own kind (run near the end: it leaks vacuum)
        let plan = plan_part(&f, &job(FLAT6, 8.0, ""), "P").unwrap();
        assert!(!plan.ops.iter().any(|o| o.kind == OpKind::HoleMilling));
        let pts = sample(&op(&plan.ops, OpKind::ThroughHole), [20.0, 20.0, 13.0]);
        let cutting: Vec<[f64; 3]> = pts.iter().filter(|a| !a.rapid && a.p[2] < 8.0).map(|a| a.p).collect();
        for p in &cutting {
            assert!((p[0] - 20.0).hypot(p[1] - 20.0) <= 2.0 + 1e-6, "the mill at {p:?} cuts the hole's wall");
        }
        assert!(cutting.iter().any(|p| ((p[0] - 20.0).hypot(p[1] - 20.0) - 2.0).abs() < 1e-6), "the last circle is on the wall");
        let low = cutting.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
        assert!((low + 0.2).abs() < 1e-9, "through to {low}");
    }

    /// A counterbore Ø10 × 4 over a Ø5 hole with a Ø3 mill: circles within 5 − 1.5 across down
    /// to the seat (z 4), within 2.5 − 1.5 below it, and nothing wider below the seat.
    #[test]
    fn a_counterbore_is_milled_step_by_step() {
        let mut b = box_solid(v(0.0, 0.0, 0.0), v(40.0, 40.0, 8.0)).unwrap();
        b = cut(b, cylinder(v(20.0, 20.0, 4.0), v(0.0, 0.0, 1.0), 5.0, 5.0).unwrap());
        b = cut(b, cylinder(v(20.0, 20.0, -1.0), v(0.0, 0.0, 1.0), 2.5, 10.0).unwrap());
        let f = read_part(&b).unwrap();
        let flat3 = r#"{"number": 2, "kind": "flat", "diameter": 3, "flute_length": 12, "rpm": 20000, "feed": 600, "plunge": 150, "stepdown": 1.5}"#;
        let plan = plan_part(&f, &job(&format!("{FLAT6}, {flat3}"), 8.0, ""), "P").unwrap();
        let ops = &plan.ops;
        // the counterbore (blind) and the hole under it (through, run later) together
        let mut holes = op(ops, OpKind::HoleMilling);
        holes.moves.extend(op(ops, OpKind::ThroughHole).moves);
        let holes = &holes;
        assert_eq!(holes.tool, 2, "the smaller mill finishes holes");
        let pts = sample(holes, [20.0, 20.0, 13.0]);
        for a in pts.iter().filter(|a| !a.rapid && a.p[2] < 8.0) {
            let r = (a.p[0] - 20.0).hypot(a.p[1] - 20.0);
            let wall = if a.p[2] >= 4.0 - 1e-9 { 3.5 } else { 1.0 };
            assert!(r <= wall + 1e-6, "at {:?} the mill is {r} from the axis (wall {wall})", a.p);
        }
        let seat = pts.iter().filter(|a| !a.rapid && (a.p[2] - 4.0).abs() < 1e-9).map(|a| (a.p[0] - 20.0).hypot(a.p[1] - 20.0)).fold(0.0, f64::max);
        assert!((seat - 3.5).abs() < 1e-6, "the seat is cut out to its wall: {seat}");
    }

    #[test]
    fn levels_land_on_every_target() {
        use crate::plan::levels_for_tests as levels;
        assert_eq!(levels(10.0, &[6.2], 3.0), vec![8.1, 6.2]);
        let l = levels(10.0, &[7.0, 2.0, -0.2], 2.0);
        assert_eq!(l.first(), Some(&8.5));
        for t in [7.0, 2.0, -0.2] {
            assert!(l.iter().any(|z| (z - t).abs() < 1e-12), "{l:?} misses {t}");
        }
        assert!(l.windows(2).all(|w| w[0] - w[1] <= 2.0 + 1e-12 && w[0] > w[1]));
        assert!(levels(5.0, &[6.0], 1.0).is_empty(), "nothing above the start");
    }

    #[test]
    fn tabs_split_the_ring_evenly() {
        use crate::toolpath::with_tabs;
        let ring = vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]];
        let pts = with_tabs(&ring, 4, 10.0);
        let mut on = 0.0;
        for w in pts.windows(2) {
            if w[0].1 && w[1].1 || (w[0].1 && !w[1].1) {
                on += (w[1].0[0] - w[0].0[0]).hypot(w[1].0[1] - w[0].0[1]);
            }
        }
        assert!((on - 40.0).abs() < 1e-9, "4 tabs of 10: {on}");
        assert_eq!(pts.first().map(|p| p.0), pts.last().map(|p| p.0), "closed");
        let _ = GRID;
    }
}

mod adaptive_tests {
    use solidcraft_geom::Vec3;
    use solidcraft_kernel::{Body, BoolOp, boolean, box_solid};

    use crate::job::Job;
    use crate::part::read_part;
    use crate::plan::plan_part;
    use crate::toolpath::{Move, OpKind, Operation, StrokeKind, trace};

    fn v(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z)
    }

    /// A 60 × 40 × 10 plate with pockets cut 6 deep (floor at z 4), each [x0, y0, x1, y1].
    fn plate(pockets: &[[f64; 4]]) -> Body {
        let mut b = box_solid(v(-30.0, -20.0, 0.0), v(30.0, 20.0, 10.0)).unwrap();
        for p in pockets {
            b = boolean(&b, &box_solid(v(p[0], p[1], 4.0), v(p[2], p[3], 11.0)).unwrap(), BoolOp::Cut).unwrap().unwrap();
        }
        b
    }

    fn job(extra: &str) -> Job {
        Job::from_json(&format!(
            r#"{{"stock": {{"size": [300, 300], "thickness": 10}},
                "tools": [{{"number": 1, "kind": "flat", "diameter": 6, "flute_length": 22, "rpm": 18000, "feed": 1200, "plunge": 300, "stepdown": 3, "stepover": 0.15}}],
                "strategy": {{"roughing": {{{extra}}}}},
                "parts": [{{"body": "P"}}]}}"#
        ))
        .unwrap()
    }

    fn roughing(b: &Body, j: &Job) -> Operation {
        let plan = plan_part(&read_part(b).unwrap(), j, "P").unwrap();
        let of: Vec<Operation> = plan.ops.into_iter().filter(|o| o.kind == OpKind::Roughing).collect();
        let first = of.first().unwrap();
        Operation {
            kind: OpKind::Roughing,
            name: first.name.clone(),
            tool: first.tool,
            rpm: first.rpm,
            moves: of.iter().flat_map(|o| o.moves.iter().cloned()).collect(),
        }
    }

    /// The tool's positions along the roughing every 0.05 mm, stroke by stroke (a stroke: cuts
    /// between rapids), each with its direction of travel.
    struct Sample {
        p: [f64; 3],
        dir: [f64; 2],
    }

    fn strokes(op: &Operation) -> Vec<Vec<Sample>> {
        let mut out = Vec::new();
        for s in trace(op, [f64::NAN, f64::NAN, 15.0], 1f64.to_radians()) {
            if s.kind != StrokeKind::Cut {
                continue;
            }
            let mut st = Vec::new();
            for w in s.pts.windows(2) {
                let (a, b) = (w[0], w[1]);
                let len = (b[0] - a[0]).hypot(b[1] - a[1]);
                let n = ((len.max((b[2] - a[2]).abs())) / 0.05).ceil().max(1.0) as usize;
                let dir = if len > 1e-12 { [(b[0] - a[0]) / len, (b[1] - a[1]) / len] } else { [0.0, 0.0] };
                for k in 1..=n {
                    let t = k as f64 / n as f64;
                    st.push(Sample { p: [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t], dir });
                }
            }
            out.push(st);
        }
        out
    }

    /// An independent replay on a 0.04 mm grid, level by level. A sample's engagement is the
    /// textbook one: the arc of the cutter's forward half (rays ahead of its travel) over
    /// material not yet cut before this stroke, acos(1 − a/r) for a straight cut a deep. Each
    /// stroke is marked cut once it is done.
    struct Replay {
        h: f64,
        x0: f64,
        y0: f64,
        nx: usize,
        ny: usize,
        cut: Vec<bool>,
    }

    impl Replay {
        fn new() -> Replay {
            Replay::over([-31.0, -21.0], [31.0, 21.0], 0.04)
        }
        /// An uncut grid over `lo`..`hi` with cells `h` wide.
        fn over(lo: [f64; 2], hi: [f64; 2], h: f64) -> Replay {
            let (nx, ny) = (((hi[0] - lo[0]) / h) as usize, ((hi[1] - lo[1]) / h) as usize);
            Replay { h, x0: lo[0], y0: lo[1], nx, ny, cut: vec![false; nx * ny] }
        }
        fn inside(&self, p: [f64; 3]) -> bool {
            self.idx(p[0], p[1]).is_some()
        }
        fn idx(&self, x: f64, y: f64) -> Option<usize> {
            let (i, j) = (((x - self.x0) / self.h).floor(), ((y - self.y0) / self.h).floor());
            (i >= 0.0 && j >= 0.0 && (i as usize) < self.nx && (j as usize) < self.ny).then(|| j as usize * self.nx + i as usize)
        }
        fn is_cut(&self, x: f64, y: f64) -> bool {
            self.idx(x, y).is_some_and(|i| self.cut[i])
        }
        /// (engaged arc of the forward half in degrees, uncut rays on the left of `dir`, on
        /// the right)
        fn engagement(&self, p: [f64; 3], dir: [f64; 2], r: f64) -> (f64, usize, usize) {
            let (mut n, mut left, mut right) = (0, 0, 0);
            for k in 0..192 {
                let a = k as f64 * std::f64::consts::TAU / 192.0;
                let (cx, cy) = (a.cos(), a.sin());
                if cx * dir[0] + cy * dir[1] <= 0.0 {
                    continue;
                }
                if !self.is_cut(p[0] + 0.98 * r * cx, p[1] + 0.98 * r * cy) {
                    n += 1;
                    let side = dir[0] * cy - dir[1] * cx;
                    if side > 1e-9 {
                        left += 1;
                    } else if side < -1e-9 {
                        right += 1;
                    }
                }
            }
            (n as f64 * 360.0 / 192.0, left, right)
        }
        /// The cells within `r` of the segment `a`–`b` (both ends included).
        fn capsule(&mut self, a: [f64; 3], b: [f64; 3], r: f64) {
            let (lo, hi) = ([a[0].min(b[0]) - r, a[1].min(b[1]) - r], [a[0].max(b[0]) + r, a[1].max(b[1]) + r]);
            let i0 = (((lo[0] - self.x0) / self.h).floor().max(0.0)) as usize;
            let j0 = (((lo[1] - self.y0) / self.h).floor().max(0.0)) as usize;
            let i1 = ((((hi[0] - self.x0) / self.h).ceil().max(0.0)) as usize).min(self.nx.saturating_sub(1));
            let j1 = ((((hi[1] - self.y0) / self.h).ceil().max(0.0)) as usize).min(self.ny.saturating_sub(1));
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let l2 = dx * dx + dy * dy;
            for j in j0..=j1 {
                for i in i0..=i1 {
                    let (x, y) = (self.x0 + (i as f64 + 0.5) * self.h, self.y0 + (j as f64 + 0.5) * self.h);
                    let t = if l2 > 0.0 { (((x - a[0]) * dx + (y - a[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
                    if (x - a[0] - t * dx).hypot(y - a[1] - t * dy) <= r {
                        self.cut[j * self.nx + i] = true;
                    }
                }
            }
        }
        fn stamp(&mut self, p: [f64; 3], r: f64) {
            let k = (r / self.h).ceil() as i64 + 1;
            let (ci, cj) = (((p[0] - self.x0) / self.h) as i64, ((p[1] - self.y0) / self.h) as i64);
            for dj in -k..=k {
                for di in -k..=k {
                    let (i, j) = (ci + di, cj + dj);
                    if i < 0 || j < 0 || i as usize >= self.nx || j as usize >= self.ny {
                        continue;
                    }
                    let (x, y) = (self.x0 + (i as f64 + 0.5) * self.h, self.y0 + (j as f64 + 0.5) * self.h);
                    if (x - p[0]).hypot(y - p[1]) <= r {
                        self.cut[j as usize * self.nx + i as usize] = true;
                    }
                }
            }
        }
    }

    /// Does the trail need marking up to `p`: 0.5 mm since the last mark, the path turning there
    /// by more than 5° (a chord past a corner would miss it), or the trail's end?
    fn needs_stamp(q: [f64; 3], p: [f64; 3], next: Option<[f64; 3]>) -> bool {
        let Some(n) = next else { return true };
        let (ax, ay, bx, by) = (p[0] - q[0], p[1] - q[1], n[0] - p[0], n[1] - p[1]);
        let (la, lb) = (ax.hypot(ay), bx.hypot(by));
        let turn = if la > 1e-9 && lb > 1e-9 { ((ax * bx + ay * by) / (la * lb)).clamp(-1.0, 1.0).acos() } else { 0.0 };
        la >= 0.5 || turn > 5f64.to_radians()
    }

    /// What the replay saw on the level at `z`: each sample's engagement and where it was, and
    /// the grid after.
    struct Seen {
        engaged: Vec<(f64, usize, usize)>,
        at: Vec<[f64; 3]>,
        replay: Replay,
    }

    /// Replay the level at `z`; `above` is the level over it (the stock's top for the first).
    fn replay_level(op: &Operation, z: f64, above: f64) -> Seen {
        replay_level_on(&strokes(op), z, above, Replay::new())
    }

    /// The same on a grid of its own: samples off the grid are skipped (neither measured nor
    /// marked), so the grid should cover the pocket with a cutter's radius to spare.
    fn replay_level_on(all: &[Vec<Sample>], z: f64, above: f64, mut r: Replay) -> Seen {
        let mut engaged = Vec::new();
        let mut at = Vec::new();
        for st in all {
            // positions are marked cut once the tool is 1 mm further along: nearer ones would
            // cover the forward ring (anything within 0.2 r behind does) and hide the cut
            let mut trail: std::collections::VecDeque<([f64; 3], f64)> = Default::default();
            let mut along = 0.0;
            let mut prev: Option<[f64; 3]> = None;
            let mut last_stamped: Option<[f64; 3]> = None;
            // entering the level (a helix or ramp coming down onto it, less than a stepdown
            // above): cut as it goes, not measured (an entry plunges, it doesn't cut sideways)
            let entering: Vec<[f64; 3]> = st.iter().filter(|s| s.p[2] > z + 1e-9 && s.p[2] < above - 1e-9 && r.inside(s.p)).map(|s| s.p).collect();
            for w in entering.chunks(10).filter_map(|c| Some((*c.first()?, *c.last()?))) {
                r.capsule(w.0, w.1, 3.0);
            }
            if let [p] = entering.as_slice() {
                r.stamp(*p, 3.0);
            }
            let on_level: Vec<&Sample> = st.iter().filter(|s| (s.p[2] - z).abs() < 1e-9 && r.inside(s.p)).collect();
            for s in on_level {
                if let Some(q) = prev {
                    along += (s.p[0] - q[0]).hypot(s.p[1] - q[1]);
                }
                prev = Some(s.p);
                // marked in capsules about 0.5 mm long (a path bending at r 2 sags 0.016 off
                // such a chord): one capsule costs as much as a disc
                while trail.front().is_some_and(|(_, a)| along - a > 1.0) {
                    let (p, _) = trail.pop_front().unwrap();
                    let next = trail.front().map(|t| t.0);
                    match last_stamped {
                        Some(q) if !needs_stamp(q, p, next) => continue,
                        Some(q) => r.capsule(q, p, 3.0),
                        None => r.stamp(p, 3.0),
                    }
                    last_stamped = Some(p);
                }
                let e = r.engagement(s.p, s.dir, 3.0);
                engaged.push(e);
                at.push(s.p);
                trail.push_back((s.p, along));
            }
            let rest: Vec<[f64; 3]> = trail.into_iter().map(|(p, _)| p).collect();
            for (k, p) in rest.iter().enumerate() {
                match last_stamped {
                    Some(q) if !needs_stamp(q, *p, rest.get(k + 1).copied()) => continue,
                    Some(q) => r.capsule(q, *p, 3.0),
                    None => r.stamp(*p, 3.0),
                }
                last_stamped = Some(*p);
            }
        }
        Seen { engaged, at, replay: r }
    }

    /// The pocket's region the cutter's centre may reach at a level: the 40 × 24 pocket shrunk
    /// by the radius (3) and the stock to leave on walls (0.5): |x| ≤ 16.5, |y| ≤ 8.5.
    fn in_reach(p: [f64; 3]) -> bool {
        p[0].abs() <= 16.5 + 0.01 && p[1].abs() <= 8.5 + 0.01
    }

    /// The most a pass may engage within a step (0.9) of the region's edge, where a front meets a
    /// wall: a quarter of the cutter and a straight cut's engagement at the step, 90° +
    /// acos(1 − 0.9/3) = 135.6°.
    fn wall_limit() -> f64 {
        90.0 + (1.0f64 - 0.9 / 3.0).acos().to_degrees()
    }

    /// Engagement at the samples (each with its distance from the centres' region's edge): within
    /// the limit (default here 90°: 1.3 × the straight cut's 45.6° is less) farther than a step
    /// (and 0.1 of slack) from the edge, within `wall_limit` nearer, to within the replay's
    /// resolution (rays every 1.9°, a 0.04 mm grid): p99 and max of each.
    fn engagement_ok(what: &str, engaged: &[(f64, f64)]) {
        let split = |wall: bool| {
            let mut e: Vec<f64> = engaged.iter().filter(|(_, d)| (*d <= 0.9 + 0.1) == wall).map(|(e, _)| *e).collect();
            e.sort_by(f64::total_cmp);
            e
        };
        for (wall, limit) in [(false, 90.0), (true, wall_limit())] {
            let e = split(wall);
            if e.is_empty() {
                continue;
            }
            let (p99, max) = (e[(e.len() - 1) * 99 / 100], *e.last().unwrap());
            assert!(
                p99 <= limit && max <= limit + 6.0,
                "{what}, {}: p99 {p99:.1}°, max {max:.1}° (limit {limit:.1}°)",
                if wall { "by the walls" } else { "inside" }
            );
        }
    }

    /// Engagement stays within the limits (`engagement_ok`) at both levels; no move takes the
    /// centre outside its region; and the last level is cut everywhere the cutter can reach.
    #[test]
    fn adaptive_keeps_engagement_and_clears_the_pocket() {
        let op = roughing(&plate(&[[-20.0, -12.0, 20.0, 12.0]]), &job(""));
        for (z, above) in [(7.1, 10.0), (4.2, 7.1)] {
            let seen = replay_level(&op, z, above);
            let mut e: Vec<f64> = seen.engaged.iter().map(|e| e.0).collect();
            assert!(e.len() > 1000, "level {z} has {} samples", e.len());
            let edge = |p: [f64; 3]| (16.5 - p[0].abs()).min(8.5 - p[1].abs());
            let with: Vec<(f64, f64)> = seen.engaged.iter().zip(&seen.at).map(|(e, p)| (e.0, edge(*p))).collect();
            engagement_ok(&format!("level {z}"), &with);
            e.sort_by(f64::total_cmp);
            let median = e[e.len() / 2];
            assert!(median > 20.0, "level {z}: the cutter mostly works ({median:.1}° median)");
            if z == 4.2 {
                let mut x: f64 = -19.25;
                while x < 19.25 {
                    let mut y: f64 = -11.25;
                    while y < 11.25 {
                        let corner = x.abs() > 16.5 && y.abs() > 8.5;
                        assert!(corner || seen.replay.is_cut(x, y), "({x:.2}, {y:.2}) left uncut");
                        y += 0.25;
                    }
                    x += 0.25;
                }
            }
        }
        for st in strokes(&op) {
            for s in st.iter().filter(|s| s.p[2] < 10.0) {
                assert!(in_reach(s.p), "the centre at {:?} is outside its region", s.p);
            }
        }
    }

    /// The time a roughing takes, minutes: each move's length over its feed, rapids at `rapid`
    /// mm/min. An arc's length is its turn (a full turn where it ends where it starts) times
    /// its radius, with its drop.
    fn minutes(op: &Operation, rapid: f64) -> f64 {
        let mut at = [f64::NAN; 3];
        let mut total = 0.0;
        for m in &op.moves {
            let e = m.end();
            let (len, feed) = match m {
                Move::Rapid { .. } => ((e[0] - at[0]).hypot(e[1] - at[1]).hypot(e[2] - at[2]), rapid),
                Move::Line { feed, .. } => ((e[0] - at[0]).hypot(e[1] - at[1]).hypot(e[2] - at[2]), *feed),
                Move::Arc { center, ccw, feed, .. } => {
                    let a0 = (at[1] - center[1]).atan2(at[0] - center[0]);
                    let a1 = (e[1] - center[1]).atan2(e[0] - center[0]);
                    let tau = std::f64::consts::TAU;
                    let mut turn = if *ccw { a1 - a0 } else { a0 - a1 };
                    turn = turn.rem_euclid(tau);
                    if turn < 1e-9 {
                        turn = tau;
                    }
                    let r = (e[0] - center[0]).hypot(e[1] - center[1]);
                    ((turn * r).hypot(e[2] - at[2]), *feed)
                }
                Move::Drill { .. } => (0.0, 1.0),
            };
            if len.is_finite() {
                total += len / feed;
            }
            at = e;
        }
        total
    }

    fn rapid_job(rapid: f64) -> Job {
        let mut j = job("");
        j.strategy.rapid_feed = rapid;
        j
    }

    /// Times up to clearance height.
    fn lifts_to_clearance(op: &Operation) -> usize {
        op.moves.windows(2).filter(|w| w[1].end()[2] >= 15.0 - 1e-9 && w[0].end()[2] < 15.0 - 1e-9).count()
    }

    /// One way, every engaged sample (over 10°) has the uncut material on its left: climbing.
    /// Both ways, a good share cut with it on the right (conventional), and the job is quicker
    /// (fewer links: it starts each stretch at whichever end is nearer).
    #[test]
    fn one_way_climbs_and_both_ways_is_quicker() {
        let side = |op: &Operation| {
            let seen = replay_level(op, 7.1, 10.0);
            let engaged: Vec<_> = seen.engaged.iter().filter(|e| e.0 > 10.0).collect();
            let left = engaged.iter().filter(|e| e.1 > e.2).count();
            (left as f64 / engaged.len() as f64, engaged.len())
        };
        let b = plate(&[[-20.0, -12.0, 20.0, 12.0]]);
        let one = roughing(&b, &job(""));
        let both = roughing(&b, &job(r#""both_ways": true"#));
        let (left_one, n_one) = side(&one);
        let (left_both, _) = side(&both);
        assert!(left_one >= 0.97, "one way: {:.1} % of {n_one} engaged samples climb", left_one * 100.0);
        assert!(left_both < 0.9, "both ways: {:.1} % climb", left_both * 100.0);
        let (t_one, t_both) = (minutes(&one, 5000.0), minutes(&both, 5000.0));
        assert!(t_both < t_one, "both ways takes {t_both:.2} min, one way {t_one:.2}");
    }

    /// An L-shaped pocket (a 50 × 12 arm along x and a 12 × 30 arm along y, meeting at the
    /// right-hand end) is cleared arm by arm: at each level, following the samples that cut
    /// (engaged over 10°), the tool passes between the arms' far parts (more than a cutter's
    /// diameter past the corner square) only a few times, not on every pass.
    #[test]
    fn adaptive_clears_an_l_arm_by_arm() {
        let op = roughing(&plate(&[[-25.0, -15.0, 25.0, -3.0], [13.0, -15.0, 25.0, 15.0]]), &job(""));
        for (z, above) in [(7.1, 10.0), (4.2, 7.1)] {
            let seen = replay_level(&op, z, above);
            // the corner square is x 13..25, y −15..−3: the far part of the x arm lies left of
            // x 7, of the y arm above y 3
            let arm = |p: [f64; 3]| {
                if p[0] < 7.0 {
                    Some(0)
                } else if p[1] > 3.0 {
                    Some(1)
                } else {
                    None
                }
            };
            let mut seq: Vec<u8> = Vec::new();
            for (e, p) in seen.engaged.iter().zip(&seen.at) {
                if let (true, Some(a)) = (e.0 > 10.0, arm(*p))
                    && seq.last() != Some(&a)
                {
                    seq.push(a);
                }
            }
            assert!(seq.contains(&0) && seq.contains(&1), "level {z}: both arms cut: {seq:?}");
            assert!(seq.len() <= 4, "level {z}: the tool goes between the arms {} times", seq.len() - 1);
        }
    }

    /// Links take the quickest way for the machine's rapid speed: with fast rapids the tool goes
    /// up to clearance (15 here: the sheet top 10 plus 5) more often than with slow ones, which
    /// keep it down in cleared ground; and each job is quickest at its own rapid speed.
    #[test]
    fn links_take_the_quickest_way_for_the_rapid_speed() {
        let b = plate(&[[-20.0, -12.0, 20.0, 12.0]]);
        let fast = roughing(&b, &rapid_job(1e6));
        let slow = roughing(&b, &rapid_job(600.0));
        assert!(
            lifts_to_clearance(&fast) > lifts_to_clearance(&slow),
            "fast rapids lift {} times, slow {}",
            lifts_to_clearance(&fast),
            lifts_to_clearance(&slow)
        );
        assert!(minutes(&fast, 1e6) <= minutes(&slow, 1e6), "fast rapids");
        assert!(minutes(&slow, 600.0) <= minutes(&fast, 600.0), "slow rapids");
    }

    /// Signed distance from `p` to a rectangle of half-sizes `hx`, `hy` about `c` with corner
    /// radius `r` (negative inside).
    fn rounded_rect_dist(p: [f64; 2], c: [f64; 2], hx: f64, hy: f64, r: f64) -> f64 {
        let qx = (p[0] - c[0]).abs() - (hx - r);
        let qy = (p[1] - c[1]).abs() - (hy - r);
        qx.max(0.0).hypot(qy.max(0.0)) + qx.max(qy).min(0.0) - r
    }

    /// Points every `step` round a rectangle of half-sizes `hx`, `hy` about `c` with corner
    /// radius `r`.
    fn rounded_rect_outline(c: [f64; 2], hx: f64, hy: f64, r: f64, step: f64) -> Vec<[f64; 2]> {
        let mut out = Vec::new();
        let (ix, iy) = (hx - r, hy - r);
        for (sx, sy) in [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)] {
            let a0: f64 = f64::atan2(sy, sx) - std::f64::consts::FRAC_PI_4;
            let n = ((r * std::f64::consts::FRAC_PI_2) / step).ceil().max(1.0) as usize;
            for k in 0..=n {
                let a = a0 + std::f64::consts::FRAC_PI_2 * k as f64 / n as f64;
                out.push([c[0] + sx * ix + r * a.cos(), c[1] + sy * iy + r * a.sin()]);
            }
        }
        // the straight sides
        for (a, b) in [([ix, hy], [-ix, hy]), ([-hx, iy], [-hx, -iy]), ([-ix, -hy], [ix, -hy]), ([hx, -iy], [hx, iy])] {
            let n = ((a[0] - b[0]).hypot(a[1] - b[1]) / step).ceil().max(1.0) as usize;
            for k in 0..=n {
                let t = k as f64 / n as f64;
                out.push([c[0] + a[0] + (b[0] - a[0]) * t, c[1] + a[1] + (b[1] - a[1]) * t]);
            }
        }
        out
    }

    /// A slot's true shape, from its description in features.json.
    enum Slot {
        /// Rectangles of half-sizes about `c` with corner radius `r`, joined (a cross's arms).
        Rects { c: [f64; 2], halves: Vec<[f64; 2]>, r: f64 },
        /// Every point within `hw` of the segment `a`–`b`.
        Straight { a: [f64; 2], b: [f64; 2], hw: f64 },
        /// Every point within `hw` of the arc of radius `rad` about `c` from `from` turning
        /// `sweep` counter-clockwise (radians).
        Curved { c: [f64; 2], rad: f64, from: f64, sweep: f64, hw: f64 },
        /// A polygon with sharp corners.
        Polygon(Vec<[f64; 2]>),
        /// A sharp-cornered rectangle less a round island of radius `ri`, both about `c`.
        Island { c: [f64; 2], hx: f64, hy: f64, ri: f64 },
    }

    fn seg_dist(q: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let t = (((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
        (a[0] + t * dx - q[0]).hypot(a[1] + t * dy - q[1])
    }

    /// Points every `step` along `a`–`b`, both ends included.
    fn seg_points(a: [f64; 2], b: [f64; 2], step: f64) -> Vec<[f64; 2]> {
        let n = ((a[0] - b[0]).hypot(a[1] - b[1]) / step).ceil().max(1.0) as usize;
        (0..=n).map(|k| k as f64 / n as f64).map(|t| [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]).collect()
    }

    /// Points every `step` along the arc of radius `r` about `c` from angle `a0` turning `sweep`.
    fn arc_points(c: [f64; 2], r: f64, a0: f64, sweep: f64, step: f64) -> Vec<[f64; 2]> {
        let n = ((r * sweep.abs()) / step).ceil().max(1.0) as usize;
        (0..=n).map(|k| a0 + sweep * k as f64 / n as f64).map(|a| [c[0] + r * a.cos(), c[1] + r * a.sin()]).collect()
    }

    impl Slot {
        fn from_feature(feat: &serde_json::Value) -> Slot {
            let n = |v: &serde_json::Value| v.as_f64().unwrap();
            let pt = |v: &serde_json::Value| [n(&v[0]), n(&v[1])];
            match feat["shape"].as_str().unwrap() {
                "angled_slot" => Slot::Straight { a: pt(&feat["centres"][0]), b: pt(&feat["centres"][1]), hw: n(&feat["width"]) / 2.0 },
                "arc_slot" => Slot::Curved {
                    c: pt(&feat["arc_center"]),
                    rad: n(&feat["arc_radius"]),
                    from: n(&feat["from_deg"]).to_radians(),
                    sweep: n(&feat["sweep_deg"]).to_radians(),
                    hw: n(&feat["width"]) / 2.0,
                },
                "polygon" => Slot::Polygon(feat["points"].as_array().unwrap().iter().map(pt).collect()),
                "rect_round_island" => Slot::Island {
                    c: pt(&feat["center"]),
                    hx: n(&feat["size"][0]) / 2.0,
                    hy: n(&feat["size"][1]) / 2.0,
                    ri: n(&feat["island_diameter"]) / 2.0,
                },
                _ => Slot::Rects {
                    c: pt(&feat["center"]),
                    halves: match feat.get("arms") {
                        Some(arms) => arms.as_array().unwrap().iter().map(|a| [n(&a[0]) / 2.0, n(&a[1]) / 2.0]).collect(),
                        None => vec![[n(&feat["size"][0]) / 2.0, n(&feat["size"][1]) / 2.0]],
                    },
                    r: n(&feat["corner_radius"]),
                },
            }
        }

        /// Signed distance from `q` to the slot's wall, negative inside: exact inside every
        /// shape but a cross (where it is the deeper of the two arms, so never more).
        fn sdf(&self, q: [f64; 2]) -> f64 {
            match self {
                Slot::Rects { c, halves, r } => halves.iter().map(|h| rounded_rect_dist(q, *c, h[0], h[1], *r)).fold(f64::INFINITY, f64::min),
                Slot::Straight { a, b, hw } => seg_dist(q, *a, *b) - hw,
                Slot::Curved { c, rad, from, sweep, hw } => {
                    let along = ((q[1] - c[1]).atan2(q[0] - c[0]) - from).rem_euclid(std::f64::consts::TAU);
                    let d = if along <= *sweep {
                        ((q[0] - c[0]).hypot(q[1] - c[1]) - rad).abs()
                    } else {
                        let end = |a: f64| (c[0] + rad * a.cos() - q[0]).hypot(c[1] + rad * a.sin() - q[1]);
                        end(*from).min(end(from + sweep))
                    };
                    d - hw
                }
                Slot::Polygon(pts) => {
                    let mut inside = false;
                    let mut d = f64::INFINITY;
                    for (i, a) in pts.iter().enumerate() {
                        let b = pts[(i + 1) % pts.len()];
                        d = d.min(seg_dist(q, *a, b));
                        if (a[1] > q[1]) != (b[1] > q[1]) && q[0] < a[0] + (q[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) {
                            inside = !inside;
                        }
                    }
                    if inside { -d } else { d }
                }
                Slot::Island { c, hx, hy, ri } => rounded_rect_dist(q, *c, *hx, *hy, 0.0).max(ri - (q[0] - c[0]).hypot(q[1] - c[1])),
            }
        }

        /// Its walls, a point every `step` (a cross: each arm's outline less what lies inside
        /// the other arm).
        fn outline(&self, step: f64) -> Vec<[f64; 2]> {
            use std::f64::consts::PI;
            match self {
                Slot::Rects { c, halves, r } => halves
                    .iter()
                    .enumerate()
                    .flat_map(|(i, h)| {
                        let others: Vec<[f64; 2]> = halves.iter().enumerate().filter(|(j, _)| *j != i).map(|(_, o)| *o).collect();
                        rounded_rect_outline(*c, h[0], h[1], *r, step)
                            .into_iter()
                            .filter(move |q| others.iter().all(|o| rounded_rect_dist(*q, *c, o[0], o[1], *r) >= -1e-9))
                    })
                    .collect(),
                Slot::Straight { a, b, hw } => {
                    let dir = (b[1] - a[1]).atan2(b[0] - a[0]);
                    let (nx, ny) = (-dir.sin() * hw, dir.cos() * hw);
                    let mut out = seg_points([a[0] + nx, a[1] + ny], [b[0] + nx, b[1] + ny], step);
                    out.extend(seg_points([a[0] - nx, a[1] - ny], [b[0] - nx, b[1] - ny], step));
                    out.extend(arc_points(*b, *hw, dir - PI / 2.0, PI, step));
                    out.extend(arc_points(*a, *hw, dir + PI / 2.0, PI, step));
                    out
                }
                Slot::Curved { c, rad, from, sweep, hw } => {
                    let mut out = arc_points(*c, rad + hw, *from, *sweep, step);
                    out.extend(arc_points(*c, rad - hw, *from, *sweep, step));
                    let end = |a: f64| [c[0] + rad * a.cos(), c[1] + rad * a.sin()];
                    out.extend(arc_points(end(from + sweep), *hw, from + sweep, PI, step));
                    out.extend(arc_points(end(*from), *hw, from + PI, PI, step));
                    out
                }
                Slot::Polygon(pts) => pts.iter().enumerate().flat_map(|(i, a)| seg_points(*a, pts[(i + 1) % pts.len()], step)).collect(),
                Slot::Island { c, hx, hy, ri } => {
                    let mut out = rounded_rect_outline(*c, *hx, *hy, 0.0, step);
                    out.extend(arc_points(*c, *ri, 0.0, 2.0 * PI, step));
                    out
                }
            }
        }

        fn bounds(&self) -> ([f64; 2], [f64; 2]) {
            self.outline(0.05).iter().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(lo, hi), p| {
                ([lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])])
            })
        }
    }

    /// Adaptive on the slot test parts (tests/cam: slot-plate, stepped-slot, cross-slot,
    /// odd-slots) with a Ø6 cutter, 0.9 mm stepover, 3 mm stepdown: for each slot, replayed on a
    /// 0.05 mm grid round it at its last roughing level (0.2 above its floor; 0.5 through, the skin's top),
    /// engagement stays within the limits (`engagement_ok`), every point the cutter can reach
    /// (within 3 of a point 3.5 inside the wall: the radius and the 0.5 roughing leaves on walls)
    /// is cut, and the centre stays 3.5 from the wall; in a slot too narrow for that (marked
    /// `slotting`, and announced) 3.02. A cross's reach
    /// is its two arms'; a sharp corner's tip is out of reach.
    #[test]
    fn adaptive_clears_the_slot_test_parts() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/cam");
        let expected: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(dir.join("features.json")).unwrap()).unwrap();
        for part in expected["parts"].as_array().unwrap().iter().filter(|p| p["design"].as_str().unwrap().contains("slot")) {
            let name = part["design"].as_str().unwrap();
            let step = std::fs::read_to_string(dir.join(name.replace(".sc", ".step"))).unwrap();
            let body = solidcraft_kernel::step_import(&step).unwrap().bodies.remove(0).body;
            let top = part["top_z"].as_f64().unwrap();
            let j = Job::from_json(&format!(
                r#"{{"stock": {{"size": [300, 300], "thickness": {top}}},
                    "tools": [{{"number": 1, "kind": "flat", "diameter": 6, "flute_length": 22, "rpm": 18000, "feed": 1200, "plunge": 300, "stepdown": 3, "stepover": 0.15}}],
                    "parts": [{{"body": "P"}}]}}"#
            ))
            .unwrap();
            let plan = plan_part(&read_part(&body).unwrap(), &j, "P").unwrap();
            // only slots features.json says must be slotted may warn, and those must
            let slotted: Vec<&serde_json::Value> = part["features"].as_array().unwrap().iter().filter(|f| f["adaptive"] == "slotting").collect();
            let adaptive_notes = plan.warnings.iter().filter(|w| w.contains("adaptive:")).count();
            assert_eq!(adaptive_notes, slotted.len(), "{name}: {:?}", plan.warnings);
            let of: Vec<&Operation> = plan.ops.iter().filter(|o| o.kind == OpKind::Roughing).collect();
            let op = &Operation {
                kind: OpKind::Roughing,
                name: String::new(),
                tool: of[0].tool,
                rpm: of[0].rpm,
                moves: of.iter().flat_map(|o| o.moves.iter().cloned()).collect(),
            };
            let sampled = strokes(op);
            // the levels the roughing cuts at: heights of moves that stay level (a helix moves
            // sideways while it comes down)
            let mut at = [0.0f64; 3];
            let mut levels: Vec<f64> = Vec::new();
            for m in &op.moves {
                let to = m.end();
                if matches!(m, crate::toolpath::Move::Line { .. } | crate::toolpath::Move::Arc { .. }) && (to[2] - at[2]).abs() < 1e-12 && to[2] < top
                {
                    levels.push((to[2] * 1e6).round() / 1e6);
                }
                at = to;
            }
            levels.sort_by(|a, b| b.total_cmp(a));
            levels.dedup();
            for feat in part["features"].as_array().unwrap().iter().filter(|f| f["kind"] == "pocket" || f["kind"] == "cutout") {
                let c = [feat["center"][0].as_f64().unwrap(), feat["center"][1].as_f64().unwrap()];
                let slot = Slot::from_feature(feat);
                // a through slot is roughed down to the skin's top, 0.5 above the bottom (the
                // separation cuts the rest)
                let z = if feat["kind"] == "cutout" { 0.5 } else { feat["floor_z"].as_f64().unwrap() + 0.2 };
                let z = *levels.iter().find(|l| (**l - z).abs() < 1e-6).unwrap_or_else(|| panic!("{name}: no roughing level at {z}: {levels:?}"));
                let above = levels.iter().copied().filter(|l| *l > z + 1e-9).fold(top, f64::min);
                let (lo, hi) = slot.bounds();
                let grid = Replay::over([lo[0] - 4.0, lo[1] - 4.0], [hi[0] + 4.0, hi[1] + 4.0], 0.05);
                let seen = replay_level_on(&sampled, z, above, grid);
                assert!(!seen.engaged.is_empty(), "{name}: nothing cut at {z} in the slot at {c:?}");
                // how far inside the wall the centre stays: the radius and the 0.5 left on walls,
                // or, in a slot too narrow for that, the radius and 0.02
                let keep = if feat["adaptive"] == "slotting" { 3.02 } else { 3.5 };
                if feat["adaptive"] == "slotting" {
                    let at = format!("({:.1}, {:.1})", c[0], c[1]);
                    assert!(
                        plan.warnings.iter().any(|w| w.contains("slotting") && w.contains(&at)),
                        "{name}: the slot at {at} is slotted unannounced: {:?}",
                        plan.warnings
                    );
                } else {
                    // each sample's distance from the centres' region's edge, 3.5 inside the wall
                    let with: Vec<(f64, f64)> = seen.engaged.iter().zip(&seen.at).map(|(e, p)| (e.0, -slot.sdf([p[0], p[1]]) - keep)).collect();
                    engagement_ok(&format!("{name}, slot at {c:?}, level {z}"), &with);
                }
                // the gouge check uses the slot's true outline, sampled every 0.02 mm
                let outline = slot.outline(0.02);
                // outline points by 1 mm cell: a centre need only look at the cells within 4 mm
                let mut cells: std::collections::HashMap<(i64, i64), Vec<[f64; 2]>> = Default::default();
                for o in &outline {
                    cells.entry((o[0].floor() as i64, o[1].floor() as i64)).or_default().push(*o);
                }
                let depth = |q: [f64; 2]| {
                    let (ci, cj) = (q[0].floor() as i64, q[1].floor() as i64);
                    let mut d = 4.0f64;
                    for i in ci - 4..=ci + 4 {
                        for j in cj - 4..=cj + 4 {
                            for o in cells.get(&(i, j)).into_iter().flatten() {
                                d = d.min((o[0] - q[0]).hypot(o[1] - q[1]));
                            }
                        }
                    }
                    d
                };
                for st in &sampled {
                    for s in st.iter().filter(|s| (s.p[2] - z).abs() < 1e-9 && seen.replay.inside(s.p)).step_by(4) {
                        let d = depth([s.p[0], s.p[1]]);
                        assert!(d >= keep - 0.02, "{name}: the centre at {:?} is {d:.3} from the slot's wall", s.p);
                    }
                }
                // Where the cutter can reach: within 3 of a centre 3.5 inside the wall. A point that
                // deep is itself reached; any other is near the edge of the centres' region, so
                // only centres on that edge (3.5 to 3.6 deep, every 0.1) are stamped, each with
                // the points within 2.95 of it (0.05 of slack: the planner's region is built from
                // offsets drawn as chords within 0.01 and paths within 0.005, so its rounded
                // corners sit a little inside).
                let h = 0.1;
                let (gx, gy) = (((hi[0] - lo[0]) / h).ceil() as usize + 1, ((hi[1] - lo[1]) / h).ceil() as usize + 1);
                let mut reach = vec![false; gx * gy];
                let k = (2.95 / h).ceil() as i64;
                for i in 0..gx {
                    for j in 0..gy {
                        let p = [lo[0] + i as f64 * h, lo[1] + j as f64 * h];
                        let d = slot.sdf(p);
                        if !(-keep - 0.1..=-keep).contains(&d) {
                            continue;
                        }
                        for di in -k..=k {
                            for dj in -k..=k {
                                let (ii, jj) = (i as i64 + di, j as i64 + dj);
                                if ii >= 0
                                    && jj >= 0
                                    && (ii as usize) < gx
                                    && (jj as usize) < gy
                                    && ((di * di + dj * dj) as f64) * h * h <= 2.95 * 2.95
                                {
                                    reach[ii as usize * gy + jj as usize] = true;
                                }
                            }
                        }
                    }
                }
                let mut checked = 0;
                for i in 0..gx {
                    for j in 0..gy {
                        let (x, y) = (lo[0] + i as f64 * h, lo[1] + j as f64 * h);
                        if (i % 4 != 0 || j % 4 != 0) || !(slot.sdf([x, y]) <= -keep || reach[i * gy + j]) {
                            continue;
                        }
                        checked += 1;
                        assert!(seen.replay.is_cut(x, y), "{name}: ({x:.2}, {y:.2}) in the slot at {c:?} is left uncut at {z}");
                    }
                }
                assert!(checked > 100, "{name}: only {checked} points checked in the slot at {c:?}");
            }
        }
    }

    /// Two pockets, each two levels deep. By depth: every level's z never rises again (both
    /// pockets at 7.1, then both at 4.2). By area: one pocket all the way down, then the other.
    #[test]
    fn levels_go_by_depth_or_by_area() {
        let b = plate(&[[-27.0, -10.0, -5.0, 10.0], [5.0, -10.0, 27.0, 10.0]]);
        for pattern in ["adaptive", "offset"] {
            let seq = |order: &str| {
                let op = roughing(&b, &job(&format!(r#""order": "{order}", "pattern": "{pattern}""#)));
                let mut seq: Vec<(i8, f64)> = Vec::new();
                for st in strokes(&op) {
                    // horizontal moves only: a descent passes through 7.1 on its way to 4.2
                    for s in st.iter().filter(|s| s.dir != [0.0, 0.0] && ((s.p[2] - 7.1).abs() < 1e-6 || (s.p[2] - 4.2).abs() < 1e-6)) {
                        let key = (if s.p[0] < 0.0 { -1 } else { 1 }, (s.p[2] * 1e3).round() / 1e3);
                        if seq.last() != Some(&key) {
                            seq.push(key);
                        }
                    }
                }
                seq
            };
            let depth = seq("depth");
            assert!(depth.windows(2).all(|w| w[1].1 <= w[0].1), "{pattern} by depth: {depth:?}");
            assert_eq!(depth.iter().filter(|k| k.1 == 7.1).count(), 2, "{pattern}: both pockets at 7.1 first: {depth:?}");
            let area = seq("area");
            assert_eq!(area.len(), 4, "{pattern} by area: each pocket's two levels, once: {area:?}");
            assert!(area[0].0 == area[1].0 && area[2].0 == area[3].0 && area[0].0 != area[2].0, "{pattern} by area: {area:?}");
            assert!(area[0].1 > area[1].1 && area[2].1 > area[3].1, "{pattern} by area, top level first: {area:?}");
        }
    }
}

#[cfg(test)]
mod scene_tests {
    use solidcraft_geom::Vec3;

    /// A binary STL: 80-byte header, the count, 50 bytes a triangle; a 1 × 2 × 3 box as written
    /// for the stock: 12 triangles facing out (their signed volumes add up to +6).
    #[test]
    fn stl_files_are_binary_and_boxes_face_out() {
        let tris: Vec<[Vec3; 3]> = vec![[Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)]];
        let b = crate::scene::stl("t", &tris);
        assert_eq!(b.len(), 84 + 50);
        assert_eq!(u32::from_le_bytes([b[80], b[81], b[82], b[83]]), 1);
        let f = |o: usize| f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        assert_eq!((f(84), f(88), f(92)), (0.0, 0.0, 1.0), "the normal");
        assert_eq!((f(108), f(112), f(116)), (1.0, 0.0, 0.0), "the second corner");
        let stock = crate::scene::stock_tris([0.0, 0.0, -3.0], [1.0, 2.0, 0.0]);
        assert_eq!(stock.len(), 12);
        let vol: f64 = stock.iter().map(|t| t[0].dot(t[1].cross(t[2])) / 6.0).sum();
        assert!((vol - 6.0).abs() < 1e-12, "{vol}");
    }

    /// The bounds of a binary STL's corners.
    fn stl_bounds(b: &[u8]) -> (usize, [f64; 3], [f64; 3]) {
        let n = u32::from_le_bytes([b[80], b[81], b[82], b[83]]) as usize;
        assert_eq!(b.len(), 84 + 50 * n);
        let f = |o: usize| f64::from(f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]));
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for t in 0..n {
            for k in 0..3 {
                let o = 84 + 50 * t + 12 + 12 * k;
                for i in 0..3 {
                    lo[i] = lo[i].min(f(o + 4 * i));
                    hi[i] = hi[i].max(f(o + 4 * i));
                }
            }
        }
        (n, lo, hi)
    }

    /// Two 40 × 30 × 6 plates on a 120 × 60 × 6 sheet, work zero in its top centre, packed in a
    /// zip: scene.json, stock.stl, one ready-placed model per copy and placed.stl. Each copy's
    /// model lies where scene.json says (its box), 40 × 30 or 30 × 40 across, from the sheet's
    /// bottom (−6) to its top (0), inside the sheet (±60, ±30); placed.stl holds both.
    #[test]
    fn the_scene_zip_holds_ready_placed_models() {
        use std::io::Read as _;
        let b = solidcraft_kernel::box_solid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 6.0)).unwrap();
        let job = crate::job::Job::from_json(
            r#"{"stock": {"size": [120, 60], "thickness": 6}, "strategy": {"roughing": {"pattern": "offset"}}, "parts": [{"body": "P", "name": "Plate", "quantity": 2}]}"#,
        )
        .unwrap();
        let plan = crate::plan::plan_job(&job, std::slice::from_ref(&b)).unwrap();
        let scene = crate::scene::export(&job, &[b], &plan).unwrap();
        let bytes = crate::scene::zip(&scene).unwrap();
        let mut z = ::zip::ZipArchive::new(std::io::Cursor::new(bytes)).unwrap();
        let mut names: Vec<String> = z.file_names().map(|n| n.unwrap().to_string()).collect();
        names.sort();
        assert_eq!(names, ["job.ngc", "parts/1-plate-1.stl", "parts/1-plate-2.stl", "placed.stl", "scene.json", "stock.stl"]);
        let mut read = |name: &str| {
            let mut v = Vec::new();
            z.by_name(name).unwrap().read_to_end(&mut v).unwrap();
            v
        };
        let doc: serde_json::Value = serde_json::from_slice(&read("scene.json")).unwrap();
        let (_, slo, shi) = stl_bounds(&read("stock.stl"));
        assert_eq!((slo, shi), ([-60.0, -30.0, -6.0], [60.0, 30.0, 0.0]));
        let mut tris = 0;
        for c in doc["parts"][0]["copies"].as_array().unwrap() {
            let (n, lo, hi) = stl_bounds(&read(c["model"].as_str().unwrap()));
            tris += n;
            let v3 = |v: &serde_json::Value| [0, 1, 2].map(|i| v[i].as_f64().unwrap());
            let (clo, chi) = (v3(&c["min"]), v3(&c["max"]));
            for i in 0..3 {
                assert!((lo[i] - clo[i]).abs() < 1e-4 && (hi[i] - chi[i]).abs() < 1e-4, "{lo:?} {hi:?} vs {clo:?} {chi:?}");
            }
            let (w, d) = (hi[0] - lo[0], hi[1] - lo[1]);
            assert!(((w - 40.0).abs() < 1e-4 && (d - 30.0).abs() < 1e-4) || ((w - 30.0).abs() < 1e-4 && (d - 40.0).abs() < 1e-4), "{w} × {d}");
            assert!((lo[2] + 6.0).abs() < 1e-4 && hi[2].abs() < 1e-4);
            assert!(lo[0] >= -60.0 && hi[0] <= 60.0 && lo[1] >= -30.0 && hi[1] <= 30.0);
        }
        assert_eq!(stl_bounds(&read("placed.stl")).0, tris, "every copy");
    }
}

#[cfg(test)]
mod sim_tests {
    use solidcraft_geom::Vec3;
    use solidcraft_kernel::{BoolOp, boolean, box_solid};

    use crate::sim::{Cutter, HeightMap, simulate, sweep};

    /// A Ø6 flat end mill 1 deep along 20 mm of a 40 × 20 block (0.05 mm cells) takes 20 × 6 +
    /// π × 3² (the round ends) = 148.27 mm³; every cell is at 0 or −1; a cell 2.9 from the path
    /// is cut, 3.1 is not.
    #[test]
    fn a_flat_cut_removes_its_swept_volume() {
        let mut m = HeightMap::new([-10.0, -10.0], [30.0, 10.0], 0.0, 0.05).unwrap();
        let c = Cutter { r: 3.0, cone: 0.0 };
        sweep(&mut m, c, [0.0, 0.0, -1.0], [20.0, 0.0, -1.0]);
        let vol: f64 = m.z.iter().map(|z| -f64::from(*z)).sum::<f64>() * 0.05 * 0.05;
        let want = 20.0 * 6.0 + std::f64::consts::PI * 9.0;
        assert!((vol - want).abs() < 0.01 * want, "{vol} vs {want}");
        assert!(m.z.iter().all(|z| *z == 0.0 || *z == -1.0));
        let cell = |x: f64, y: f64| m.at(((x + 10.0) / 0.05) as usize, ((y + 10.0) / 0.05) as usize);
        assert_eq!((cell(10.0, 2.9), cell(10.0, 3.1)), (-1.0, 0.0));
    }

    /// A 118° Ø5 drill plunged to −5 leaves a cone: at d from its axis −5 + d / tan 59°.
    #[test]
    fn a_drill_leaves_its_point() {
        let mut m = HeightMap::new([-5.0, -5.0], [5.0, 5.0], 0.0, 0.02).unwrap();
        let k = 1.0 / 59f64.to_radians().tan();
        sweep(&mut m, Cutter { r: 2.5, cone: k }, [0.0, 0.0, 1.0], [0.0, 0.0, -5.0]);
        for d in [0.0, 0.5, 1.2, 2.0, 2.4] {
            let i = ((d + 5.0) / 0.02) as usize;
            let j = (5.0 / 0.02) as usize;
            let c = m.centre(i, j);
            let r = c[0].hypot(c[1]);
            assert!((m.at(i, j) - (-5.0 + r * k)).abs() < 1e-4, "at {r}: {}", m.at(i, j));
        }
        assert_eq!(m.at(0, 0), 0.0, "outside the drill");
    }

    /// The report of a planned job: a 60 × 40 × 10 plate with a 30 × 20 pocket 4 deep with
    /// sharp corners (finishing T2 Ø3 leaves 1.5 × (√2 − 1) in each), cut out of a 100 × 80
    /// sheet: no gouges, no rapid through material, material left only in the four corners
    /// (within a millimetre of them, 4 high: the pocket's depth), the plate cut free.
    #[test]
    fn a_simulated_pocket_plate_shows_only_its_sharp_corners() {
        let plate = box_solid(Vec3::new(-30.0, -20.0, 0.0), Vec3::new(30.0, 20.0, 10.0)).unwrap();
        let b = boolean(&plate, &box_solid(Vec3::new(-15.0, -10.0, 6.0), Vec3::new(15.0, 10.0, 11.0)).unwrap(), BoolOp::Cut).unwrap().unwrap();
        let job = crate::job::Job::from_json(
            r#"{"stock": {"size": [100, 80], "thickness": 10}, "nest": {"margin": 10}, "strategy": {"roughing": {"pattern": "offset"}},
                "tools": [{"number": 1, "kind": "flat", "diameter": 6, "flute_length": 22, "rpm": 18000, "feed": 1200, "plunge": 300, "stepdown": 3},
                          {"number": 2, "kind": "flat", "diameter": 3, "flute_length": 12, "rpm": 22000, "feed": 700, "plunge": 150, "stepdown": 1.5}],
                "parts": [{"body": "P", "orient": "+z"}]}"#,
        )
        .unwrap();
        let plan = crate::plan::plan_job(&job, std::slice::from_ref(&b)).unwrap();
        let (r, _) = simulate(&job, &[b], &plan, 0.1).unwrap();
        assert!(r.gouges.is_empty(), "{:?}", r.gouges);
        assert!(r.rapid_hits.is_empty(), "{:?}", r.rapid_hits);
        assert!(r.spoilboard.is_empty());
        assert!(r.parts.len() == 1 && r.parts[0].free, "{:?}", r.parts);
        assert_eq!(r.left.len(), 4, "{:?}", r.left);
        // the pocket's corners where the plate sits: turned by the nesting, its centre at the
        // placement's offset
        let pl = &plan.nesting.placements[0];
        let turn = |p: (f64, f64)| if pl.quarter_turns % 2 == 1 { (-p.1, p.0) } else { p };
        for s in &r.left {
            let near = [(-15.0, -10.0), (15.0, -10.0), (-15.0, 10.0), (15.0, 10.0)]
                .map(turn)
                .iter()
                .any(|(x, y)| (s.at[0] - (pl.offset[0] + x)).abs() < 1.0 && (s.at[1] - (pl.offset[1] + y)).abs() < 1.0);
            assert!(near, "material left at {:?}, not in a corner: {:?}", s.at, r.left);
            assert!((s.by - 4.0).abs() < 0.05, "{} high", s.by);
        }
    }

    /// A 60 × 40 × 6 plate with a 20 × 14 cutout through it, `skin` cut-out with a Ø6: the
    /// separation runs 3 inside the cutout's walls, so the skin's middle, (20 − 12) × (14 − 12)
    /// = 16 mm² at 0.5 high, drops out as one loose slug; it is not material left on the plate.
    /// The cutout's four sharp corners are: the Ø6 cannot reach into them.
    #[test]
    fn the_skin_in_a_cutout_drops_out_as_a_slug() {
        let plate = box_solid(Vec3::new(-30.0, -20.0, 0.0), Vec3::new(30.0, 20.0, 6.0)).unwrap();
        let b = boolean(&plate, &box_solid(Vec3::new(-10.0, -7.0, -1.0), Vec3::new(10.0, 7.0, 7.0)).unwrap(), BoolOp::Cut).unwrap().unwrap();
        let job = crate::job::Job::from_json(
            r#"{"stock": {"size": [100, 80], "thickness": 6}, "nest": {"margin": 10}, "strategy": {"roughing": {"pattern": "offset"}},
                "tools": [{"number": 1, "kind": "flat", "diameter": 6, "flute_length": 22, "rpm": 18000, "feed": 1200, "plunge": 300, "stepdown": 3}],
                "parts": [{"body": "P", "orient": "+z"}]}"#,
        )
        .unwrap();
        let plan = crate::plan::plan_job(&job, std::slice::from_ref(&b)).unwrap();
        let (r, _) = simulate(&job, &[b], &plan, 0.1).unwrap();
        assert!(r.gouges.is_empty() && r.rapid_hits.is_empty(), "{:?} {:?}", r.gouges, r.rapid_hits);
        // the cutout's corners are sharp: the Ø6 leaves them, the full 6 high
        let pl = &plan.nesting.placements[0];
        let turn = |p: (f64, f64)| if pl.quarter_turns % 2 == 1 { (-p.1, p.0) } else { p };
        assert_eq!(r.left.len(), 4, "{:?}", r.left);
        for s in &r.left {
            let near = [(-10.0, -7.0), (10.0, -7.0), (-10.0, 7.0), (10.0, 7.0)]
                .map(turn)
                .iter()
                // all of a corner's leftover stands 6 high: the spot may be named by any of its
                // cells, which reach r(√2 − 1) + r(1 − 1/√2) ≈ 2.1 from the corner
                .any(|(x, y)| (s.at[0] - (pl.offset[0] + x)).abs() < 2.5 && (s.at[1] - (pl.offset[1] + y)).abs() < 2.5);
            assert!(near && (s.by - 6.0).abs() < 0.05, "{s:?}");
        }
        assert_eq!(r.slugs.len(), 1, "{:?}", r.slugs);
        assert!((r.slugs[0].area - 16.0).abs() < 2.5 && (r.slugs[0].by - 0.5).abs() < 1e-3, "{:?}", r.slugs[0]);
        assert!(r.parts[0].free);
    }

    /// A rapid through material is caught, with how deep it went.
    #[test]
    fn a_rapid_through_material_is_caught() {
        let b = box_solid(Vec3::new(-10.0, -10.0, 0.0), Vec3::new(10.0, 10.0, 10.0)).unwrap();
        let job = crate::job::Job::from_json(
            r#"{"stock": {"size": [60, 60], "thickness": 10}, "tools": [{"number": 1, "kind": "flat", "diameter": 6, "flute_length": 22, "rpm": 18000, "feed": 1200, "plunge": 300}], "parts": [{"body": "P"}]}"#,
        )
        .unwrap();
        let op = crate::toolpath::Operation {
            kind: crate::toolpath::OpKind::Roughing,
            name: "bad".into(),
            tool: 1,
            rpm: 18000.0,
            moves: vec![
                crate::toolpath::Move::Rapid { to: [-20.0, 0.0, 5.0] },
                crate::toolpath::Move::Rapid { to: [-20.0, 0.0, -2.0] },
                crate::toolpath::Move::Rapid { to: [20.0, 0.0, -2.0] },
            ],
        };
        let _ = b;
        let mut m = HeightMap::new([-30.0, -30.0], [30.0, 30.0], 0.0, 0.2).unwrap();
        let hits = crate::sim::run(&mut m, &job, &[op]).unwrap();
        assert!(!hits.is_empty() && hits.iter().all(|h| (h.depth - 2.0).abs() < 1e-6), "{hits:?}");
    }
}
