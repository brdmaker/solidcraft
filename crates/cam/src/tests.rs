//! CAM tests. Every expected value is worked out by hand or in closed form, never read back
//! from the code under test.

mod region_tests {
    use std::f64::consts::PI;

    use crate::region::{Area, GRID, circle_ring, ring_length, segments_for, signed_area};

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
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
    use crate::job::{Job, ToolKind};

    const MINIMAL: &str = r#"{"stock": {"size": [600, 400], "thickness": 10}, "parts": [{"body": "Flange"}]}"#;

    #[test]
    fn a_minimal_job_takes_the_defaults() {
        let j = Job::from_json(MINIMAL).unwrap();
        assert_eq!(j.version, 1);
        assert_eq!(j.stock.spoilboard_allowance, 0.2);
        assert_eq!((j.nest.margin, j.nest.spacing, j.nest.rotate), (10.0, 10.0, true));
        assert_eq!(j.parts[0].quantity, 1);
        assert_eq!(j.parts[0].label(), "Flange");
        assert!(j.tools.is_empty());
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
        Stock { size: [w, h], thickness: t, material: String::new(), spoilboard_allowance: 0.2 }
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

    fn op(ops: &[Operation], kind: OpKind) -> &Operation {
        ops.iter().find(|o| o.kind == kind).unwrap_or_else(|| panic!("no {kind:?} operation: {:?}", ops.iter().map(|o| o.kind).collect::<Vec<_>>()))
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

    /// Roughing stays 3 + 0.2 mm off the pocket's walls and 0.2 mm above its floor; feeds below
    /// the top only inside the pocket; rapids that move sideways only at the clearance height.
    #[test]
    fn roughing_keeps_the_stock_to_leave() {
        let f = read_part(&pocket_plate()).unwrap();
        let plan = plan_part(&f, &job(FLAT6, 10.0, ""), "P").unwrap();
        assert!(plan.warnings.is_empty(), "{:?}", plan.warnings);
        let rough = op(&plan.ops, OpKind::Roughing);
        let pts = sample(rough, [0.0, 0.0, 15.0]);
        assert!(pts.iter().all(|a| !a.drill), "roughing has no drilling cycles");
        let cutting: Vec<&At> = pts.iter().filter(|a| !a.rapid && a.p[2] < 10.0 - 1e-9).collect();
        assert!(!cutting.is_empty());
        for a in &cutting {
            let d = rect_dist(a.p, -15.0, -10.0, 15.0, 10.0);
            assert!(d <= -(3.2 - PATH_TOL - 1e-6), "roughing at {:?} is {} from the pocket wall", a.p, -d);
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
        let pts = sample(op(&plan.ops, OpKind::FloorFinish), [0.0, 0.0, 15.0]);
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

    /// The wall pass runs exactly 3 mm (the tool's radius) inside each wall, clockwise round the
    /// pocket (wall on the left: climb milling), and down to the floor.
    #[test]
    fn walls_are_finished_climbing_at_the_tool_radius() {
        let f = read_part(&pocket_plate()).unwrap();
        let plan = plan_part(&f, &job(FLAT6, 10.0, ""), "P").unwrap();
        let walls = op(&plan.ops, OpKind::WallFinish);
        let pts = sample(walls, [0.0, 0.0, 15.0]);
        let at6: Vec<[f64; 2]> = pts.iter().filter(|a| !a.rapid && (a.p[2] - 6.0).abs() < 1e-9).map(|a| [a.p[0], a.p[1]]).collect();
        assert!(!at6.is_empty(), "the walls are finished down to the floor");
        for p in &at6 {
            let d = rect_dist([p[0], p[1], 0.0], -15.0, -10.0, 15.0, 10.0);
            assert!((d + 3.0).abs() <= PATH_TOL + 1e-6, "wall pass at {p:?} is {} from the wall", -d);
        }
        assert!(signed_area(&at6) < 0.0, "clockwise round the pocket");
        for side in [[-12.0, 0.0], [12.0, 0.0], [0.0, -7.0], [0.0, 7.0]] {
            assert!(
                at6.iter().any(|p| (p[0] - side[0]).abs() < 0.3 && (p[1] - side[1]).abs() < 0.3
                    || (side[0] == 0.0 && (p[1] - side[1]).abs() < 1e-6)
                    || (side[1] == 0.0 && (p[0] - side[0]).abs() < 1e-6)),
                "no pass along the wall at {side:?}"
            );
        }
    }

    /// The profile runs 3 mm outside the outline, anticlockwise (part on the left), down to the
    /// spoilboard allowance (−0.2), except over four tabs 1 mm high and 6 + 6 mm long.
    #[test]
    fn the_profile_cuts_through_round_four_tabs() {
        let f = read_part(&pocket_plate()).unwrap();
        let plan = plan_part(&f, &job(FLAT6, 10.0, ""), "P").unwrap();
        let prof = op(&plan.ops, OpKind::Profile);
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
        let d = op(&plan.ops, OpKind::Drilling);
        assert_eq!(d.tool, 5);
        let tip = 5.0 / 59f64.to_radians().tan();
        match d.moves.as_slice() {
            [Move::Rapid { .. }, Move::Drill { at, r, bottom, peck, .. }] => {
                assert!((at[0] - 20.0).abs() < 1e-6 && (at[1] - 20.0).abs() < 1e-6);
                assert!((r - 9.0).abs() < 1e-9 && (bottom - (-0.2 - tip)).abs() < 1e-9 && *peck == Some(2.0), "{:?}", d.moves);
            }
            m => panic!("a rapid over the hole and one drilling cycle, not {m:?}"),
        }
        assert!(!plan.ops.iter().any(|o| o.kind == OpKind::HoleMilling));

        let plan = plan_part(&f, &job(FLAT6, 8.0, ""), "P").unwrap();
        let pts = sample(op(&plan.ops, OpKind::HoleMilling), [20.0, 20.0, 13.0]);
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
        let holes = op(ops, OpKind::HoleMilling);
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
    use crate::toolpath::{OpKind, Operation, StrokeKind, trace};

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
        plan.ops.into_iter().find(|o| o.kind == OpKind::Roughing).unwrap()
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
            let h = 0.04;
            let (x0, y0) = (-31.0, -21.0);
            let (nx, ny) = ((62.0 / h) as usize, (42.0 / h) as usize);
            Replay { h, x0, y0, nx, ny, cut: vec![false; nx * ny] }
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

    /// What the replay saw on the level at `z`: each sample's engagement, and the grid after.
    struct Seen {
        engaged: Vec<(f64, usize, usize)>,
        replay: Replay,
    }

    /// Replay the level at `z`; `above` is the level over it (the stock's top for the first).
    fn replay_level(op: &Operation, z: f64, above: f64) -> Seen {
        let mut r = Replay::new();
        let mut engaged = Vec::new();
        for st in strokes(op) {
            // positions are marked cut once the tool is 1 mm further along: nearer ones would
            // cover the forward ring (anything within 0.2 r behind does) and hide the cut
            let mut trail: std::collections::VecDeque<([f64; 3], f64)> = Default::default();
            let mut along = 0.0;
            let mut prev: Option<[f64; 3]> = None;
            // entering the level (a helix or ramp coming down onto it, less than a stepdown
            // above): cut as it goes, not measured (an entry plunges, it doesn't cut sideways)
            for s in st.iter().filter(|s| s.p[2] > z + 1e-9 && s.p[2] < above - 1e-9) {
                r.stamp(s.p, 3.0);
            }
            for s in st.iter().filter(|s| (s.p[2] - z).abs() < 1e-9) {
                if let Some(q) = prev {
                    along += (s.p[0] - q[0]).hypot(s.p[1] - q[1]);
                }
                prev = Some(s.p);
                while trail.front().is_some_and(|(_, a)| along - a > 1.0) {
                    let (p, _) = trail.pop_front().unwrap();
                    r.stamp(p, 3.0);
                }
                engaged.push(r.engagement(s.p, s.dir, 3.0));
                trail.push_back((s.p, along));
            }
            for (p, _) in trail {
                r.stamp(p, 3.0);
            }
        }
        Seen { engaged, replay: r }
    }

    /// The pocket's region the cutter's centre may reach at a level: the 40 × 24 pocket shrunk
    /// by the radius (3) and the stock to leave (0.2): |x| ≤ 16.8, |y| ≤ 8.8.
    fn in_reach(p: [f64; 3]) -> bool {
        p[0].abs() <= 16.8 + 0.01 && p[1].abs() <= 8.8 + 0.01
    }

    /// Engagement stays within the limit (default here 90°: 1.3 × the straight cut's 45.6° is
    /// less) at both levels, to within the replay's resolution (rays every 1.9°, a 0.04 mm grid);
    /// no move takes the centre outside its region; and the last level is cut everywhere the
    /// cutter can reach.
    #[test]
    fn adaptive_keeps_engagement_and_clears_the_pocket() {
        let op = roughing(&plate(&[[-20.0, -12.0, 20.0, 12.0]]), &job(""));
        for (z, above) in [(7.1, 10.0), (4.2, 7.1)] {
            let seen = replay_level(&op, z, above);
            let mut e: Vec<f64> = seen.engaged.iter().map(|e| e.0).collect();
            assert!(e.len() > 1000, "level {z} has {} samples", e.len());
            e.sort_by(f64::total_cmp);
            let p99 = e[(e.len() - 1) * 99 / 100];
            let max = *e.last().unwrap();
            assert!(p99 <= 90.0 && max <= 90.0 + 6.0, "level {z}: p99 {p99:.1}°, max {max:.1}°");
            let median = e[e.len() / 2];
            assert!(median > 20.0, "level {z}: the cutter mostly works ({median:.1}° median)");
            if z == 4.2 {
                let mut x: f64 = -19.75;
                while x < 19.75 {
                    let mut y: f64 = -11.75;
                    while y < 11.75 {
                        let corner = x.abs() > 16.8 && y.abs() > 8.8;
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

    /// One way, every engaged sample (over 10°) has the uncut material on its left: climbing.
    /// Both ways, a good share cut with it on the right (conventional), and the job lifts less.
    #[test]
    fn one_way_climbs_and_both_ways_lifts_less() {
        let lifts = |op: &Operation| op.moves.windows(2).filter(|w| w[1].end()[2] > w[0].end()[2] + 1e-9).count();
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
        assert!(lifts(&both) < lifts(&one), "both ways lifts {} times, one way {}", lifts(&both), lifts(&one));
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
