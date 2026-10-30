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
