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
