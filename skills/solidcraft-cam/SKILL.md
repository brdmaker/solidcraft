---
name: solidcraft-cam
description: "Use when working on CAM in SolidCraft: crates/cam (machining features, toolpaths, G-code, cutting simulation), the cam.* commands, the tests/cam parts, or the quick CAM gate `cargo xtask cam`."
---

# CAM for 3-axis routers

## Keep CAM separate
- `crates/cam` (layer 2: `geom`, `kernel`, `i_overlay`) holds everything computed from a `solidcraft_kernel::Body`: feature recognition (`part.rs`), 2D areas (`region.rs`), toolpaths, the post-processor and the cutting simulation. It never depends on `doc`, `io` or `engine`, so its tests build in minutes.
- `crates/engine/src/cmd/cam.rs` holds thin `cam.*` commands: find the bodies, call the crate, return its result. Contracts go in `crates/engine/src/schema/cam.rs`, labels in `assets/locales/pending/`.
- No CAM logic in the engine, the UI or the kernel. If the crate needs something from the kernel, add a small public function there.

## Exact geometry
Toolpaths come from exact faces (`face_surface`, `planar_edges`). The mesh only gives topology, face extents and sides, and the triangulated copy for the check simulation. Holes keep their exact diameters. Areas are drawn with `AREA_TOL` chords, and tests bound that error (2/3 · tol · curved length) instead of loosening tolerances.

## The shop it serves
- A job listing no tools uses `crates/cam/library/router-aluminium.json`; its `library` and `tools` override it.
- Roughing leaves 0.5 on walls, 0.2 on floors; wall passes leave 0.3, 0.1, then none.
- Vacuum hold (`skin` cut-out): a band round the parts is roughed to 0.5 above the bottom; the last wall pass and 0.1 skin passes run last. A sheet runs kind by kind (end-milled countersinks first, through holes then the skin last), each level by level over every part.

## Tests: every feature has them
- Unit tests go in `crates/cam/src/tests.rs`, on solids built with the kernel (`box_solid`, `cylinder`, `boolean`), with expected values worked out by hand.
- Fixture tests go in `crates/cam/src/fixture_tests.rs`. They read `tests/cam/*.step` and compare with `tests/cam/features.json`.
- A new test part needs its `.sc`, `.step`, `.png`, a row in `attribution/pending/`, and `features.json` entries written before the code. Never edit `features.json` to match the output.
- `crates/engine/src/cmd/cam_tests.rs` checks only the command plumbing.

## The gate
- A commit that changes only CAM files (`crates/cam`, `tests/cam`, CAM docs and skills) runs `cargo xtask cam`: fmt, assets, licences, lists, skills and layers, then clippy, tests and the wasm check of `crates/cam`.
- Add `--engine` when `crates/engine/src/cmd/cam*` or `crates/engine/src/schema/cam.rs` changed. Run `cargo xtask i18n` when you add labels.
- The full `cargo xtask ci` takes hours on a small machine. For CAM-only commits it runs in GitHub CI, not locally. A commit that changes other crates still runs the full gate.

## Show the toolpaths
When work changes toolpaths, their order or the preview, look at the result and show it to the person you are working with. Render a preview with `cargo run -p solidcraft-cam --example preview -- JOB.json OUT.png` (parts as STEP files, relative to the job), read the PNG yourself, then attach it to your reply. Say what each colour is (`legend()` in `crates/cam/src/preview.rs`) and anything that looks wrong. A picture does not replace the tests that measure the paths.

## Never crash
The usual rules apply (`skills/solidcraft-crash-safety/SKILL.md`). Call `i_overlay` only through its fixed-scale functions, which return `Result`; its plain `outline` can panic.
