# CAM test parts

Aluminium parts machined from the top only on a 3-axis router. `features.json` lists what feature
recognition must find in each one, with its exact sizes, levels and volume; the PNGs show the parts.

| Part | Stock | Features |
|---|---|---|
| `nema23-flange` | 90 × 90 × 10 mm | R6 corners; 1 mm top chamfer (also round the corners: cones); Ø38.2 × 2.5 pilot recess with 0.5 mm chamfer; Ø24 through bore from the recess floor; 4 × M5 counterbores (Ø5.5 / Ø10 × 5) on the NEMA 23 47.14 mm square; 4 × M5 90° countersinks (Ø5.5 / Ø10.4) |
| `front-panel` | 200 × 100 × 3 mm | R4 corners; 0.5 mm top chamfer; 60 × 30 R2 display cutout with 0.5 mm chamfer; 70 × 14 R3 label recess 0.8 mm deep; Ø22.5, 2 × Ø12, 3 × Ø5 through holes; 4 × M3 90° countersinks (Ø3.4 / Ø6.6) |

Both are built from planes, cylinders and cones only. The `.step` files carry the same solids for
importing elsewhere.
