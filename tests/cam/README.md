# CAM test parts

Aluminium parts machined from the top only on a 3-axis router. `features.json` lists what feature
recognition must find in each one, with its exact sizes, levels and volume; the PNGs show the parts.

| Part | Stock | Features |
|---|---|---|
| `nema23-flange` | 90 × 90 × 10 mm | R6 corners; 1 mm top chamfer (also round the corners: cones); Ø38.2 × 2.5 pilot recess with 0.5 mm chamfer; Ø24 through bore from the recess floor; 4 × M5 counterbores (Ø5.5 / Ø10 × 5) on the NEMA 23 47.14 mm square; 4 × M5 90° countersinks (Ø5.5 / Ø10.4) |
| `front-panel` | 200 × 100 × 3 mm | R4 corners; 0.5 mm top chamfer; 60 × 30 R2 display cutout with 0.5 mm chamfer; 70 × 14 R3 label recess 0.8 mm deep; Ø22.5, 2 × Ø12, 3 × Ø5 through holes; 4 × M3 90° countersinks (Ø3.4 / Ø6.6) |

Both are built from planes, cylinders and cones only. The `.step` files carry the same solids for
importing elsewhere.
| `slot-plate` | 140 × 100 × 10 mm | three 70 mm slots side by side: 10 wide 4 deep (R4), 16 wide 6 deep (R3), 24 wide through (R4) |
| `stepped-slot` | 100 × 60 × 12 mm | an 80 × 30 slot 4 deep (R4) with a 60 × 14 slot (R3) on down to z 3 in it; a 50 × 7 slot 3 deep (R3.4) only just wider than a Ø6 cutter |
| `cross-slot` | 100 × 100 × 8 mm | an 80 × 12 and a 12 × 60 slot crossing, 5 deep, R3 at the arms' ends, sharp where they cross |
| `odd-slots` | 160 × 120 × 10 mm | a 12 wide slot at 30° (centres 40 apart) 4 deep; a 10 wide kidney slot on a 120° arc of R25, 5 deep; an L-shaped dogleg of 12 wide arms, 5 deep, and a triangle 30 wide and 30 high, 6 deep, both with sharp corners; a 36 × 30 pocket round a Ø12 island, 5 deep |
| `relieved-corners` | 160 × 120 × 10 mm | `odd-slots` with its inner corners relieved for a Ø3 finishing cutter: the dogleg's five corners dogboned (R1.75 circles through each corner on its bisector), the triangle's corners rounded R3, the island pocket's R4 |

The slot parts test adaptive clearing: fronts meeting long walls, a stepped floor, a slot barely
wider than the cutter, fronts crossing at an intersection, slots at an angle and along an arc,
sharp corners the cutter can't reach into (`odd-slots`; `relieved-corners` is the same part made to be machined clean), and a pocket with an island. A pocket that isn't one rounded
rectangle gives its `floor_area` and `curved_length` in `features.json`.
