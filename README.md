# 3D-printable CNC, 300 × 300 × 100 mm workspace (SolidCraft design)

This orphan branch holds a SolidCraft design only; it shares no history with `main`.

- `cnc-300x300x100.sc`: the design (open with SolidCraft, or `solidcraft-cli mcp --in cnc-300x300x100.sc`).
- `cnc-iso.png`, `cnc-workspace-tour.gif`: render and the "Workspace tour" motion study.
- `dowel-joints.png`: the dowelled joint faces of a rail and of the gantry beam.
- `print-stl/`: the 28 printed parts, one STL per part, in their assembled position (orient them in the slicer).
- `tools/`: the generator for the build batches (`gen.py`) and the small MCP clients used to build it.

## Machine
- Everything is printed except 4 × NEMA 23 motors (57 × 57 × 76 mm, Ø6.35 shaft) and 3 × lever micro switches
  (20 × 10.2 × 6.4 mm), which are `purchased` in the BOM.
- X and Y: printed rack and pinion, module 2, 16-tooth involute pinions (pitch radius 16 mm, 0.15 mm backlash);
  two Y motors. Z: printed M16×2 lead screw in a threaded nut in the Z carriage.
- Guides: printed lip-in-groove slides, 0.5–1 mm clearance.
- Tool holder: Ø52 mm clamp ring.

## Joints
Drive the motor joints; the axes follow through motion links.

| Joint | Range | Moves |
|---|---|---|
| `Y pinion R` (`Y pinion L` mirrors it) | ±537.15° | `Y axis` ±150 mm (0.2793 mm/°) |
| `X pinion` | ±537.15° | `X axis` ∓150 mm |
| `Z screw` | −18000…0° (50 turns) | `Z axis` 0…100 mm (2 mm/turn, right-hand) |

Limit switches trip at Y +149, X +148 and Z +99 mm. A collision sweep of the workspace tour finds no contact
except those switch levers.

## Printing on a Bambu Lab H2D
Every printed part fits 300 × 320 × 325 mm, the H2D's two-nozzle volume (single nozzle: 325 × 320 × 325 mm).
The longest parts are the front Y racks (299.1 mm; they fit more easily laid diagonally) and the rail halves (273 mm).

| Joint | Pieces | Dowels |
|---|---|---|
| Y rails, split at y = −23 | 2 per side | 2 per joint, at x = ±225, z = 10 and 30 |
| Crossmembers, split at x = 0 | 2 each | 2 per joint, at z = 9, 10 mm either side of the centre |
| Gantry beam, split at x = 0 | 2 | 3, at (y −10, z 170), (y 10, z 158), (y 10, z 182) |
| Y racks, split at a tooth gap (y = 3.14) | 2 per side | none: glued to the rail, 26 mm from the rail joint |
| X rack, split at a tooth gap (x = 59.69) | 2 | none: glued to the beam, 60 mm from the beam joint |

Dowel holes are Ø8.2 × 10.5 mm on each side of the cut, for 11 × ISO 8734 Ø8 × 20 mm parallel pins (in the design
as purchased parts), glued in. The racks' bases (12 mm and 5 mm under the teeth) are too thin for an 8 mm pin, so
their joints are staggered from the joints under them instead.

## Purchased parts
| Part | Qty |
|---|---|
| NEMA 23 stepper, 57 × 57 × 76 mm, Ø6.35 shaft | 4 |
| Subminiature lever micro switch, 20 × 10.2 × 6.4 mm | 3 |
| Parallel pin ISO 8734, Ø8 × 20 mm | 11 |

## Not modelled
Motor screws, the spindle, wiring. The printed guides and racks suit light work (plotter, drag knife, laser, foam);
the M16 screw and nut are nominal ISO sizes, so print a test pair to tune clearance.
