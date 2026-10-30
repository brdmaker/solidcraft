# 3D-printable CNC, 300 × 300 × 100 mm workspace (SolidCraft design)

This orphan branch holds a SolidCraft design only; it shares no history with `main`.

- `cnc-300x300x100.sc`: the design (open with SolidCraft, or `solidcraft-cli mcp --in cnc-300x300x100.sc`).
- `cnc-iso.png`, `cnc-workspace-tour.gif`: render and the "Workspace tour" motion study.
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

## Not modelled
Motor screws, the spindle, wiring. The printed guides and racks suit light work (plotter, drag knife, laser, foam);
the M16 screw and nut are nominal ISO sizes, so print a test pair to tune clearance.
