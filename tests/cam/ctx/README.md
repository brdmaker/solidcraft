# Turn-mill test parts

Parts for a twin-spindle turn-mill lathe with live tools, bar fed through spindle 1. Each lies
along Z: z 0 is the back end (cut off from the bar, then finished in spindle 2), z = length is
the front (finished in spindle 1). `parts.json` gives each part's finished diameter and length,
a round bar to turn it from, its material, its volume worked out by hand, what is turned, and the
milled and drilled features that need live tools, with the side they are cut from. `parts.png`
shows them. Built with `solidcraft-cli` from the `*.build.json` scripts; the `.step` files carry
the same solids.

| Part | Bar | Turned | Live tools |
|---|---|---|---|
| `cross-pin` | Ø22 | Ø20 × 50, 0.5 chamfers both ends | a Ø6 cross hole along X at z 35; a keyway 5 wide (centres z 10..22) milled radially to x 7.5 |
| `hex-flange` | Ø50 | Ø45 × 20 flange, Ø12 bore through | a hex 28 across flats from z 20 to 30 (flats at 0°, 60°, …), milled from the front; 6 × Ø6.6 through on PCD 38 from 0° |
| `slotted-cap` | Ø32 | Ø30 × 25 | a slot 6 wide, 4 deep across the front face along X; two flats 24 across (y ±12), 8 long from the back |
| `face-pattern` | Ø45 | Ø40 × 20, Ø10 bore through, a 0.5 front chamfer | 6 × Ø5 blind holes 8 deep on PCD 28 from the front; a stadium pocket 20 × 10, 3 deep in the back face |

`the_turn_mill_parts_carry_their_milled_features` checks each part's volume and size, and that
every feature in `parts.json` is there as exact faces (hole and slot-end cylinders, flats and
floors as planes).
