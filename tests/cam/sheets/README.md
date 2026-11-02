# Test sheets

Nine aluminium parts on three sheets, one per thickness, for trying the whole chain on a machine:
plan, LinuxCNC program, placed models. Every inner corner is rounded wider than the cutter that
reaches it or dogboned in the model (R1.75 for the Ø3 finishing cutter in pockets, R3.5 in
through openings, which the Ø6 cuts free). Built with `solidcraft-cli` over MCP; the `.step` files
carry the same solids. `parts.png` shows the parts, `sheets-simulated.png` the sheets as the
simulation cuts them.

| Sheet | Part | Quantity | Features |
|---|---|---|---|
| `sheet-3mm.json` | `vent-cover` | 2 | 160 × 100 R5; five 8 wide vent slots through; a 50 × 30 window through, dogboned R3.5; a 60 × 14 label recess 1 deep, dogboned R1.75; 4 × M3 90° countersinks |
| | `hex-plate` | 2 | 80 across flats, corners R4; Ø30 bore; 6 × Ø5.5 on a 60 circle |
| | `washer-ring` | 4 | Ø70 / Ø36 ring; three 8 wide kidney pockets 1.5 deep on a 70° arc of R26.5 round the bore |
| `sheet-6mm.json` | `motor-bracket` | 2 | 100 × 80 L, corners R4, inside corner R6; two M8 adjustment slots (8.5 wide) through; 3 × M5 counterbores Ø10 × 3; 0.5 top chamfer |
| | `gusset` | 3 | 90 × 90 right triangle, corners R6; Ø24 lightening hole; 3 × Ø6.5 |
| | `fork-clamp` | 2 | 70 × 50 with a 20 wide fork open to one edge, its root R5; Ø10 pivot; 4 × Ø3.3 |
| `sheet-10mm.json` | `bearing-block` | 2 | 80 × 60 R4, 1 top chamfer; 608 bearing seat Ø22.2 × 7 over a Ø12 bore; a 20 × 30 pocket 4 deep, dogboned R1.75; 4 × M5 counterbores Ø10 × 5 |
| | `nema17-mount` | 2 | 60 × 60 R4; Ø22.2 × 2 pilot recess over a Ø16 bore; 4 × M3 90° countersinks on the 31 mm square |
| | `stepped-plate` | 1 | 110 × 70 R5; a 60 × 40 pocket 4 deep with R4 corners and a 36 × 18 step 3 deeper in it, dogboned R1.75; a pentagon pocket 5 deep (R13 corners circle) with R3 corners |

All three sheets are 400 × 300 with the work zero at the top centre and the built-in tool library.
Simulated on 0.2 mm cells, each is cut clean: nothing left on a part past 0.05 mm, nothing cut
into one, no rapid through material, every part free. The vent covers' windows drop loose 50 × 30
pieces when cut free.
