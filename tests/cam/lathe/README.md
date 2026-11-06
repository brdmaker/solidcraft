# Lathe test parts

Turned parts for a lathe CAM: each is a half-section profile revolved about Z, from z 0 (the end
held in the chuck) to z = length (the front face). `parts.json` gives each part's finished
diameter and length, a standard round bar to turn it from, its material, its volume worked out
from the profile (Pappus), its cosmetic threads and the profile itself (lines and arcs in
(r, z), with the chamfers and fillets cut in). `parts.png` shows them. Built with `solidcraft-cli`
over MCP (sketch on XZ, `solid.revolve` about Z, `solid.thread`); the `.step` files carry the
same solids.

| Part | Bar | Features |
|---|---|---|
| `stepped-shaft` (steel) | Ø22 | Ø20 × 20, Ø16 × 30, Ø12 × 25, Ø10 × 15 steps; 0.5 chamfers, R0.5 / R0.3 shoulder fillets; a DIN 471 retaining-ring groove for Ø10 (Ø9.6 × 1.1) |
| `flanged-bushing` (brass) | Ø45 | Ø40 × 5 flange, Ø25 body to 30; Ø16 bore through; 0.5 chamfers inside and out, R0.5 under the flange |
| `bearing-sleeve` (aluminium) | Ø38 | Ø35 × 40; Ø15 bore; a Ø22 counterbore 9 deep from the front (608 bearing seat) with a DIN 472 internal ring groove (Ø23 × 1.1, 7 in); chamfers |
| `taper-arbor` (steel) | Ø32 | Ø30 × 15 grip; a taper Ø24 to Ø18 over 30 (5.71° half angle); a Ø9.5 × 3 thread undercut; M12 thread over 12 (cosmetic); 1 and 0.75 chamfers |
| `v-pulley` (aluminium) | Ø55 | Ø50 × 14 rim with a 40° V-groove 6 deep (2 flat at the bottom); Ø20 × 8 hub; Ø8 bore; chamfers, R0.5 at the hub |
| `knob` (brass) | Ø28 | Ø12 × 15 shank with an M12 thread over 12 (cosmetic); Ø25 body with an R2 grip groove; a spherical R15 dome to the front |

Every face is a flat face square to Z or a cylinder, cone, torus or sphere about Z;
`the_lathe_parts_are_turned_about_z_as_designed` checks that and each part's volume and size.
