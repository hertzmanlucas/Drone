# ESP32 quad frame v1

Printed frame for: ESP32 DevKit (30-pin) · 4in1 60A BLHeli_S ESC (30.5×30.5) · 4× YSIDO 1104 8600KV · MPU6050 · VL53L0X · 28.5×15 mm radio · 2S LiPo.

Front = camera end (+X). Motor-to-motor diagonal 152.7 mm, clears 3" props with ≥3 mm to everything at prop height (2.5" props also fit).

## Files
| File | What | Print |
|---|---|---|
| `frame_bottom` | Unibody plate + arms, ESC stack holes, battery strap slots, ToF tail (rear), camera interface (front) | PETG, flat, 4 walls, 50 % gyroid |
| `deck` | Electronics deck on the stack: 4 posts for the FC board + upright radio holder at the rear | PETG, flat, posts up |
| `camera_mount_19` / `camera_mount_14` | Optional. Bolts to the 2× M3 holes at the front. 19 mm micro or 14 mm nano FPV cam, 0–45° tilt | PETG, base down |
| `landing_foot` | Optional, print 4. Bolts on with the motor screws, keeps the battery off the ground | TPU 95A |
| `drone_assembly.step` | Everything incl. dummy electronics — open it to check fit | — |
| `drone_frame.py` | Parametric source (CadQuery). Edit numbers at the top, run, get new files | — |

STEP = edit in Fusion/Onshape/SolidWorks/FreeCAD. STL = straight to the slicer.

## Stack (bottom → top)
1. Battery under the plate, strap through the two side slots.
2. Plate. 4× M3×30 bolts up through the 30.5 mm holes.
3. M3 nylon nut on each bolt (≈3 mm spacer), then the ESC on its rubber grommets.
4. 4× ~12 mm spacers (10–15 mm is fine), then the deck, then M3 nuts.
5. FC perfboard (cut to 59×34 mm) on the 4 deck posts, M2 screws or foam tape.
6. Radio slides into the grooved posts at the rear, antenna up, pins facing back.
7. VL53L0X under the rear tail facing down (foam tape + zip ties), wires through the slot.

Motor screws: M2×6 without feet, M2×8 with feet. Check thread depth in the motor so the screw can't touch the windings.

## Check before printing
- ESC hole pattern: frame assumes 30.5×30.5. If yours is 20×20, set `STACK = 20`.
- Motor bolt circle: slots cover 8.5–9.5 mm (M1.6–M2). Measure yours.
- ESP32 board size (`ESP32`), only matters for the FC perfboard size.
