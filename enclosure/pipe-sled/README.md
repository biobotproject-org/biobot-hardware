# Pipe enclosure: 6-inch PVC tube with a slide-in sled

The node lives in a length of 6-inch Schedule 40 PVC pipe hung vertically
on a post. A printed sled carries every part and slides in from the top.
The top is sealed with a PVC cleanout adapter and threaded plug, with an
IP68 cable gland for the solar lead. The bottom is open to the air through
stainless mesh, and the sensors hang there on the outer face of the
sled's nose disc, so they sample outside air rather than the warm air
around the electronics. A downward-facing opening cannot take rain.

`biobot_sled.py` is an Autodesk Fusion script that builds the sled from
the measured parts. Every dimension is a named constant at the top of the
file, in millimetres.

## Running the script

1. In Fusion, open a new design.
2. Utilities tab, Add-Ins, Scripts and Add-Ins. Click the "+" beside
   My Scripts, choose "Script or add-in from device", and select this
   folder (Fusion wants a folder whose `.py` has the same name).
3. Select `biobot_sled` and click Run. Two components appear: the sled
   and a translucent reference tube for checking fit.

Rerun into a fresh design; running twice in one design adds a second set.

## What it builds

Three bodies, each of which prints flat with no supports:

| Body | Print orientation | Size (mm) |
| --- | --- | --- |
| `plate` | rails up, component face on the bed | 136 x 237 x 4, rails 10 tall |
| `nose bulkhead` | sensor face on the bed | 152.5 dia x 4 |
| `top bulkhead` | outer face on the bed | 152.5 dia x 4 |

The bulkhead discs centre the sled in the tube. Each has a block on its
inner face; the plate rests on the blocks and is held by two M3 screws
per end (3.4 mm clearance in the plate, 2.6 mm pilot in the block).

On the plate, bottom to top:

- Three printed standoffs for the PCB at the gerber hole positions
  (70 x 52 mm board, 3.0 mm plated holes, M2.5 screws). The board lies
  with its 52 mm side across the sled and the JST sensor connectors
  toward the nose.
- The battery beside the PCB, held by two hook-and-loop straps at right
  angles: one across it at mid-length, one along it end to end, each
  through a pair of slots in the plate. The right-hand rail is notched
  where the across strap passes under the plate.
- Four printed standoffs for the Victron SmartSolar 75/15 (holes 105 x
  85 mm, M3 screws). It is mounted sideways with its terminals facing
  the PCB side. Victron's terminals-down rule is for cooling at the
  15 A rating; at the 0.7 A this node draws from a 10 W panel the
  controller dissipates well under a watt.

On the nose disc's outer face: four holes for the BME688 Qwiic board
(20.32 mm square, 3.2 mm) and two for the BMV080 enclosure (26 mm apart,
3.2 mm), both toward the PCB side, with a slot for the two Qwiic leads.
The top disc has a 26 mm finger hole for pulling the sled out and a
10 mm pass-through for the solar lead.

## Printing

The script writes three STL files next to itself when it finishes
(`sled_plate.stl`, `sled_nose_disc.stl`, `sled_top_disc.stl`); set
`EXPORT_STL = False` to skip that. Open each in your slicer (Bambu
Studio, PrusaSlicer, OrcaSlicer or Cura), lay it on its flat face with
"place on face", slice, and send to the printer. The plate goes rails
up, the discs go block up. No supports are needed.

## Materials

PETG or ASA, not PLA, which softens in a sun-baked pipe. Four walls and
25 to 30 percent infill for the plate; 20 percent for the discs.
Clearance between disc and pipe wall is `CLEAR` = 0.8 mm; print one disc
first and adjust it for your printer before printing the set.

## Not yet designed

- Mesh retainer for the bottom of the tube.
- Pipe-strap mounting positions and the solar panel bracket.
- Drip edge on the nose disc.
