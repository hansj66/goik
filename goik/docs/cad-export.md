# CAD export

The simulator can export a pod in its rest pose as a STEP assembly: a servo at every joint in the right place and
orientation, connected by printable brackets (body plates, coxa and femur brackets, tibias). Open it in Fusion 360 (or
any other CAD program) to check and refine the design, move its joints with the
[Fusion script](fusion-joints.md), and print the parts from the STL files.

![Example pod 6 exported with export_cad, in Fusion 360](../pictures/example_cad_export.png)

*Example pod 6 (`reset 6`) in Fusion 360: real AX-12A models, the body plates (blue) and the printable brackets
(orange). The joints were added by the [GOIK_Joints](fusion-joints.md) script.*

## Quick start

```sh
python cad/tools.py setup     # once: installs CadQuery in cad/.venv (Python 3.9 - 3.12)
python cad/tools.py vendor    # vendor servo models (see servo-models.md)
```

Or `make cad-setup` and `make cad-vendor`. Then in the simulator:

```
>reset 6                      # example hexapod designed for AX-12A servos
>export_cad myhexapod
```

This writes, in `cad/out/`:

| File | Contains |
|---|---|
| `myhexapod.json` | The assembly description: joint frames and servo placements, written by the simulator |
| `myhexapod.step` | The assembly, built from the description with CadQuery ([cad/goik_cad.py](../cad/goik_cad.py)) |
| `myhexapod_stl/` | One STL per printable part, each in its own frame lying on Z = 0 (servos are left out) |
| `myhexapod_joints.json` | The parts that move together and the joint axes, for the [Fusion script](fusion-joints.md) |

In Fusion 360: *File > Open > Open from my computer* and choose the STEP file. Every servo and part becomes a separate
component. The vendor servo model is stored once in the STEP file, so Fusion gives every servo the same name
(`leg0_coxa_servo_id1:1` ... `:18`).

Without CadQuery, `export_cad` still writes the JSON, and the model can be built later with
`cad/.venv/Scripts/python cad/goik_cad.py cad/out/myhexapod.json` (`cad/.venv/bin/python` on macOS and Linux).

## What is exported

* **Servos**: the vendor model when it has been downloaded and measured, or a box with a horn sized from the datasheet
  ([servo-models.md](servo-models.md)). Every servo is centred on its leg plane: the case mid-plane goes through the
  joint. Coordinates are in mm with Z up, so GOIK's +Z (towards the ground) becomes -Z.
* **Printable parts** ([cad/brackets.py](../cad/brackets.py)), for servo models with mounting features in
  `cad/servos.json` (currently the AX-12A):
  * two **body plates**, one on each case face of the coxa servos, screwed into the case holes. Cut outs for the horn
    and the idler (the bearing hub on the back of the servo), and a rounded nose at each coxa so the coxa bracket can
    turn
  * **coxa bracket**: arms on the coxa horn (4 x M2 on the 16 mm circle) and idler, joined by side plates that are
    screwed to the femur servo's case faces
  * **femur bracket**, in two halves (horn side and idler side): an arm on the femur horn or idler and a pad screwed to
    the tibia servo case. The servos complete the box, like the femur of the PhantomX
  * **tibia**: arms on the tibia horn and idler, joined into a beam that ends in a ball foot
* Other servo models get placeholders instead: a base plate, rods and ball feet.

### Printing

* Screws are M2 through 2.4 mm clearance holes, into the servo's own nuts and the tapped horn. No heat-set inserts.
* The idler arm uses a 3.4 mm hole for the idler's centre screw (check with the real servo).
* Parts that a servo slides into have 0.2 mm of extra room.
* Each STL lies in the part's own frame on Z = 0: choose the print orientation (and supports) in the slicer.

### Collision check

After building, all parts and servos are checked for collisions in the rest pose (`--no-check` skips it). A pod that
can't work stops with a message: the older example pods are too small for real servos, and exporting `reset 1` stops
with "the coxa is too short". Example pod 6 was designed around the AX-12A's dimensions, and nothing collides.

The check doesn't cover other joint angles yet: the coxa can turn freely, but folding a tibia far back against its
femur will eventually hit the femur bracket.

## How a servo sits on its joint

The horn axis is always the joint axis. How the servo sits around that axis is set per joint in the servo mapping
(saved with the pod):

```
>servo <ALL | legNum> <coxa|femur|tibia> case <degrees>      # rotate the case around the axis
>servo <ALL | legNum> <coxa|femur|tibia> axis_offset <mm>    # move the servo along the axis
>servo <ALL | legNum> <coxa|femur|tibia> invert on           # horn points the other way
```

At case angle 0, a case extends back along the link the servo is mounted on (towards the previous joint, or towards
the body for the coxa servos). The printable brackets assume the default (case angle 0, no axis offset).

## Command line

```
python cad/goik_cad.py <description.json> [-o folder] [--no-stl] [--assembly-stl] [--no-check]
```

`--assembly-stl` also writes the whole assembly (servos included) as one STL, for viewing.
