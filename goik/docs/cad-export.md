# CAD export

The simulator can export a pod in its rest pose as a STEP assembly: a servo at every joint in the right place and
orientation, connected by printable brackets (body plates, coxa and femur brackets, tibias). Open it in Fusion 360 (or
any other CAD program) to check and refine the design, move its joints with the
[Fusion script](fusion-joints.md), and print the parts from the STL files.

![Example pod 6 exported with export_cad, in Fusion 360](../pictures/example_cad_export.png)

*Example pod 6 (`reset 6`) in Fusion 360: real AX-12A models, the body plates (blue) and the printable brackets
(orange). The joints were added by the [GOIK_Joints](fusion-joints.md) script.*

Pods with [twisted joints](designing-a-pod.md#twisted-joints) can be exported if only the femur and tibia joints are
twisted. See [Twisted joints](#twisted-joints).

## Quick start

```sh
python cad/tools.py setup     # once: installs CadQuery in cad/.venv (Python 3.9 - 3.12)
python cad/tools.py check     # optional: shows what is installed (the vendor servo models are included)
```

Or `make cad-setup` and `make cad-check`. Then in the simulator:

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
  `cad/servos.json` (the AX-12A and the STS3215):
  * two **body plates**, one on each case face of the coxa servos, screwed into the case holes. Cut outs for the horn
    and the idler (the bearing hub on the back of the servo), and a rounded nose at each coxa so the coxa bracket can
    turn
  * **coxa bracket**: arms on the coxa horn and idler, joined by side plates that are screwed to the femur servo's
    case faces
  * **femur bracket**, in two halves (horn side and idler side): an arm on the femur horn or idler and a pad screwed to
    the tibia servo case. The servos complete the box, like the femur of the PhantomX
  * **tibia**: arms on the tibia horn and idler, joined into a beam that ends in a ball foot
* Other servo models get placeholders instead: a base plate, rods and ball feet.

### Printing

No heat-set inserts: screws go through clearance holes into the servo.

| | AX-12A | STS3215 |
|---|---|---|
| Case | M2 through 2.4 mm holes, into the servo's own nuts (10 holes, the same on both sides) | Self-tapping M2 through 2.4 mm holes, into the 8 holes in the case walls (4 per side, at different positions on the two sides) |
| Horn | 4 x M2 on a 16 mm circle, into the tapped horn | 4 screws on a 14 mm circle into the horn (2.5 mm holes, probably M3: check with the real servo), 3.4 mm clearance |
| Idler (the back of the servo) | The hub's centre screw, 3.4 mm hole (check with the real servo) | The disc on the back, with the same 4 screw pattern as the horn |
| Arms | On the horn; on a 2 mm boss on the idler | On 3 mm bosses on both the horn and the disc, since they stand only about 2 mm proud of the case |

* Parts that a servo slides into have 0.2 mm of extra room.
* Each STL lies in the part's own frame on Z = 0: choose the print orientation (and supports) in the slicer.

### Collision check

After building, all parts and servos are checked for collisions in the rest pose (`--no-check` skips it), and every
printable part must be a single piece. A pod that
can't work stops with a message: the older example pods are too small for real servos, and exporting `reset 1` stops
with "the coxa is too short". Example pods 6 and 7 were designed around the AX-12A's and the STS3215's dimensions, and
nothing collides.

Then neighbouring legs are turned towards each other about their coxa axes, in 5 degree steps up to 45 degrees, to
find how far they can turn before their parts collide:

```
No collisions in the rest pose
Neighbouring legs: clear while their coxas turn 30 degrees towards each other. Legs 0 and 1 collide at 35 degrees (leg0_tibia and leg1_tibia).
```

The coxas typically turn 15 to 20 degrees either way while walking. With less room than 20 degrees, the message
suggests a larger body: `design scale <factor>` or `design radius <mm>` moves all legs out from the centre (see
[designing-a-pod.md](designing-a-pod.md#add-move-turn-and-remove-legs)).

Other joint angles aren't checked yet: folding a tibia far back against its femur will eventually hit the femur
bracket.

## Twisted joints

With [twisted joints](designing-a-pod.md#twisted-joints), the femur servo is turned about the coxa link and the tibia
servo about the femur link. The servos are placed from the joint frames, so they follow the twists by themselves. The
brackets follow them like this:

* **Coxa bracket** (femur twist): the arms on the coxa horn and idler stay in the coxa's plane. The side plates are
  built on the femur servo's case as without a twist, tilted with the servo, and trimmed to the arms. Tilted, the
  plates meet the two arms at different places, so each arm is only as wide as the plates where they meet it. Near
  the coxa servo the arms are as narrow as without a twist, and they widen towards the plates: neighbouring legs are
  closest to each other near the body.
* **Femur bracket** (tibia twist): the pads between the femur arms and the tibia servo's case become wedges, flat on
  the arm on one side and on the tilted case face on the other. The screw holes follow the case face.
* **Body plates**: a femur twist tilts the coxa bracket's side plates towards the coxa axis, so the plates' rounded
  nose around each coxa gets smaller.

![Example pod 7 with femur twists of 20 degrees and tibia twists of 15 degrees on the front and rear legs](../pictures/cad_twisted_pod.png)

![One of its twisted legs: coxa bracket (orange), femur bracket (green) and tibia (purple)](../pictures/cad_twisted_leg.png)

*Example pod 7 with femur twists of 20 degrees and tibia twists of 15 degrees on the front and rear legs (mirrored),
and one of those legs on its own: coxa bracket in orange, femur bracket in green, tibia in purple.*

### Example: femur twists of 30 and 40 degrees

Example pod 7 with longer coxas, the front legs' femurs twisted by 30 degrees and the rear legs' by 40 (their mirror
images get the opposite twists):

```
reset 7
design import
set_femur_twist 1 -30
set_femur_twist 5 40
set_coxa_length all 70
export_cad twisted9
```

![The twisted9 export in Fusion 360](../pictures/cad_twisted_fusion.png)

*The `twisted9` export in Fusion 360.*

The export reports:

```
No collisions in the rest pose
Neighbouring legs: clear while their coxas turn 5 degrees towards each other. Legs 4 and 5 collide at 10 degrees (leg4_coxa_bracket and leg5_coxa_bracket). A larger body gives the legs more room (design scale).
```

The rear legs' coxa brackets, tilted by 40 degrees, leave each other little room. A larger body moves the legs apart:
with `design scale 1.15` (or `design radius 76`: pod 7's legs are mounted 66 mm from the centre) before
`export_cad`, the rear legs can turn 15 degrees towards each other before their tibias meet.

### Limits

How far a joint can be twisted depends on the servo: the twisted servo has to stay clear of the arms it sits between.
For the example pods:

| | Femur twist | Tibia twist |
|---|---|---|
| AX-12A (pod 6) | Up to 21 degrees | Up to 21 degrees |
| STS3215 (pod 7) | Up to about 20 degrees with pod 7's 52 mm coxa; 30 degrees with a 62 mm coxa | Up to 45 degrees |

A twist that is too large stops the export with the reason, for example "the femur twist of -25.0 degrees turns the
femur servo into the arms of the coxa bracket. With this servo, at most 21.0 degrees fit", or "the coxa is too short
... A femur twist tilts the coxa bracket's side plates towards the coxa axis: make the coxa longer or the twist
smaller".

Coxa twists can't be exported yet: they tilt the coxa servos, and the body plates are built for upright coxa servos.

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
