# Servos

## Servo mapping

The servo mapping ([robot/servoMapping.go](../robot/servoMapping.go)) connects the pod's joints to physical servos. It
is part of the pod definition, so `save` and `load` keep it. Pods without one get the default mapping: AX-12A servos
at 1 Mbps, ids as in the addressing scheme below, nothing inverted.

Per joint:

| Setting | Command | Does |
|---|---|---|
| Servo id | `servo <ALL \| leg> <joint> id <n>` | Address on the bus. Ids must be unique |
| Inverted | `servo ... invert <on \| off>` | The horn points the other way (towards -Z of the joint's reference frame) |
| Offset | `servo ... offset <degrees>` | Added to the joint angle, to compensate for how the horn is mounted |
| Soft limits | `servo ... limits <min> <max>` | Angles are clamped to these (both 0: the full servo range) |
| Case angle, axis offset | `servo ... case <degrees>`, `servo ... axis_offset <mm>` | How the servo sits on its axis in the [CAD export](cad-export.md) |

`<joint>` is `coxa`, `femur` or `tibia`. For the pod: `servo_model <AX-12A | STS3215 | XL-320>`. `servos` shows the
whole mapping.

Raw servo positions are always clamped to the servo's range.

## Servo models

| Model | Protocol | Range | Positions | Goal position register |
|---|---|---|---|---|
| AX-12A (default) | Dynamixel 1.0 | 300° | 1024 | 30 |
| XL-320 | Dynamixel 2.0 | 300° | 1024 | 30 |
| STS3215 | Feetech, Dynamixel 1.0 compatible | 360° | 4096 | 42 (to be verified against Feetech's memory table) |

The CAD geometry of the models is described in [servo-models.md](servo-models.md).

## Hardware

### Dynamixel servos

I designed this application as a design helper for my own dynamixel based pod projects. The forward and inverse
kinematics bits of the code is not really tied to any particular servo types, but it is assumed that the servo
midpoint is at 0 degrees.

The servos are driven by a separate controller board. The target is the Raspberry Pi CM5 based
[Overlord](https://github.com/hansj66/overlord) board, which is meant to run GOIK's gait engine directly (see
[design-notes.md](design-notes.md)).

### Servo addressing scheme

Dynamixel are serial servos and each servo requires its own unique address. The default addressing scheme is as follows.

|Leg|Coxa Id|Femur Id|Tibia Id|
|---|-------|--------|--------|
|0| 1| 2| 3|
|1| 4| 5| 6|
|2| 7| 8| 9|
|3| 10| 11| 12|
|4| 13| 14| 15|
|5|16|17|18|

### Hardware interfaces

Dynamixel servos are serial servos that can be daisy chained. Communication is half duplex using a single data line.
Robotis offers several commercial variants, both for controlling the servos via a computer or from a microcontroller.
See the [DYNAMIXEL Quick Start](https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-insert/) for more info.

Or you can roll your own by bolting a buffer / line driver to your favourite microcontroller. (I have used the
[NC7WZ241](https://rocelec.widen.net/view/pdf/r9bmizc4je/FAIRS42890-1.pdf?t.download=true&u=5oefqw) in past projects,
but this seem to to have reached end of life and is no longer produced)

> Tip: If you are using the [OpenCM 9.04](https://emanual.robotis.com/docs/en/parts/controller/opencm904/) board
> stand-alone or with the [OpenCM 485 expansion board](https://www.robotis.us/opencm-485-expansion-boar/), it is
> possible to use this as a replacement for the [U2D2](https://emanual.robotis.com/docs/en/parts/interface/u2d2/).
> Install [OpenCM IDE](https://emanual.robotis.com/docs/en/software/opencm_ide/getting_started/) and flash the board
> with the smartTosser.ino sketch). This enables you to connect to the servos using the
> [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) from your computer
> (handy for changing servo IDs).
