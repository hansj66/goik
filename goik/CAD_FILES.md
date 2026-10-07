# CAD export

The simulator can export a pod in its rest pose as a STEP assembly, with a servo at every joint in
the right place and orientation, and printable brackets connecting them (body plates, coxa and femur
brackets, tibias). Import it in Fusion 360 (or any other CAD program) to check and refine the design,
and print the parts from the STL files.

## Quick start

```sh
python cad/tools.py setup     # once: installs CadQuery in cad/.venv (Python 3.9 - 3.12)
python cad/tools.py vendor    # optional: vendor servo models (see below)
```

Then in the simulator:

```
>reset 6                      # example hexapod designed for AX-12A servos
>export_cad myhexapod
```

This writes `cad/out/myhexapod.json` (the assembly description) and builds `cad/out/myhexapod.step`
and one STL per printable part in `cad/out/myhexapod_stl/`. In Fusion 360: *File > Open > Open from my computer* and choose the
STEP file. Every servo, link and the base plate become separate components, named after the leg
and joint (for example `leg2_femur_servo_id8`).

With make (macOS, Linux, or Windows with make installed): `make cad-setup`, `make cad-vendor`,
`make cad-check`. Without CadQuery, `export_cad` still writes the JSON, and you can build the model
later with `cad/.venv/Scripts/python cad/goik_cad.py cad/out/myhexapod.json` (`cad/.venv/bin/python`
on macOS and Linux).

## What is exported

* **Servos**: a vendor model (when downloaded and measured, see below) or a box with a horn, sized
  from the datasheet (`cad/servos.json`). Every servo is centred on its leg plane (the case mid-plane
  goes through the joint). Coordinates are in mm with Z up, so GOIK's +Z (towards the ground)
  becomes -Z.
* **Printable parts** (`cad/brackets.py`), for servo models with mounting features in `servos.json`
  (currently the AX-12A):
  * two **body plates**, one on each case face of the coxa servos, screwed into the case holes. Cut
    outs for the horn and idler, and a rounded nose at each coxa so the coxa bracket can turn
  * **coxa bracket**: arms on the coxa horn (4 x M2 on the 16 mm circle) and idler, joined by side
    plates that are screwed to the femur servo's case faces
  * **femur bracket**, in two halves (horn side and idler side): an arm on the femur horn or idler and
    a pad screwed to the tibia servo case. The servos complete the box, like the femur of the
    PhantomX
  * **tibia**: arms on the tibia horn and idler, joined into a beam that ends in a ball foot

  Screws are M2 through 2.4 mm clearance holes, into the servo's own nuts and the tapped horn. The
  idler arm uses a 3.4 mm hole for the idler's centre screw (check with the real servo). Parts that a
  servo slides into have 0.2 mm extra room. Each STL is written in the part's own frame, lying on
  Z = 0: choose the print orientation (and supports) in the slicer.
* Other servo models get placeholders instead: a base plate, rods and ball feet.

After building, all parts and servos are checked for collisions in the rest pose (`--no-check`
skips it). The check doesn't cover other joint angles yet: the coxa can turn freely, but folding a
tibia far back against its femur will eventually hit the femur bracket.

Each servo sits with its horn axis on the joint axis. How it sits around that axis is set per joint
in the servo mapping (saved with the pod):

```
>servo <ALL | legNum> <coxa|femur|tibia> case <degrees>      # rotate the case around the axis
>servo <ALL | legNum> <coxa|femur|tibia> axis_offset <mm>    # move the horn face along the axis
>servo <ALL | legNum> <coxa|femur|tibia> invert on           # horn points the other way
```

At case angle 0, a case extends back along the link the servo is mounted on (towards the previous
joint, or towards the body for the coxa servos).

Example pod 6 (`reset 6`) was designed around the AX-12A's dimensions: with the brackets, nothing
collides. Most of the older example pods were designed before the export existed and are too small
for real servos: exporting `reset 1` stops with "the coxa is too short".

## Servo models and vendor files

| Servo | Geometry in `cad/servos.json` | Vendor model |
|---|---|---|
| AX-12A | Measured on the ROBOTIS model, including mounting features (verified) | Manual download, mount transform measured |
| STS3215 | Measured on the Waveshare model (verified) | Downloaded by `tools.py vendor`, mount transform measured |
| XL-320 | Measured on the ROBOTIS model (verified) | Manual download, mount transform measured |

`python cad/tools.py check` shows the current state.

The vendor models are **not included in this repository**: no licence or redistribution statement
was found for them (checked October 2026), so they may be downloaded and used, but not
redistributed. They go in `goik/cad/vendor/` (ignored by git), named after the servo model:

```
goik/cad/vendor/AX-12A.step
goik/cad/vendor/XL-320.step
goik/cad/vendor/STS3215.step
```

### Where to download

| Servo | Source | Notes |
|---|---|---|
| AX-12A | [ROBOTIS Download Center](https://en.robotis.com/service/downloadpage.php?ca_id=70), entry "AX-12A / AX-18A / AX-12W / MX-12W", STEP format | Official. The download starts from a button on the page (there is no direct link) |
| XL-320 | [ROBOTIS Download Center](https://en.robotis.com/service/downloadpage.php?ca_id=70), entry "XL-320", STEP format | Official. Also linked from the [XL-320 e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl320/) |
| STS3215 | [Waveshare wiki: ST3215 Servo](https://www.waveshare.com/wiki/ST3215_Servo), "ST3215 Servo STEP Module" in the Resources section | Downloaded automatically by `tools.py vendor`. Feetech's own [product page](https://www.feetechrc.com/525603.html) only has a PDF drawing. Waveshare sells the servo as "ST3215" and doesn't say it is the same part as the Feetech STS3215, so compare the model (45.22 x 24.72 x 35 mm case) with Feetech's PDF drawing |

Alternatives:

* [Poppy project Robotis library](https://github.com/poppy-project/Robotis-library): community models
  of the AX-12/18 and MX series under CC BY-SA 4.0 (attribution, share alike). SolidWorks format,
  so they would have to be converted to STEP.
* Community STS3215 models, for example on [Printables](https://www.printables.com/model/1477431-servo-motor-sts3215-feetech-model)
  or [step.parts](https://www.step.parts/parts/waveshare_feetech_st3215_servo). Check each model's
  licence and accuracy before using it.

### Measuring a vendor model (mount transform)

A vendor model is only used once `cad/servos.json` knows where its horn is: `vendor_step.transform`
maps the vendor file's coordinates to the servo frame (origin on the horn axis at the outer face of
the horn, +Z out of the horn, case extending along -X). To measure a newly downloaded file:

```sh
python cad/tools.py measure cad/vendor/AX-12A.step
```

This finds the horn (the largest full circle on an outer face of the model), prints its position and
diameter, and suggests the transform. Check it against the drawing, paste the transform into
`servos.json`, and record the file's `sha256` (printed too), so that `tools.py` can warn if the
vendor ever changes the file. Then export again and compare the vendor model with the simple box in
CAD.

## Licensing

If a vendor gives explicit permission to redistribute their files, they could be committed in a
separate folder together with the source and terms, kept outside the Apache 2.0 licence of this
repository. Until then, keep them local.
