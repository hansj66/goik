# Servo models for the CAD export

The CAD export needs each servo model's geometry: its size, where the horn is, and (for the printable brackets) its
mounting features. These are in [cad/servos.json](../cad/servos.json), together with how to place the vendor's 3D
model.

| Servo | Geometry in `cad/servos.json` | Vendor model |
|---|---|---|
| AX-12A | Measured on the ROBOTIS model, including mounting features | Manual download, mount transform measured |
| XL-320 | Measured on the ROBOTIS model | Manual download, mount transform measured |
| STS3215 | Measured on the Waveshare model, including mounting features | Downloaded by `cad/tools.py vendor`, mount transform measured |

`python cad/tools.py check` (or `make cad-check`) shows what is installed, downloaded and measured.

## Vendor models

The vendor models are included in `goik/cad/vendor/`, named after the servo model, so the CAD export works out of
the box:

```
goik/cad/vendor/AX-12A.step
goik/cad/vendor/XL-320.step
goik/cad/vendor/STS3215.step
```

They belong to their manufacturers and are not covered by this repository's licence: see
[cad/vendor/NOTICE.md](../cad/vendor/NOTICE.md). The links below are where they came from, for replacing a file with a
newer version (measure it again afterwards, see below). `python cad/tools.py vendor` downloads any that are missing,
where that can be done automatically. Without a vendor model, the export uses a box with a horn instead.

### Where to download

| Servo | Source | Notes |
|---|---|---|
| AX-12A | [ROBOTIS Download Center](https://en.robotis.com/service/downloadpage.php?ca_id=70), entry "AX-12A / AX-18A / AX-12W / MX-12W", STEP format | Official. The download starts from a button on the page (there is no direct link). The file is called `AX-12.stp`: save it as `AX-12A.step` |
| XL-320 | [ROBOTIS Download Center](https://en.robotis.com/service/downloadpage.php?ca_id=70), entry "XL-320", STEP format | Official. Also linked from the [XL-320 e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl320/) |
| STS3215 | [Waveshare wiki: ST3215 Servo](https://www.waveshare.com/wiki/ST3215_Servo), "ST3215 Servo STEP Module" in the Resources section | Downloaded automatically by `tools.py vendor`. Feetech's own [product page](https://www.feetechrc.com/525603.html) only has a PDF drawing. Waveshare sells the servo as "ST3215" and doesn't say it is the same part as the Feetech STS3215, so compare the model (45.22 x 24.72 x 35 mm case) with Feetech's PDF drawing |

Alternatives:

* [Poppy project Robotis library](https://github.com/poppy-project/Robotis-library): community models of the AX-12/18
  and MX series under CC BY-SA 4.0 (attribution, share alike). SolidWorks format, so they would have to be converted
  to STEP.
* Community STS3215 models, for example on [Printables](https://www.printables.com/model/1477431-servo-motor-sts3215-feetech-model)
  or [step.parts](https://www.step.parts/parts/waveshare_feetech_st3215_servo). Check each model's licence and
  accuracy before using it.

## Measuring a vendor model

A vendor model is only used once `cad/servos.json` knows where its horn is: `vendor_step.transform` maps the vendor
file's coordinates to the servo frame (origin on the horn axis at the outer face of the horn, +Z out of the horn, case
extending along -X). To measure a newly downloaded file:

```sh
python cad/tools.py measure cad/vendor/AX-12A.step
```

This finds the horn (the largest full circle on an outer face of the model; circles stored as several arcs are put
back together), prints its position and diameter, and suggests the transform. Check it against the drawing, paste the
transform into `servos.json`, and record the file's `sha256` (printed too), so that `tools.py` can warn if the vendor
ever changes the file. Then export again and compare the vendor model with the simple box in CAD.

The mounting features for the printable brackets (`mounting` in `servos.json`) were read from a survey of the vendor
model's circular and flat faces. `measure` doesn't report them yet. They are, in the servo frame:

| Field | Meaning |
|---|---|
| `mid_plane` | Z of the case's mid-plane (servos are centred on their leg plane) |
| `case_faces` | Z of the flat case faces the brackets sit on, horn side and idler side |
| `idler_face`, `idler_boss_diameter` | The outer face and diameter of the idler (hub or disc) on the back |
| `idler_screw_clearance`, `idler_screws` | The hole for the idler's centre screw, and the idler's screw pattern if it has one |
| `horn_screws`, `horn_screw_clearance`, `horn_centre_clearance` | The horn's screw pattern and the holes for it |
| `horn_standoff`, `idler_standoff` | How far the arms stand off the horn and idler (on a boss) |
| `case_holes` | Mounting holes in the case faces: one list for both faces, or `horn_side` and `idler_side` lists |
| `screw_clearance` | Clearance hole for the case screws |

Only XL-320 has no mounting features yet, so it gets placeholder parts in the export.

## Licensing

No licence or redistribution statement was found for the vendor models (checked October 2026). They are included for
convenience, kept apart from the code in `cad/vendor/` with their sources in
[NOTICE.md](../cad/vendor/NOTICE.md), and will be removed if a rights holder asks.
