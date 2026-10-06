# Servo CAD files

The planned CAD export (see the roadmap) places a 3D model of the servo at every joint. These models
come from the servo vendors and are **not included in this repository**: no licence or
redistribution statement was found for them (checked October 2026), so they may be downloaded and
used, but not redistributed. Download them yourself from the links below.

Put the files in `goik/cad/vendor/` (ignored by git), named after the servo model:

```
goik/cad/vendor/AX-12A.step
goik/cad/vendor/XL-320.step
goik/cad/vendor/STS3215.step
```

## Where to download

| Servo | Source | Notes |
|---|---|---|
| AX-12A | [ROBOTIS Download Center](https://en.robotis.com/service/downloadpage.php?ca_id=70), entry "AX-12A / AX-18A / AX-12W / MX-12W", STEP format | Official. The download starts from a button on the page (there is no direct link) |
| XL-320 | [ROBOTIS Download Center](https://en.robotis.com/service/downloadpage.php?ca_id=70), entry "XL-320", STEP format | Official. Also linked from the [XL-320 e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl320/) |
| STS3215 | [Waveshare wiki: ST3215 Servo](https://www.waveshare.com/wiki/ST3215_Servo), "ST3215 Servo STEP Module" in the Resources section | Feetech's own [product page](https://www.feetechrc.com/525603.html) only has a PDF drawing. Waveshare sells the servo as "ST3215", but its wiki doesn't say it is the same part as the Feetech STS3215, so check the dimensions against Feetech's drawing |

Alternatives:

* [Poppy project Robotis library](https://github.com/poppy-project/Robotis-library): community models
  of the AX-12/18 and MX series under CC BY-SA 4.0 (attribution, share alike). SolidWorks format,
  so they would have to be converted to STEP.
* Community STS3215 models, for example on [Printables](https://www.printables.com/model/1477431-servo-motor-sts3215-feetech-model)
  or [step.parts](https://www.step.parts/parts/waveshare_feetech_st3215_servo). Check each model's
  licence and accuracy before using it.

## Mount data

The export needs to know where each servo's horn axis is in the vendor's STEP file (position,
direction, and the offset of the horn face). These values are measured once per servo model and
stored in GOIK. Vendors don't number their file versions, so record which file the values were
measured on (file name, download date and SHA-256), so that a changed file can be detected.

## Licensing

If a vendor gives explicit permission to redistribute their files, they could be committed in a
separate folder together with the source and terms, kept outside the Apache 2.0 licence of this
repository. Until then, keep them local.
