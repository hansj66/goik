# Vendor servo models

The STEP files in this folder are 3D models of servos, published by their manufacturers. They are the property of
their respective owners and are **not** covered by the Apache 2.0 licence of this repository.

They are included for convenience, so the CAD export works without downloading them first. No licence or
redistribution statement was found for them (checked October 2026). If you are a rights holder and want a file
removed, please open an issue and it will be removed.

| File | Servo | Owner | Source | SHA-256 |
|---|---|---|---|---|
| AX-12A.step | ROBOTIS DYNAMIXEL AX-12A | ROBOTIS | [ROBOTIS Download Center](https://en.robotis.com/service/downloadpage.php?ca_id=70), entry "AX-12A / AX-18A / AX-12W / MX-12W" (downloaded as `AX-12.stp`) | `0178185b6dc62ecf4355784a0f48de2f939dc2f6c763e050671d1b15546b173f` |
| XL-320.step | ROBOTIS DYNAMIXEL XL-320 | ROBOTIS | [ROBOTIS Download Center](https://en.robotis.com/service/downloadpage.php?ca_id=70), entry "XL-320" | `70a83aa5f67b34047d21fb7110205ef18a38e41537c687bdef67ed45e298002d` |
| STS3215.step | Waveshare ST3215 (Feetech STS3215) | Waveshare / Feetech | [Waveshare wiki: ST3215 Servo](https://www.waveshare.com/wiki/ST3215_Servo), "ST3215 Servo STEP Module" (`ST3215.step` in `ST3215-3D.zip`) | `58e38e4dc49f97df738c5f229f9aa8a7dce64a0a1d01335486a52d53e6017e8a` |

The mounting data in [../servos.json](../servos.json) was measured on exactly these files (the SHA-256 checksums are
recorded there too). See [docs/servo-models.md](../../docs/servo-models.md).
