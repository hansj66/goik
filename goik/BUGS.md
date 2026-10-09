# Known bugs

Found during a code review in October 2026. Items marked **(verified)** were reproduced with a test
harness against the current code; the rest come from reading the code.

Items marked **(fixed)** have been fixed since the review.

Severity: **High** = wrong motion / crash / unsafe on hardware, **Medium** = wrong result in some
situations, **Low** = cosmetic, documentation or code hygiene.

## Gait engine (`robot/pod.go`, `robot/leg.go`, `robot/gaits.go`)

| # | Sev | Location | Bug |
|---|-----|----------|-----|
| 1 | High | `pod.go` `Update`/`SetStrideVector` | **(obsolete: table based gait removed)** **(verified)** `currentGaitCycle` is never reset. After a first `stride_vector 2 …` + `start` finishes, a second `stride_vector 2 …` + `start` does nothing (0 ticks), because `currentGaitCycle (2) < targetGaitCycles (2)` is false. Only a repeat count larger than the accumulated cycle count works. |
| 2 | High | `pod.go` `Stop` | **(obsolete: table based gait removed)** **(verified)** `Stop()` sets `targetGaitCycles = currentGaitCycle`. With an infinite walk (`nrepeats = 0`) stopped during the first cycle, the target becomes 0 (= "infinite") and the pod keeps walking. |
| 3 | High | `pod.go` `Stop` | **(obsolete: table based gait removed)** **(verified)** `Stop()` halts immediately, mid-swing, leaving the swing legs in the air (hexapod: 3 legs at z = -6 mm instead of the 38 mm ground level). It also resets `CurrentGaitIndex` to 0 while the legs keep their interpolation indices, so a later `start` swings the wrong legs. |
| 4 | High | `gaits.go` stance factors | **(obsolete: table based gait removed)** **(verified)** `StanceReturnSpeedFactor` is wrong for ripple (0.4) and the pentapod wave gait (0.2). A stance leg should travel 20 interpolation steps over `stancePhases*21 - 1` ticks, so the factor should be `(STEPS-1) / (stancePhases*STEPS - 1)`: ripple = 20/41 ≈ 0.488, pentapod = 20/83 ≈ 0.241, hexapod wave = 20/104 ≈ 0.192 (the "rounding error" in the comment), heptapod 20/125 = 0.16 (correct). With the current values the leg is still 3.2 steps from the end of stance when it lifts, then jumps to the start of the swing table. This should be computed instead of hard-coded. |
| 5 | High | `pod.go` `ResetInterpolator` | **(obsolete: table based gait removed)** **(verified)** All legs start at the middle of their table regardless of their phase in the gait. In wave gait the stance index of later legs runs out, wraps back to the front, and the leg then jumps to the back for its swing: a 60 mm foot teleport in one tick (normal step is 3 mm). Each leg needs a start index that matches its phase offset. |
| 6 | High | `gaits.go` / `commands.go` `executeGaitCmd` | **(verified, crash fixed)** `NewGait` has no case for 7 or 8 legs, and `executeGaitCmd` assigns the `nil` gait before checking the error. `gait wave` on the heptapod or spider (or `gait tripod` on the pentapod) sets `Gait = nil`, and the next render dereferences it and panics. *The command now returns the error and keeps the old gait. The 7 leg gait is now available; there is still no 8 leg gait.* |
| 7 | High | `leg.go` `UpdateSwing`, `RevertToNutral`, `UpdateRevertPhase0/1` | **(fixed: the gait engine keeps the last valid pose; the table based gait was removed)** **(verified)** IK errors are discarded (`l.ServoAngles, _ = SolveEffectorIK(...)`). On failure the solver returns a partly filled `ServoAngles` (femur = tibia = 0), so the leg snaps straight out. On hardware that is a violent move. The last valid angles should be kept and the error reported. |
| 8 | Medium | `pod.go` `SetRotation` | **(obsolete: table based gait removed)** **(verified)** Uses `π/360` where `π/180` was meant. `stride_angle 1 30` sweeps 15° in total (±7.5°), while `stride_vector` sweeps ±the given distance. One of the two conventions has to be chosen and applied to both. |
| 9 | Medium | `pod.go` `UpdatePodStructure` | **(fixed)** `CoxaCoordinates[l].Z * math.Pi / 360` scales a length by π/360. All examples have Z = 0, so this has no effect yet, but any pod whose coxa mounts are not at Z = 0 will be wrong. |
| 10 | Medium | `pod.go` `SetStrideVector`/`SetRotation` | **(obsolete: table based gait removed)** The stride is centred on the leg's *current* end effector position, not on `NeutralEffectorCoordinate`. Defining a new stride while walking (the "is walking" guard is commented out in the shell) centres it on a mid-stride position, and the pod drifts. |
| 11 | Medium | `pod.go` `POD_Z_HEIGHT` | **(obsolete: table based gait removed)** A global, set only in `NewPod` from leg 0. It is not updated by `load`, `set_*_length`, `set_*_angle` or `ground`, so strides use a stale ground height after any geometry change. For pods with different leg geometries (spider) the legs' neutral heights differ, yet all strides force leg 0's height. |
| 12 | Medium | `pod.go` `Set*Length`/`Set*Angle`, `LoadBodyDefinition` | **(obsolete: table based gait removed)** `UpdatePodStructure` recreates the legs with empty (all-zero) `IntermediateAngles` but leaves `HasDefinedStride`/`IsWalking` set. If the pod is walking or later started, all legs snap to 0/0/0. |
| 13 | Medium | `pod.go` `SetStrideVector`/`SetRotation` | **(obsolete: table based gait removed)** If IK fails for leg *n*, legs 0..n-1 already have new tables and the rest keep the old ones, while `HasDefinedStride` remains true from the earlier stride. The result is a mix of two strides. |
| 14 | Medium | `pod.go` `UpdateMovement` | **(obsolete: table based gait removed)** `endOfSwing` holds only the result of the *last* swing leg in the loop. This works only while all swing legs are in sync, and a pattern column with no swing leg would stall the gait forever. |
| 15 | Medium | `pod.go` `ReverseDirection` | **(obsolete: table based gait removed)** Reversing mid-stride causes jumps: a swing leg at index 0 immediately wraps to 20 and reports end-of-swing. |
| 16 | Medium | `leg.go` `UpdateRevertPhase1` | **(obsolete: table based gait removed)** The revert table starts at the leg's position *before* grounding (possibly lifted), so after phase 0 grounds the leg, phase 1 step 0 lifts it back up. |
| 17 | Low | `pod.go` `UpdateMovement` | **(obsolete: table based gait removed)** The `currentGaitCycle > targetGaitCycles` → `IsWalking = false` branch can never be reached because `Update` already stops at `>=`. The `>` / `<` checks disagree. |
| 18 | Low | `gaits.go` ripple pattern | Row 0 has 6 entries and the other rows have 3. Only the first 3 are used, so it works, but it reads like a mistake. The pattern is also really a 3-phase "tetrapod" gait (pairs of legs), not a classic overlapping ripple. Patterns are allocated with `make(GaitPattern, 36)` and mostly nil rows (these get serialised as `null` by `save`). |
| 19 | Low | `examplePods.go` | `NewExamplePentapod` has 6 `CoxaAngles` for 5 legs. `NewSpider` uses the hexapod tripod table for 8 legs, which only works because that table happens to have 8 rows. |
| 48 | Medium | `gaitEngine.go` `canLift` | **(verified)** Halting can deadlock: when two feet end up far from neutral on the same end of the body, lifting either one leaves the centre of gravity outside the support polygon, so `canLift` refuses both forever and the engine never becomes idle. Likely when the neutral stance has a small stability margin. Reproduced with a rectangular design (3 legs per side, 180 x 100 mm, coxa 50, femur 70, tibia 120, stance 100 mm with reach 80) with the same joint twists on all legs (coxa 10, femur 25, tibia -15), which moves all feet about 33 mm back (stability margin 57 mm instead of 90): 9 of 120 random three-step walks never settle after a halt. With balanced twists (front feet forward, rear feet back, margin 85 mm) and without twists: none of 120. Fix idea: shift the body's weight (pose X/Y) towards the remaining feet so a blocked foot can lift, as insects do. |

## Shell / simulator (`simulator/*.go`)

| # | Sev | Location | Bug |
|---|-----|----------|-----|
| 20 | High | `shell.go` / `simulator.go` | **(fixed)** Data race: shell commands run on the chatui goroutine and mutate `Pod` (or replace `s.Pod` entirely in `reset`) while Ebiten's `Update`/`Draw` read it on the game goroutine. There is no mutex. `executeGaitCmd` even busy-waits on `IsReverting` from the shell goroutine. |
| 21 | High | `simulator.go` / `commands.go` `reset`, `load` | **(obsolete: UDP streaming removed)** `networkcontroller` keeps a pointer to the *original* pod and a packet buffer sized for its leg count. After `reset <n>` the UDP stream keeps sending the old (now static) pod. |
| 22 | Medium | `commands.go` `executeGaitCmd` | **(obsolete: table based gait removed)** Changing gait also calls `Start()` (and ignores its error), so selecting a gait starts the robot walking. |
| 23 | Medium | `commands.go` `executeDownCmd` | **(fixed: `up`/`down` set the body height)** Half-implemented. It mutates `Joints[0..1].Z` (overwritten on the next FK update), shifts `NeutralEffectorCoordinate`, records a frame, and then returns "not implemented". It should be disabled until it is implemented. |
| 24 | Medium | `commands.go` `executeGroundCmd` | **(obsolete: `ground` replaced by `stance`)** Help text says "updates rest angles", but `RestAngles`/`NeutralEffectorCoordinate`/`POD_Z_HEIGHT` are not updated. |
| 25 | Medium | `bodyDefinition.go` `Load` | **(fixed)** Reads at most 1024 bytes. Larger definitions (spider, or anything with the serialised 36-row gait pattern) are truncated and fail to parse. Use `os.ReadFile`. |
| 26 | Medium | `commands.go` `executeExportCmd` | **(fixed)** The error returned by `MotionPrimitive.Export` is ignored, and the "exported" message is printed regardless. |
| 27 | Low | `commands.go:227` | **(fixed)** `go vet`: `Sprintf("Changing femur angle of leg %d to %2.2f", angle)` is missing the `legnum` argument. |
| 28 | Low | `commands.go` `Dispatch` | **(fixed)** Prefix matching (`upload` runs `up`) and `strings.Split(cmd, " ")` (double spaces break the argument count). Use `strings.Fields` and an exact match on the first word. |
| 29 | Low | `commands.go` help / README | **(help text fixed)** Help says `reset <1|2|3|4|5>`, but presets are 0–5. The root README's example session says "select model #3" and then runs `reset 1`. |
| 30 | Low | `commands.go` `record off` | **(obsolete: recording and export removed)** `record off` discards the recording, so you must `export` while still recording. Surprising: `record off` should stop recording and keep the data. |
| 31 | Low | `simulator/shell.go` | Copyright banner says 2024; file headers say 2025. |

## Recording / export (`robot/motionPrimitive.go`)

| # | Sev | Location | Bug |
|---|-----|----------|-----|
| 32 | High | `motionPrimitive.go` `normalize`, `networkController.go` `Update` | **(verified, fixed)** No clamping. Angles outside ±range/2 convert a negative float to `uint16` (−200° → `0xFF56`) and command the servo to a garbage position. The inverted formula `1024 - x` produces 1024 at the end stop, which is out of range for 10-bit servos (0–1023). Both need clamping to `[0, 1023]` (and the mapping should be shared). |
| 33 | High | `networkController.go` `Update` | **(obsolete: UDP streaming removed)** Checksum low byte is `uint8(checksum << 8)`, which is always 0. It should be `uint8(checksum & 0xFF)`. |
| 34 | Medium | `networkController.go` | **(fixed: export uses the servo mapping, streaming removed)** Servo orientation was hard-coded in streaming, while `export` took a configurable range and mask. |
| 35 | Medium | `motionPrimitive.go` | **(obsolete: recording and export removed)** The exported file is raw servo words with no header: no leg count, frame period, servo range, start/end pose or version. A player has to assume 18 servos and a fixed playback rate. |
| 36 | Low | `cmd/dummyrobotserver.go` | **(obsolete: UDP streaming removed)** Hard-codes 6 legs / 39 bytes and ignores the checksum. |

## ESP32 firmware

Bugs #37–#41 were in the ESP32 firmware (`esp32/udp_dxl`), which has been removed together with UDP
streaming. Overlord is the target now. See git history for the old entries.

## Overlord examples ([github.com/hansj66/overlord](https://github.com/hansj66/overlord))

Found while reading the Overlord repository (not verified on hardware).

| # | Sev | Location | Bug |
|---|-----|----------|-----|
| 44 | High | `firmware/examples/dynamixel/scan/scan.go` | **Check with a scope.** DXL_DIR (GPIO22) is pulled low 10 µs after `port.Write(...)` returns. On Linux a serial `Write` normally returns once the bytes are in the kernel buffer, not when they have left the UART, and a 6-byte ping takes ~1 ms at 57600 baud. DIR can therefore switch the NC7WZ241 to receive while the packet is still being transmitted. Wait for the transmitter to drain (`tcdrain`, i.e. the `TCSBRK` ioctl with argument 1) before switching DIR. |
| 45 | Low | `scan.go` | `pingPacket` is 10 bytes, but only 6 are used. `port.Write(pingPacket)` sends 4 trailing zero bytes after every ping. Write `pingPacket[:6]`. |
| 46 | Medium | `firmware/examples/imu/euler.go` | Roll and pitch are decoded with `binary.LittleEndian.Uint16`, but the BNO055 Euler registers are signed. The README output shows a roll of 4066.62°, which is really −29.4°. Use `int16(binary.LittleEndian.Uint16(...))`. |
| 47 | Low | `firmware/examples/dynamixel/README.md` | Says AX-12A uses protocol 2.0 and MX-12W protocol 1.0. The AX-12A only speaks protocol 1.0, and the MX-12W ships with 1.0 (2.0 only after a firmware update). |

## Repository hygiene

| # | Sev | Bug |
|---|-----|-----|
| 42 | Low | **(fixed)** Committed artefacts: `cpu.pprof` (profiling output) and `h1.hex`/`h2.hex` (old saved robot definitions) have been removed. |
| 43 | Low | **(obsolete: table based gait removed)** No tests for the table based gait. |
