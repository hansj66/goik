# Chaining motion primitives

Design notes for smooth transitions between motions (walk → turn → arc → change gait → stop), and
for running them on a robot controller ([Overlord](https://github.com/hansj66/overlord)) from a
script, a gamepad or a remote device. The ESP32 is no longer a target for now.

Status: implemented in Go and tested: the gait engine with body pose (part 1), motion scripts in the
simulator and the servo mapping (part 2). Parts 3–4 are a proposal.

## Why the old table based gait couldn't chain

The original gait (`stride_vector` / `stride_angle` / `start` / `stop` / `revert`, now removed)
precomputed a table of 21 joint angle sets per leg for one stride, and played them back with an
index per leg. That gave three problems:

* **State is an index, not a position.** A new stride meant new tables, and the indices then pointed
  to different foot positions, so the feet jumped. Bugs #3, #5, #10 and #15 in [BUGS.md](BUGS.md) were
  all this problem.
* **No notion of where a leg is in the gait.** All legs started at the middle of the table, so only
  tripod gait started cleanly.
* **Recordings are joint angles with no metadata.** Two recordings can only be chained if one happens
  to end in the exact pose the other starts in. (Still true for `export`, see part 2.)

## Part 1: phase based gait engine (implemented)

[robot/gaitEngine.go](robot/gaitEngine.go) is the only way the pod moves. It is created when the pod
first moves (`walk`, a pose command or `run`) and dropped when the pod's structure changes. In the
simulator, `walk`, `halt`, `gait`, `swing_time`, `step_height`, the body pose commands and `engine`
control it.

The engine state is the **current foot position** of every leg plus a **phase** per leg. Commands
only change rates, never positions, so there are no jumps when one motion follows another.

| Concept | How it works |
|---|---|
| Body velocity | A `Twist` (x, y in mm/s, yaw in deg/s). Grounded feet move opposite to it. Walking straight, turning on the spot and walking an arc (roadmap item 9) are all just different twists. |
| Gait | A `PhaseGait`: a duty factor (fraction of the cycle in stance) and a phase offset per leg. These are derived from the existing pattern tables, so existing and future gaits plug straight in. |
| Swing | A leg lifts when its phase passes the duty factor and swings (smoothstep in XY, `sin²` lift in Z) towards a landing point that centres its next stance on the neutral position. The landing point is recomputed every tick, so velocity changes during a swing are absorbed. |
| Velocity transitions | Acceleration limited (`LinearAcceleration`, `YawAcceleration`). Velocities are scaled down so no foot travels more than `MaxStride` per stance. `MaxStride` is derived from the pod's measured reach. |
| Gait transitions | Each leg's phase is pulled towards its offset in the new gait (a small phase locked loop, ±25 % rate), and the duty factor is blended. The swing time is fixed, so cycle time follows the duty factor and wave gait is naturally slower than tripod. |
| Stability guard | A leg that is due to lift waits until lifting it keeps the centre of gravity (body origin) at least `StabilityMargin` inside the support polygon of the other feet. This never triggers in a steady gait; it shapes gait transitions. |
| Reach governor | While a leg waits, the body slows down (and stops if necessary) if a grounded foot gets close to the edge of its reach. Recovery is rate limited. |
| Stopping | `halt` sets the velocity to 0. Swing targets become the neutral positions, so the pod steps back into its neutral stance (skipping legs that are already there) and goes idle. This fixes "stop leaves legs in the air". |
| IK failure | The leg keeps its last valid pose and the error is counted, instead of snapping to 0/0/0. |
| Body pose | Pitch, roll, yaw, X/Y shift and height of the body relative to the feet ([robot/bodyPose.go](robot/bodyPose.go)). The feet are kept in the ground frame and transformed into the body frame before IK. The pose is ramped (`PoseRotationRate`, `PoseTranslationRate`) and works while standing or walking. A pose that would put a foot out of reach, or a corner of the body within `MIN_GROUND_CLEARANCE` of the ground, is stopped at the last valid pose (`PoseLimited`). Shifting the body moves the centre of gravity used by the stability guard. |
| Cycles | `Cycles()` counts completed gait cycles, so a walk can last a number of cycles (`walk ... cycles <n>`), like the old `stride_vector <n>` / `step`. `Stride()` reports the current stride length. |

Tests ([robot/gaitEngine_test.go](robot/gaitEngine_test.go)) chain walk → turn → arc → wave →
ripple (changing direction) → tripod (diagonal) → halt on three example hexapods and check:

* no IK errors
* at least 3 legs grounded and the centre of gravity inside the support polygon on every tick
* no position or velocity discontinuities (bounded by the analytical maximum of a normal swing; the
  old engine jumped 60 mm in one tick)
* the pod settles back exactly in the neutral stance

### Trying it in the simulator

```sh
>reset 1
>speed 10             # the engine advances 20 ms per pod update; lower speeds are slow motion
>walk 0 80 0          # forward, 80 mm/s
>walk 0 50 15         # arc
>gait wave            # blends into wave gait (and slows down) while walking
>pitch 10             # raise the front of the body while walking
>walk 40 0 0          # sideways
>gait tripod
>halt                 # steps back into the neutral stance
>level                # body back to the neutral pose
>walk 0 30 0 cycles 4 # walk 4 gait cycles, then halt
>engine               # show phases, stride, cycle time, pose, IK errors
```

The XY view shows the feet's recent trails, the neutral foot positions (circles) and where swinging
feet will land (blue rings). All views draw the pod in the ground frame, so the body pose is visible.

`record on` / `export` work while the engine is running. Idle ticks are not recorded.

### Known limitations / next steps for the engine

* The swing starts and ends with zero velocity relative to the body, while stance feet move at the
  body speed. A Hermite swing that matches the stance velocity at liftoff and touchdown would remove
  the remaining velocity step (about 3 mm/tick at 50 Hz).
* The centre of gravity is assumed to be at the body origin. Shifting the body moves it, but tilting
  the body doesn't move its projection yet (that needs the height of the centre of gravity).
* The velocity is in the ground frame of the stance, not in the (yawed) body frame. With a yaw pose,
  `walk 0 50 0` still walks along the stance's Y axis.
* There is no gait table for 8 legs (the spider uses the hexapod tripod table, which happens to have
  8 rows). Gaits are easier to generate from the leg angles than to write as tables.

## Part 2: what a "motion primitive" becomes

Two kinds, stored side by side:

1. **Locomotion primitives** are parameters, not frames: gait + twist + duration (+ swing time,
   step height, later body pose). They are tiny, readable, and any of them can follow any other,
   because the engine handles the transition.
2. **Clips** are keyframed joint- or foot-space animations for things that aren't walking (wave a
   leg, bow, dance). Contract: **a clip starts and ends in the neutral stance.** Before a clip
   plays, the engine halts and settles. Afterwards it resumes from neutral. The export validates
   the start and end pose. With this contract any clip can be chained with anything else, with no
   pairwise transitions to design.

A **script** is a sequence of both. The shell, scripts and the remote should all use **the same
command language**.

### Motion scripts (implemented)

[script/script.go](script/script.go) parses and runs scripts. It has no simulator dependency, so
the same runner can run on Overlord. It drives anything that implements `script.Target`, and
`*robot.GaitEngine` does. In the simulator, scripts live in `scripts/*.goik` and are started with
`run <name>`. `abort`, or a manual `walk`/`halt`, stops them. The current line and the engine state
are shown at the bottom of the window. See [scripts/demo.goik](scripts/demo.goik).

```
gait <tripod|ripple|wave>          blend into a new gait
walk <x> <y> <yaw> [for <seconds> | cycles <n>]
                                   set the body velocity (mm/s, mm/s, degrees/s), optionally for a
                                   while or a number of gait cycles
wait <seconds>                     keep the current motion for a while
halt                               stop and wait until the pod (legs and body) has settled
swing_time <seconds>               duration of a leg swing
step_height <mm>                   height of the swing arc
pitch | roll | yaw <degrees>       rotate the body (positive pitch raises the front, positive roll
                                   lowers the +X side, positive yaw turns from +X towards +Y)
up | down <mm>                     raise or lower the body relative to the neutral stance
shift <x> <y>                      move the body (and its centre of gravity)
level                              return the body to the neutral pose
repeat [count]                     run the block since the previous repeat (or the start of the
                                   script) count times in total. Without a count: forever
```

* Scripts run in **robot time** (the engine's 20 ms step), so they look the same at any `speed`.
* The whole file is checked before anything runs, and errors include line numbers.
* The pod always halts at the end of a script. A `repeat` loop that never waits is detected.
* Body pose commands set a target that the body moves towards while the script continues. Use
  `wait` to let it get there.
* In the shell, `walk ... for <s>` and `walk ... cycles <n>` run as one-line scripts (and halt at the end).
* Not yet: `play <clip>` (needs the clip format) and nested loops.

### Servo mapping (implemented)

[robot/servoMapping.go](robot/servoMapping.go). Each joint has a servo id, an inversion flag, an
offset and optional soft limits. The pod has a servo model and a baud rate. The mapping is stored
in the pod definition (`save`/`load`; pods saved without one get the default mapping, which uses
the README addressing scheme). `export` uses it (and Overlord will), and values are always
clamped to the servo range.

| Model | Protocol | Range | Positions | Goal position address |
|---|---|---|---|---|
| AX-12A (default) | 1.0 | 300° | 1024 | 30 |
| XL-320 | 2.0 | 300° | 1024 | 30 |
| STS3215 | 1.0 compatible (Feetech) | 360° | 4096 | 42 (*to be verified against Feetech's memory table*) |

## Part 3: where it runs: Overlord

The target is the [Overlord](https://github.com/hansj66/overlord) controller, a carrier board for
the Raspberry Pi CM5. The robot runs the *same* Go gait engine as the simulator: `robot` and
`script` cross-compile to `linux/arm64` with `CGO_ENABLED=0` (verified), so whatever you design in
the simulator is what the robot does. Overlord also brings a BNO055 IMU (body levelling, fall
detection) and CAN for Cybergear motors.

The ESP32 firmware and the UDP streaming of servo positions from the simulator have been removed.
Live mirroring of the simulator is replaced by sending high-level commands to Overlord and showing
its telemetry in the simulator (below).

### Overlord architecture (proposal)

A headless program, `cmd/overlord`, built with
`GOOS=linux GOARCH=arm64 CGO_ENABLED=0 go build ./cmd/overlord`. It must not import `simulator` or
`views`.

```
 gamepad (/dev/input/js0) ─► twist / gait ─┐
 UDP text commands ────────────────────────┼─► command queue ─► control loop @ 50 Hz
 script runner (scripts/*.goik) ───────────┘                     engine tick → IK → servo mapping
                                                                  → Dynamixel Sync Write (/dev/ttyAMA0)
 BNO055 IMU (/dev/i2c-1) ─► body attitude (later: levelling, fall detection)
 telemetry (UDP) ─► simulator in "follow" mode shows the real robot's state
```

* **Servo bus:** UART0 on `/dev/ttyAMA0`, with direction controlled by GPIO22 (`DXL_DIR`, high =
  transmit) through the NC7WZ241 buffer. A `dynamixel` package with Protocol 1.0 and 2.0 framing
  (AX-12A/MX-12W use 1.0, the XL-320 uses 2.0), Ping, Read, Write and **Sync Write**. The control loop
  only writes broadcast Sync Write packets, which get no status reply, so DIR stays high during
  control. It only has to be switched for reads such as ping and feedback.
* **Bus time:** one Sync Write of 18 positions is about 60–70 bytes. That is about 0.7 ms at
  1 Mbps, but about 11 ms at 57600 baud. 50 Hz works either way, but the servos should be configured
  for 1 Mbps.
* **Servo mapping:** the pod definition saved by the simulator, including its servo mapping, is
  copied to the robot. Overlord and the simulator then map angles to servo positions identically.
* **Remote control:** gamepad left stick → x/y velocity, right stick → yaw, buttons → gait and
  halt. Plus the same UDP text commands as the shell. A watchdog halts the pod if the gamepad or
  remote goes quiet.
* **Testing without hardware:** the control loop writes to a `ServoBus` interface. The real
  implementation is the UART. A fake records the frames, so the whole program can be tested on a PC.

### Things noticed in the Overlord examples

Also tracked as #44–#47 in [BUGS.md](BUGS.md).

* **DIR switching (needs checking with a scope):** `scan.go` writes the packet and pulls DIR low
  10 µs later. A serial `Write` on Linux normally returns once the bytes are in the kernel buffer,
  not when they are on the wire, so DIR can drop while the packet is still being sent. A 6-byte ping
  takes about 1 ms at 57600 baud. The safe pattern is to wait for the transmitter to drain
  (`tcdrain`, i.e. the `TCSBRK` ioctl with argument 1) before switching to receive. `scan.go` also
  writes the whole 10-byte `pingPacket` buffer, so 4 trailing zero bytes are sent after each ping.
* **IMU:** `euler.go` decodes roll and pitch as `Uint16`. They are signed, which is why the example
  output shows a roll of 4066.62° (that is −29.4°). Use `int16(...)`.
* **Protocol versions:** the Dynamixel README says AX-12A uses protocol 2.0 and MX-12W uses 1.0. The
  AX-12A only speaks 1.0, and the MX-12W ships with 1.0.

## Part 4: suggested order of work

1. ~~Phase based gait engine with smooth velocity and gait transitions~~ (done)
2. ~~Shell/update locking (#20), network controller pod pointer (#21), clamping (#32), checksum (#33)~~ (done)
3. ~~Servo mapping in the pod definition~~ (done)
4. ~~Motion scripts in the simulator~~ (done)
5. ~~Remove the table based gait (`stride_vector`, `start`, `stop`, `revert` ...); body pose~~ (done)
6. `dynamixel` package for protocol 1.0 (AX-12A, STS3215): Ping, Read, Write, Sync Write, DIR
   handling with `tcdrain`, plus a `ServoBus` interface with a fake
7. `cmd/overlord`: control loop + gamepad. First milestone: walking with the gamepad. Then running
   scripts from disk and UDP text commands (sharing the script parser)
8. Clip format v2 with the neutral start/end contract, and `play <clip>` in scripts
9. Telemetry, so the simulator can show the real robot. IMU levelling (the body pose makes this a
   matter of feeding pitch/roll corrections back into `SetBodyPose`)

## Open decisions

* ~~Overlord as the primary target~~: yes. The ESP32 firmware and UDP streaming have been removed.
* ~~Which servos~~: AX-12A first (protocol 1.0), with the STS3215 as a protocol compatible option.
* **Clip space:** joint angles (simple, what `export` produces today) or foot positions (survive
  geometry changes, need IK on the robot, which Overlord has).
* **Remote:** the Bluetooth gamepad on Overlord covers direct control. Is a network remote
  (phone, PC) also needed? If so, UDP text commands are probably enough.
