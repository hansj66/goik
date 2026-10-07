# Design notes: motion chaining and the robot controller

Design notes for smooth transitions between motions (walk → turn → arc → change gait → stop), and for running them on
a robot controller ([Overlord](https://github.com/hansj66/overlord)) from a script, a gamepad or a remote device.

Implemented and documented: the gait engine with body pose ([gait-engine.md](gait-engine.md)), motion scripts
([scripts.md](scripts.md)) and the servo mapping ([servos.md](servos.md)). The robot controller (below) is a proposal.

## Why the old table based gait couldn't chain

The original gait (`stride_vector` / `stride_angle` / `start` / `stop` / `revert`, now removed) precomputed a table of
21 joint angle sets per leg for one stride, and played them back with an index per leg. That gave three problems:

* **State is an index, not a position.** A new stride meant new tables, and the indices then pointed to different foot
  positions, so the feet jumped. Bugs #3, #5, #10 and #15 in [BUGS.md](../BUGS.md) were all this problem.
* **No notion of where a leg is in the gait.** All legs started at the middle of the table, so only tripod gait started
  cleanly.
* **Recordings are joint angles with no metadata.** Two recordings can only be chained if one happens to end in the
  exact pose the other starts in. (Still true for `export`.)

The gait engine replaces positions-by-index with the actual foot positions plus a phase per leg, and only lets commands
change rates. See [gait-engine.md](gait-engine.md).

## What a "motion primitive" becomes

Two kinds, stored side by side:

1. **Locomotion primitives** are parameters, not frames: gait + velocity + duration (+ swing time, step height, body
   pose). They are tiny, readable, and any of them can follow any other, because the engine handles the transition.
   These are the commands of the [script language](scripts.md).
2. **Clips** are keyframed joint- or foot-space animations for things that aren't walking (wave a leg, bow, dance).
   Contract: **a clip starts and ends in the neutral stance.** Before a clip plays, the engine halts and settles.
   Afterwards it resumes from neutral. The export validates the start and end pose. With this contract any clip can be
   chained with anything else, with no pairwise transitions to design. (Not implemented yet.)

A script is a sequence of both. The shell, scripts and the remote should all use the same command language.

## Where it runs: Overlord

The target is the [Overlord](https://github.com/hansj66/overlord) controller, a carrier board for the Raspberry Pi CM5.
The robot runs the *same* Go gait engine as the simulator: `robot` and `script` cross-compile to `linux/arm64` with
`CGO_ENABLED=0` (verified), so whatever you design in the simulator is what the robot does. Overlord also brings a
BNO055 IMU (body levelling, fall detection) and CAN for Cybergear motors.

The ESP32 firmware and the UDP streaming of servo positions from the simulator have been removed. Live mirroring of
the simulator is replaced by sending high-level commands to Overlord and showing its telemetry in the simulator.

### Overlord architecture (proposal)

A headless program, `cmd/overlord`, built with `GOOS=linux GOARCH=arm64 CGO_ENABLED=0 go build ./cmd/overlord`. It
must not import `simulator` or `views`.

```
 gamepad (/dev/input/js0) ─► twist / gait ─┐
 UDP text commands ────────────────────────┼─► command queue ─► control loop @ 50 Hz
 script runner (scripts/*.goik) ───────────┘                     engine tick → IK → servo mapping
                                                                  → Dynamixel Sync Write (/dev/ttyAMA0)
 BNO055 IMU (/dev/i2c-1) ─► body attitude (later: levelling, fall detection)
 telemetry (UDP) ─► simulator in "follow" mode shows the real robot's state
```

* **Servo bus:** UART0 on `/dev/ttyAMA0`, with direction controlled by GPIO22 (`DXL_DIR`, high = transmit) through the
  NC7WZ241 buffer. A `dynamixel` package with Protocol 1.0 and 2.0 framing (AX-12A/MX-12W use 1.0, the XL-320 uses
  2.0), Ping, Read, Write and **Sync Write**. The control loop only writes broadcast Sync Write packets, which get no
  status reply, so DIR stays high during control. It only has to be switched for reads such as ping and feedback.
* **Bus time:** one Sync Write of 18 positions is about 60–70 bytes. That is about 0.7 ms at 1 Mbps, but about 11 ms
  at 57600 baud. 50 Hz works either way, but the servos should be configured for 1 Mbps.
* **Servo mapping:** the pod definition saved by the simulator, including its servo mapping, is copied to the robot.
  Overlord and the simulator then map angles to servo positions identically.
* **Remote control:** gamepad left stick → x/y velocity, right stick → yaw, buttons → gait and halt. Plus the same UDP
  text commands as the shell. A watchdog halts the pod if the gamepad or remote goes quiet. The gamepad code should
  be shared with the simulator (see the roadmap).
* **Testing without hardware:** the control loop writes to a `ServoBus` interface. The real implementation is the
  UART. A fake records the frames, so the whole program can be tested on a PC.

### Things noticed in the Overlord examples

Also tracked as #44–#47 in [BUGS.md](../BUGS.md).

* **DIR switching (needs checking with a scope):** `scan.go` writes the packet and pulls DIR low 10 µs later. A serial
  `Write` on Linux normally returns once the bytes are in the kernel buffer, not when they are on the wire, so DIR can
  drop while the packet is still being sent. A 6-byte ping takes about 1 ms at 57600 baud. The safe pattern is to wait
  for the transmitter to drain (`tcdrain`, i.e. the `TCSBRK` ioctl with argument 1) before switching to receive.
  `scan.go` also writes the whole 10-byte `pingPacket` buffer, so 4 trailing zero bytes are sent after each ping.
* **IMU:** `euler.go` decodes roll and pitch as `Uint16`. They are signed, which is why the example output shows a roll
  of 4066.62° (that is −29.4°). Use `int16(...)`.
* **Protocol versions:** the Dynamixel README says AX-12A uses protocol 2.0 and MX-12W uses 1.0. The AX-12A only speaks
  1.0, and the MX-12W ships with 1.0.

## Order of work

1. ~~Phase based gait engine with smooth velocity and gait transitions~~ (done)
2. ~~Shell/update locking, clamping of servo values~~ (done)
3. ~~Servo mapping in the pod definition~~ (done)
4. ~~Motion scripts in the simulator~~ (done)
5. ~~Remove the table based gait; body pose~~ (done)
6. `dynamixel` package for protocol 1.0 (AX-12A, STS3215): Ping, Read, Write, Sync Write, DIR handling with `tcdrain`,
   plus a `ServoBus` interface with a fake
7. `cmd/overlord`: control loop + gamepad. First milestone: walking with the gamepad. Then running scripts from disk
   and UDP text commands (sharing the script parser)
8. Clip format v2 with the neutral start/end contract, and `play <clip>` in scripts
9. Telemetry, so the simulator can show the real robot. IMU levelling (the body pose makes this a matter of feeding
   pitch/roll corrections back into the body pose)

## Open decisions

* ~~Overlord as the primary target~~: yes. The ESP32 firmware and UDP streaming have been removed.
* ~~Which servos~~: AX-12A first (protocol 1.0), with the STS3215 as a protocol compatible option.
* **Clip space:** joint angles (simple, what `export` produces today) or foot positions (survive geometry changes,
  need IK on the robot, which Overlord has).
* **Remote:** the Bluetooth gamepad on Overlord covers direct control. Is a network remote (phone, PC) also needed? If
  so, UDP text commands are probably enough.
