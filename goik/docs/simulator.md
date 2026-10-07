# Using the simulator

Start the simulator from the `goik/goik` folder (`make run`, `go run .` or the built `GOIK` / `GOIK.exe`). It looks
for `scripts`, `pods`, `primitives` and `cad` relative to the current folder.

It opens a window with four views and a command shell in the terminal. Type commands in the shell; `help` lists them
all, and `help <section>` shows one section (design, walk, pose, scripts, servos). `/quit` closes both the shell and
the window, and closing the window ends the shell.

## The views

| View | Shows |
|---|---|
| XZ (top left) | The pod from the side |
| XY (top right) | The pod from above, with a 50 mm ground grid that moves and rotates under the pod as it walks, the body's path (orange), the feet's recent trails, the neutral foot positions (circles) and where swinging feet will land (blue rings). Legs in swing are drawn in blue |
| Isometric (bottom right) | The pod in 3D, standing on the ground grid |
| Gait (bottom left) | The selected gait's pattern (white: swing, grey: stance), the number of gait cycles walked, and the recording state |

All views draw the pod in the ground frame, so a body pose (pitch, roll, height ...) is visible. The bottom of the
window shows the gait engine's state (walking, transitioning or idle, velocity, cycle time, IK errors) and the running
script's current line.

The scale is fixed at 1 pixel per mm.

## Example pods

| `reset` | Pod |
|---|---|
| 0 | Hexapod with an uneven leg layout |
| 1 | Hexapod with evenly spaced legs |
| 2 | Small hexapod with short tibias (loaded at start up) |
| 3 | Pentapod (wave gait only) |
| 4 | Heptapod (wave gait only) |
| 5 | Eight legged "spider" with different leg lengths |
| 6 | Hexapod designed around real AX-12A servos. Use this one for the [CAD export](cad-export.md): the older pods are too small for real servos |

`save <file>` and `load <file>` store pod definitions (including the [servo mapping](servos.md)) in the `pods` folder.

## Commands

### Pod design

| Command | Does |
|---|---|
| `reset <0-6>` | Load an example pod |
| `save <file>` / `load <file>` | Save / load the pod definition |
| `set_coxa_length <ALL \| leg> <mm>` | Also `set_femur_length` and `set_tibia_length` |
| `set_coxa_angle <ALL \| leg> <degrees>` | Rest angle. Also `set_femur_angle` and `set_tibia_angle` |
| `ground <height>` | Move all feet to Z = height (positive is down) and make that the rest stance |
| `zero` | Set all servos to 0 degrees (every leg stretched out straight) |
| `effectors` | Print the end effector (foot) positions |

Changing the pod's geometry stops the gait engine and a running script.

### Walking

See [gait-engine.md](gait-engine.md) for how walking works.

| Command | Does |
|---|---|
| `walk <x> <y> <yaw>` | Walk with velocity x, y (mm/s) and yaw (degrees/s). +Y is forward. Change it at any time: transitions are smooth |
| `walk <x> <y> <yaw> for <s>` | Walk for a number of seconds, then halt |
| `walk <x> <y> <yaw> cycles <n>` | Walk for a number of gait cycles, then halt |
| `halt` | Slow down and step back into the neutral stance |
| `gait <tripod \| ripple \| wave>` | Select the gait. Blends into the new gait while walking |
| `swing_time <seconds>` | Duration of a leg swing (default 0.4). The cycle time follows from it |
| `step_height <mm>` | Height of the swing arc |
| `speed <1-10>` | Simulation speed. At 10 the simulation runs at about real time; lower values are slow motion |
| `engine` | Print the gait engine state: velocity, stride, cycle time, body pose, distance travelled, leg phases, IK errors |
| `clear` | Clear the body path and foot trails in the XY view |

### Body pose

Works while standing or walking. The body moves smoothly to the new pose; poses the legs can't reach are stopped at
the last reachable pose.

| Command | Does |
|---|---|
| `pitch <degrees>` | Positive raises the front (+Y) |
| `roll <degrees>` | Positive lowers the +X side |
| `yaw <degrees>` | Positive turns the body from +X towards +Y |
| `up <mm>` / `down <mm>` | Body height relative to the neutral stance |
| `shift <x> <y>` | Move the body (and its centre of gravity) |
| `level` | Back to the neutral pose |

### Scripts

See [scripts.md](scripts.md).

| Command | Does |
|---|---|
| `run <script>` | Run `scripts/<script>.goik` |
| `abort` | Stop the running script and halt (a manual `walk` or `halt` does the same) |
| `scripts` | List the scripts |

### Servos, recording and CAD

See [servos.md](servos.md) and [cad-export.md](cad-export.md).

| Command | Does |
|---|---|
| `servos` | Show the servo mapping |
| `servo_model <AX-12A \| STS3215 \| XL-320>` | Select the servo type (default AX-12A) |
| `servo <ALL \| leg> <coxa \| femur \| tibia> id <n>` | Servo id on the bus |
| `servo ... invert <on \| off>` | The servo horn points the other way |
| `servo ... offset <degrees>` | Added to the joint angle (horn mounting) |
| `servo ... limits <min> <max>` | Soft limits in degrees |
| `servo ... case <degrees>` / `axis_offset <mm>` | How the servo sits on its axis in the CAD export |
| `record <on \| off>` | Record the servo angles while the pod moves. `off` discards the recording |
| `export <file>` | Save the recording to `primitives/<file>` |
| `debug` | Show how many servo angle sets have been recorded |
| `export_cad <name>` | Export the pod in its rest pose to `cad/out/<name>.*` |
