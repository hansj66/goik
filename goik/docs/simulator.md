# Using the simulator

Start the simulator from the `goik/goik` folder (`make run`, `go run .` or the built `GOIK` / `GOIK.exe`). It looks
for `scripts`, `pods` and `cad` relative to the current folder.

It opens a window with four views and a command shell in the terminal. Type commands in the shell; `help` lists them
all, and `help <section>` shows one section (design, walk, pose, scripts, servos). `/quit` closes both the shell and
the window, and closing the window ends the shell.

At start up the views window fills the left 2/3 of the screen (without the taskbar), and the terminal with the shell
the right 1/3, with the focus on the shell. The terminal is only moved on Windows, and only if it is a terminal window
(Windows Terminal or the classic console): started from an editor's terminal (VS Code's, for example), only the views
window is placed. `GOIK -layout=false` (or `go run . -layout=false`) leaves both windows where they open. The views
window can be resized, and the views always fill it.

## The views

| View | Shows |
|---|---|
| XZ (top left) | The pod from the side |
| XY (top right) | The pod from above, with a 50 mm ground grid that moves and rotates under the pod as it walks, the body's path (orange), the feet's recent trails, the neutral foot positions (circles) and where swinging feet will land (blue rings). Legs in swing are drawn in blue |
| Isometric (bottom right) | The pod in 3D, standing on the ground grid |
| Gait (bottom left) | The selected gait's pattern (white: swing, grey: stance), and the number of gait cycles walked |

All views draw the pod in the ground frame, so a body pose (pitch, roll, height ...) is visible. The bottom of the
window shows the gait engine's state (walking, transitioning or idle, velocity, cycle time, IK errors) and the running
script's current line.

The views are drawn at 1 pixel per mm on a picture with the window's shape and 1024 pixels along its shorter side,
which is scaled to fill the window. A wider window shows more of the ground around the pod.

## Example pods

| `reset` | Pod |
|---|---|
| 0 | Hexapod with an uneven leg layout |
| 1 | Hexapod with evenly spaced legs |
| 2 | Small hexapod with short tibias (loaded at start up) |
| 3 | Pentapod (wave gait only) |
| 4 | Heptapod (wave gait only) |
| 5 | Eight legged "spider" with different leg lengths |
| 6 | Hexapod designed around real AX-12A servos |
| 7 | Hexapod designed around real STS3215 servos |

Use pod 6 or 7 for the [CAD export](cad-export.md): the older pods are too small for real servos.

`save <file>` and `load <file>` store pod definitions (including the [servo mapping](servos.md)) in the `pods` folder.

## Commands

### Pod design

A step by step guide with examples: [designing-a-pod.md](designing-a-pod.md).

| Command | Does |
|---|---|
| `reset <0-7>` | Load an example pod |
| `save <file>` / `load <file>` | Save / load the pod definition |
| `set_coxa_length <ALL \| leg> <mm>` | Coxa length |
| `set_femur_length <ALL \| leg> <mm>` | Femur length |
| `set_tibia_length <ALL \| leg> <mm>` | Tibia length |
| `set_coxa_angle <ALL \| leg> <degrees>` | Coxa rest angle |
| `set_femur_angle <ALL \| leg> <degrees>` | Femur rest angle |
| `set_tibia_angle <ALL \| leg> <degrees>` | Tibia rest angle |
| `stance <height> [reach \| vertical]` | Set how high the body stands above the ground. The femur and tibia rest angles are computed from it. See [Stance](designing-a-pod.md#stance-how-high-the-pod-stands) |
| `stance reach <leg> <reach \| default>` | Reach (foot distance from the femur joint) for one leg |
| `stance` / `stance off` | Show the stance, with the possible heights / set the femur and tibia rest angles by hand again |
| `effectors` | Print the end effector (foot) positions |
| `design ...` | Design the pod from its body outline and leg mounts, symmetric about the Y axis. See [designing-a-pod.md](designing-a-pod.md) |

While a pod has a design, the leg commands change a leg and its mirror image together.

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

### Servos and CAD

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
| `export_cad <name>` | Export the pod in its rest pose to `cad/out/<name>.*` |
