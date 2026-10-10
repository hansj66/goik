# Motion scripts

A script is a text file with one command per line, in the `scripts` folder with the extension `.goik`. Run it with
`run <name>`; `scripts` lists them, and `abort` (or a manual `walk` or `halt`) stops a running script. The current
line is shown at the bottom of the simulator window. See [scripts/demo.goik](../scripts/demo.goik) for an example, and
[scripts/centipede.goik](../scripts/centipede.goik) for the centipede (`reset 9`, then `run centipede`): a slalom,
circles to the left and right, changes of speed, turning the head on the spot and walking backwards.

```
# Everything after '#' is a comment
gait tripod
walk 0 80 0 for 3          # forward, 80 mm/s for 3 seconds
walk 0 0 20 for 3          # turn on the spot
walk 0 50 15 for 4         # arc
repeat 2                   # run the three moves above twice
gait wave                  # blend into wave gait while walking
walk 0 40 0 cycles 2       # two gait cycles
halt                       # step back into the neutral stance
pitch 10                   # raise the front of the body
wait 1
level
```

## Commands

| Command | Does |
|---|---|
| `gait <tripod \| ripple \| wave \| metachronal>` | Blend into a new gait |
| `walk <x> <y> <yaw>` | Set the body velocity (mm/s, mm/s, degrees/s) and continue with the next line |
| `walk <x> <y> <yaw> for <seconds>` | ... and keep it for a while |
| `walk <x> <y> <yaw> cycles <n>` | ... and keep it for a number of gait cycles |
| `wait <seconds>` | Keep the current motion for a while |
| `halt` | Stop, and wait until the pod (legs and body) has settled |
| `swing_time <seconds>` | Duration of a leg swing |
| `step_height <mm>` | Height of the swing arc |
| `pitch \| roll \| yaw <degrees>` | Rotate the body (see the axes in [gait-engine.md](gait-engine.md)) |
| `up \| down <mm>` | Body height relative to the neutral stance |
| `shift <x> <y>` | Move the body (and its centre of gravity) |
| `level` | Return the body to the neutral pose |
| `repeat [count]` | Run the block since the previous `repeat` (or the start of the script) `count` times in total. Without a count: forever |

## How scripts run

* Scripts run in **robot time** (the gait engine's 20 ms step), so they look the same at any `speed`.
* The whole file is checked before anything runs, and errors include the line number.
* Moves are chained without stopping: a new `walk` or `gait` blends into the next motion mid-stride. Only `halt`
  (and the end of the script) returns to the neutral stance.
* Body pose commands set a target that the body moves towards while the script continues. Use `wait` to let it get
  there.
* The pod always halts at the end of a script. A `repeat` loop that never waits is detected and stopped.
* Changing the pod (for example `reset` or `set_coxa_length`) stops a running script.
* In the shell, `walk ... for <s>` and `walk ... cycles <n>` run as one line scripts, so they also halt at the end.

The parser and runner ([script/script.go](../script/script.go)) have no simulator dependency, so the same scripts can
run on the robot controller later.

## Not yet

* `play <clip>` for keyframed motions that aren't walking (see [design-notes.md](design-notes.md))
* Nested loops
