# The gait engine

[robot/gaitEngine.go](../robot/gaitEngine.go) moves the legs. It is created when the pod first moves (`walk`, a body
pose command or `run`) and is dropped when the pod's structure changes. The same code is meant to run on the robot
controller (see [design-notes.md](design-notes.md)).

The engine's state is the **current position of every foot** plus a **phase** per leg. Commands only change rates,
never positions, so one motion can follow another (a new velocity, direction, gait or body pose) at any moment,
mid-stride, without the feet jumping.

## How it works

| Concept | How it works |
|---|---|
| Body velocity | A twist: x and y in mm/s, yaw in degrees/s. Grounded feet move opposite to it. Walking straight, turning on the spot and walking an arc are all just different twists |
| Gait | A duty factor (the fraction of the cycle a leg spends on the ground) and a phase offset per leg, derived from the gait pattern tables in [robot/gaits.go](../robot/gaits.go) |
| Swing | A leg lifts when its phase passes the duty factor and swings (smoothstep across, `sin²` up and down) towards a landing point that centres its next stance on its neutral position. The landing point is recomputed every tick, so velocity changes during a swing are absorbed |
| Velocity changes | Acceleration limited. Velocities are scaled down so no foot travels further than the maximum stride per stance, which is derived from the pod's measured reach |
| Gait changes | Each leg's phase is pulled towards its offset in the new gait (±25 % rate), and the duty factor is blended. The swing time is fixed, so the cycle time follows the duty factor: wave gait is slower than tripod |
| Stability | A leg that is due to lift waits until lifting it keeps the centre of gravity at least 10 mm inside the support polygon of the other feet. This never triggers in a steady gait; it shapes gait transitions |
| Reach | While a leg waits, the body slows down (and stops if necessary) if a grounded foot gets close to the edge of its reach |
| Stopping | `halt` sets the velocity to 0. The swing targets become the neutral positions, so the pod steps back into its neutral stance and goes idle |
| IK failure | A leg that can't reach its target keeps its last valid pose, and the error is counted (`engine`) |
| Body pose | Pitch, roll, yaw, shift and height of the body relative to the feet ([robot/bodyPose.go](../robot/bodyPose.go)). The feet stay in the ground frame and are transformed into the body frame before IK. A pose that would put a foot out of reach, or the body within 10 mm of the ground, stops at the last valid pose. Shifting the body moves the centre of gravity |
| Segmented bodies | Each foot is kept in its own segment's frame, and a grounded foot moves with its segment, which follows the head along its path. See [centipede.md](centipede.md) |
| Cycles and odometry | The engine counts gait cycles (`walk ... cycles <n>`) and integrates the body's motion into a world position and heading (`engine`, and the ground grid and path in the views) |

The simulator advances the engine 20 ms per update. Scripts run in the same robot time, so a script looks the same
at any `speed`.

## Axes

The README's convention: +Y is forward, and the feet are at +Z (Z points towards the ground).

* Pitch rotates around X: positive pitch raises the front (+Y) of the body.
* Roll rotates around Y: positive roll lowers the +X side.
* Yaw rotates around Z: positive yaw (and a positive yaw velocity) turns from +X towards +Y.
* `up` raises the body, which is negative Z.

## Tests

[robot/gaitEngine_test.go](../robot/gaitEngine_test.go) chains walk → turn → arc → wave → ripple (changing direction)
→ tripod (diagonal) → halt on four example hexapods and checks that there are no IK errors, that at least 3 legs are
on the ground and the centre of gravity is inside the support polygon on every tick, that no foot jumps (position or
velocity), and that the pod settles back exactly in its neutral stance. [robot/bodyPose_test.go](../robot/bodyPose_test.go)
covers the body pose, and checks that grounded feet keep a fixed position in the world.

## Limitations

* A swing starts and ends with zero velocity relative to the body, while stance feet move at the body's speed. A swing
  that matches the stance velocity at liftoff and touchdown would remove the remaining velocity step.
* The centre of gravity is assumed to be at the body origin. Shifting the body moves it, but tilting the body doesn't
  move its projection yet.
* The velocity is in the ground frame of the stance, not in the (yawed) body frame: with a `yaw` pose, `walk 0 50 0`
  still walks along the stance's Y axis.
* There is no gait table for 8 legs (the spider uses the hexapod tripod table, which happens to have 8 rows).
