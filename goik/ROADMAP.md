# Roadmap

## R & D / TODO

1. Script / interpreter functionality for composing walking sequences / motion primitives / moves
1. Full hexapod step file generation (with input of servo step files). Status October 2026: working, see [docs/cad-export.md](docs/cad-export.md) (`export_cad`, `cad/goik_cad.py`, `cad/tools.py`)
    1. (done) Export an assembly description (JSON) from GOIK: per leg and joint the servo model, full transform from forward kinematics, horn direction (from the servo mapping's Inverted flag), link lengths and coxa mount points. The pod is exported in its rest pose
    1. (done) CadQuery script in the repo that reads the JSON plus the vendor STEP files, places a servo at every joint, and exports a STEP assembly (vendor models stored once and instanced) and one STL per printable part
    1. (done) Per servo model mount data for the AX-12A, XL-320 and STS3215, measured with `python cad/tools.py measure` on the vendor models and stored in `cad/servos.json`
    1. (done) Per joint mounting parameters: `servo ... case <deg>` and `servo ... axis_offset <mm>`
    1. (done) Simple solids: base plate, placeholder rods and ball feet. To be replaced by printable brackets (next item)
    1. (done) Convert coordinates: GOIK has +Z towards the ground, CAD expects Z up (rotate 180 degrees around X, keeping the frame right-handed)
    1. (done) Vendor STEP files are not committed (no licence or redistribution statement found). [docs/servo-models.md](docs/servo-models.md) lists where to download them (`goik/cad/vendor/`, ignored by git)
    1. (done) Example pod 6 (`reset 6`) designed around the AX-12A: no colliding servos
    1. (done) Collision check between all parts in the export (see the printable brackets item)
1. Printable brackets, so the CAD export can be printed directly. Status October 2026: first version for the AX-12A and STS3215 in `cad/brackets.py` (see [docs/cad-export.md](docs/cad-export.md))
    1. Decisions: our own brackets for every supported servo model (no ROBOTIS frames or other third party brackets). No heat-set inserts: screws go through clearance holes or into slightly undersized holes in the plastic. Validate in Fusion 360 as early as possible
    1. (done for the AX-12A and STS3215) Mounting features in servos.json: horn screws, idler (hub or disc), case holes per face, arm standoffs. Measured by surveying the vendor models' circular and flat faces. Example pod 7 (`reset 7`) is designed for the STS3215. TODO: the XL-320, and teach `cad/tools.py measure` to report mounting features
    1. To confirm with real servos and a test print (the mounting features come from the vendor 3D models, not from measuring real servos):
        1. AX-12A: the idler hub's centre screw (size and length; the idler arm has a 3.4 mm hole), and whether the hub turns with the output or the arm has to turn on it
        1. AX-12A: M2 screws and nuts in the case's nut slots, and the screw lengths through the brackets
        1. STS3215: the horn screw size. The horn has 2.5 mm holes, taken to be M3 (3.4 mm clearance holes in the arms)
        1. STS3215: the disc on the back. Does it turn with the output, how is it fixed, and does it take the same screws as the horn
        1. STS3215: self-tapping M2 screws in the 1.6 mm case holes: screw length and how well they hold in the case
        1. STS3215: the body plates and coxa side plates rest on the raised middle of the horn side case face, 1.1 mm above the mounting holes' rails. Check that they sit flat and the screws pull them down without cracking
        1. STS3215: Waveshare's ST3215 model was used. Compare it with Feetech's STS3215 drawing (case 45.22 x 24.72 x 35 mm, hole positions)
        1. Both: the 0.2 mm fit where a servo slides between bracket arms, the 2.4 mm clearance holes, and whether the 3 mm arms and 4 mm plates are stiff enough
        1. Both: the body plates only use the case holes at least 2 screw diameters outside the horn cut out (STS3215: the outer pair per face). Check that this holds the coxa servos firmly
    1. (done) Parametric parts from the GOIK lengths, with every joint axis where the kinematics expect it: body plates on both coxa case faces, coxa bracket (arms on horn and idler, side plates on the femur servo case), femur bracket in two halves, tibia with a foot. Example pod 6 (coxa now 60 mm) has no collisions
    1. (done) Collision check of all parts in the rest pose. TODO: check over the joint ranges (sweep each joint), and between neighbouring legs
    1. TODO: test print: screw access, print orientation and supports per part, part strength (tibia beam 12 x 10 mm). Fusion 360 import and joints validated (October 2026)
    1. TODO: per part print orientation in the STL export (currently each part lies in its own frame on Z = 0)
    1. Open question: printer, material and bed size (FDM with PLA or PETG?). The body plates of pod 6 are about 230 mm across
1. Validate a design against the servos it will use, while designing it (not only when exporting to CAD)
    1. Today: the CAD export checks the rest pose for collisions and stops if the coxa is too short for the servos. Nothing warns while a pod is being designed or walked in the simulator
    1. A `validate` command in the shell (and later live warnings in the views) for the selected servo model. Needs the servo dimensions in Go (case, horn axis position, mounting faces), shared with `cad/servos.json` so there is one source of truth
    1. Fit: the coxa servos fit around the body without overlapping, the coxa is long enough for the femur servo to clear the coxa servo at every coxa angle, the femur is long enough for the tibia servo (the checks behind example pods 6 and 7, made general)
    1. Collisions over the joint ranges: sweep each joint through its range (or the servo mapping's soft limits), and check servos and brackets against each other, the body and the ground
    1. Collisions between neighbouring legs while walking: run the gait engine through the gaits and speeds and check legs against each other
    1. Joint angles: every angle the gait engine and the body pose use stays within the servo's range and the soft limits (AX-12A and XL-320: 300 degrees, STS3215: 360)
    1. Load: static torque per joint with the pod's weight (servos, brackets, electronics) on the fewest legs a gait stands on, compared with the servo's stall torque, with a safety margin
    1. Report what fails and by how much (for example "coxa 8 mm too short for the STS3215"), and suggest the smallest change that fixes it
1. Interactive pod designer (longer term): compose a new bot by pointing and clicking in the views, combined with text input for exact values. Today a pod is designed with shell commands (`set_coxa_length` ...) or by editing code in robot/examplePods.go
    1. Body: draw or edit the body outline in the XY view (polygon points), or start from a template (round, hexagonal, rectangular), with exact sizes typed in
    1. Coxa anchor points: add, select, drag and delete anchor points on the body, snap to the outline or a grid, and set the mounting angle of each leg. Type exact coordinates and angles when needed
    1. Symmetry: mirror anchor points (and legs) across the X or Y axis, or repeat them around the centre (radial symmetry with n legs), so a symmetric pod only needs half (or one leg) designed
    1. Legs: design a leg (segment lengths, rest angles, servo model) in a leg editor with a side view, save it as a reusable leg type, and place it on one or more anchor points
    1. Live feedback while designing: reach, standing height and the design validation above (servo fit and collisions), so a design is checked as it is built
    1. Undo / redo, and save / load of designs (the existing pod files), so experimenting is cheap
    1. Builds on the visualization items below (camera controls: zoom, pan, selection) and the design validation item above
1. Improve visualization
    1. Footfall diagram (scrolling swing/stance timeline per leg) instead of the static gait pattern table, which shows the target gait and is wrong during transitions
    1. Phase dials: each leg's current phase compared to the phase its gait wants (shows legs re-timing during gait transitions)
    1. Support polygon, centre of gravity and stability margin (green to red) in the XY view
    1. Reach circles around the neutral foot positions. Colour stance feet as they approach the edge of their reach
    1. Ground grid that moves with the walk (integrated velocity) plus a trail of the body's path, so travel and arcs are visible
    1. Camera controls: zoom and pan (scale is fixed at 1 px = 1 mm), rotatable isometric view with a ground plane
    1. Pause and single-step (e.g. space / period key) for inspecting transitions frame by frame
    1. Joint angle panel: each servo's angle against its limits from the servo mapping, highlighting joints near or past a limit
    1. Fix view colours: alpha is 1 instead of 255 (e.g. RGBA{255, 0, 0, 1}), which is invalid for Ebiten's premultiplied colours
    1. Scale bar / units on the axes
1. Implement metachronal gait (for centipede type robots)
1. Gamepad control for test driving the pod in the simulator, sharing the code with the Overlord controller
    1. Input independent controller package (no Ebiten dependency, so it runs on Overlord): maps stick axes and buttons to gait engine commands. Left stick: x/y velocity, right stick: yaw rate. Buttons: cycle gait, halt, level. Dead zone, response curve (expo) and maximum speeds; the engine's acceleration limits keep it smooth
    1. Pose mode while a button is held: the sticks control body pitch, roll, yaw and height instead of walking
    1. Simulator input: Ebiten's gamepad API (standard gamepad layout, works on Windows, macOS and Linux)
    1. Overlord input: Linux joystick device `/dev/input/js0` (Bluetooth gamepad, as in the Overlord remote control example)
    1. Configurable button and axis mapping (gamepads differ), and the current stick and button state shown in the simulator
    1. Watchdog: halt if the input stops (gamepad disconnected or out of range). Essential on the real robot
