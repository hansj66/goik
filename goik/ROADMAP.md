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
1. Printable brackets, so the CAD export can be printed directly. Status October 2026: first version for the AX-12A in `cad/brackets.py` (see [docs/cad-export.md](docs/cad-export.md))
    1. Decisions: our own brackets for every supported servo model (no ROBOTIS frames or other third party brackets). No heat-set inserts: screws go through clearance holes or into slightly undersized holes in the plastic. Validate in Fusion 360 as early as possible
    1. (done for the AX-12A) Mounting features in servos.json: horn screw circle (4 x M2 on 16 mm), idler hub, case holes. Measured on the ROBOTIS model by surveying its circular features. TODO: the same for the STS3215 and XL-320, and teach `cad/tools.py measure` to report them
    1. (done) Parametric parts from the GOIK lengths, with every joint axis where the kinematics expect it: body plates on both coxa case faces, coxa bracket (arms on horn and idler, side plates on the femur servo case), femur bracket in two halves, tibia with a foot. Example pod 6 (coxa now 60 mm) has no collisions
    1. (done) Collision check of all parts in the rest pose. TODO: check over the joint ranges (sweep each joint), and between neighbouring legs
    1. TODO: validate in Fusion 360 and with a test print: screw access, idler attachment (the idler hub's centre screw), print orientation and supports per part, part strength (tibia beam 12 x 10 mm)
    1. TODO: per part print orientation in the STL export (currently each part lies in its own frame on Z = 0)
    1. Open question: printer, material and bed size (FDM with PLA or PETG?). The body plates of pod 6 are about 230 mm across
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
