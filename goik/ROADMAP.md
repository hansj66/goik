# Roadmap

## R & D / TODO

1. Script / interpreter functionality for composing walking sequences / motion primitives / moves
1. Full hexapod step file generation (with input of servo step files)
    1. Export an assembly description (JSON) from GOIK: per leg and joint the servo model, full transform from forward kinematics, horn direction (from the servo mapping's Inverted flag), link lengths and coxa mount points. The pod is exported in its rest pose
    1. CadQuery (or build123d) script in the repo that reads the JSON plus the vendor STEP files, places a servo at every joint, and exports a STEP assembly (and STL). STEP is too complex to write directly from Go. A FreeCAD macro is an alternative for opening the result in FreeCAD
    1. Per servo model mount data (AX-12A, STS3215, XL-320): horn axis position and direction, and horn face offset, relative to the vendor STEP file's origin. Measured once per model and stored with the servo models
    1. Per joint mounting parameters with sensible defaults: rotation of the servo case around its own axis, and which side of the joint it sits on
    1. Simple solids: base plate (outline around the coxa mounts plus a margin, configurable thickness), placeholder brackets with the correct length between joint axes, tibia rod with a ball foot. A starting point for designing printable parts
    1. Convert coordinates: GOIK has +Z towards the ground, CAD expects Z up (rotate 180 degrees around X, keeping the frame right-handed)
    1. Vendor STEP files are not committed (no licence or redistribution statement found). [CAD_FILES.md](CAD_FILES.md) lists where to download them and where to put them (`goik/cad/vendor/`, ignored by git). Record which file the mount data was measured on (name, date, SHA-256)
    1. Later: simple collision check between legs, and between legs and the body, using the same geometry
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
