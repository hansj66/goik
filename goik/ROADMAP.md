# Roadmap

## R & D / TODO

1. Printable brackets (see [docs/cad-export.md](docs/cad-export.md))
    1. Decisions: our own brackets for every supported servo model (no ROBOTIS frames or other third party brackets). No heat-set inserts: screws go through clearance holes or into slightly undersized holes in the plastic. Validate in Fusion 360 as early as possible
    1. Mounting features for the XL-320 (it still gets placeholder parts), and teach `cad/tools.py measure` to report mounting features (they were found by surveying the vendor models' circular and flat faces)
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
    1. Test print: screw access, print orientation and supports per part, part strength (tibia beam 12 x 10 mm)
    1. Per part print orientation in the STL export (currently each part lies in its own frame on Z = 0)
    1. Open question: printer, material and bed size (FDM with PLA or PETG?). The body plates of pod 6 are about 230 mm across
1. CAD export of [twisted joints](docs/designing-a-pod.md#twisted-joints): femur and tibia twists are exported (see [docs/cad-export.md](docs/cad-export.md#twisted-joints)), coxa twists not yet
    1. Validate twisted femur and tibia brackets in Fusion 360, and with a test print: the tilted side plates of the coxa bracket and the wedge pads of the femur bracket
    1. Coxa twists: the coxa servos sit tilted, so the flat body plates need an angled mounting block per servo
    1. Check twisted brackets over the joint ranges, not only in the rest pose: a twisted femur or tibia swings in a tilted plane, closer to the bracket arms of the joint before it (part of the "Validate a design" item below)
1. Validate a design against the servos it will use, while designing it (not only when exporting to CAD)
    1. Today: the CAD export checks the rest pose for collisions and stops if the coxa is too short for the servos. Nothing warns while a pod is being designed or walked in the simulator
    1. A `validate` command in the shell (and later live warnings in the views) for the selected servo model. Needs the servo dimensions in Go (case, horn axis position, mounting faces), shared with `cad/servos.json` so there is one source of truth
    1. Fit: the coxa servos fit around the body without overlapping, the coxa is long enough for the femur servo to clear the coxa servo at every coxa angle, the femur is long enough for the tibia servo (the checks behind example pods 6 and 7, made general)
    1. Collisions over the joint ranges: sweep each joint through its range (or the servo mapping's soft limits), and check servos and brackets against each other, the body and the ground. The CAD export already turns neighbouring legs' coxas towards each other; the femur and tibia ranges, and the ground, are still to do
    1. Collisions between neighbouring legs while walking: run the gait engine through the gaits and speeds and check legs against each other
    1. Joint angles: every angle the gait engine and the body pose use stays within the servo's range and the soft limits (AX-12A and XL-320: 300 degrees, STS3215: 360)
    1. Load: static torque per joint with the pod's weight (servos, brackets, electronics) on the fewest legs a gait stands on, compared with the servo's stall torque, with a safety margin
    1. Report what fails and by how much (for example "coxa 8 mm too short for the STS3215"), and suggest the smallest change that fixes it
1. Interactive pod designer: compose a new bot by pointing and clicking in the simulator window, with text input for exact values. Plan and decisions in [docs/pod-designer.md](docs/pod-designer.md). The shell commands that exist today (`design`, `stance`) are in [docs/designing-a-pod.md](docs/designing-a-pod.md)
    1. Design mode, body: upgrade Ebitengine and add debugui, canvas with zoom and pan, outline editing, legs with live mirroring, property panel, undo / redo. Draw the outline in the XY and isometric views
    1. Legs: side view of the selected leg, drag the body height and the feet (sets the stance, see `stance`), "try it"
    1. Validation overlay: servo case footprints at the coxa mounts and coxa clearance, from the design validation item above
    1. CAD: the outline becomes the body plate shape in the CAD export, with a check that the servos fit inside it
1. Improve visualization
    1. Footfall diagram (scrolling swing/stance timeline per leg) instead of the static gait pattern table, which shows the target gait and is wrong during transitions
    1. Phase dials: each leg's current phase compared to the phase its gait wants (shows legs re-timing during gait transitions)
    1. Support polygon, centre of gravity and stability margin (green to red) in the XY view
    1. Reach circles around the neutral foot positions. Colour stance feet as they approach the edge of their reach
    1. Camera controls: zoom and pan (the views are drawn at a fixed 1 px = 1 mm on a picture 1024 px along its shorter side, then scaled to the window), rotatable isometric view with a ground plane
    1. Pause and single-step (e.g. space / period key) for inspecting transitions frame by frame
    1. Joint angle panel: each servo's angle against its limits from the servo mapping, highlighting joints near or past a limit
    1. Fix view colours: alpha is 1 instead of 255 (e.g. RGBA{255, 0, 0, 1}), which is invalid for Ebiten's premultiplied colours
    1. Scale bar / units on the axes
1. Implement metachronal gait (for centipede type robots)
1. Gamepad control: the simulator can be driven with a gamepad (see [docs/simulator.md](docs/simulator.md#gamepad)), using the input independent `control` package. Still to do:
    1. Overlord input: Linux joystick device `/dev/input/js0` (Bluetooth gamepad, as in the Overlord remote control example), feeding the same `control` package
    1. Watchdog on Overlord: halt if the input goes quiet (out of range), not only on a disconnect
    1. Pose mode while a button is held: the sticks shift the body (x, y) and turn it (yaw) instead of walking
    1. Configurable button and axis mapping (gamepads differ), and the stick and button state shown in the simulator
