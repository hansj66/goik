# Moving the joints in Fusion 360

STEP files don't carry joints into Fusion: Fusion imports geometry and colours only, so opening the exported STEP file
gives you the parts, but no joints. The CAD export therefore also writes `cad/out/<name>_joints.json`: which parts move
together (the body, and the coxa, femur and tibia links of each leg) and the axis of every servo. The Fusion script in
[cad/fusion/GOIK_Joints](../cad/fusion/GOIK_Joints/GOIK_Joints.py) turns that into rigid groups and revolute joints.

## Running the script

1. In Fusion: *Utilities > Add-Ins > Scripts and Add-Ins*, click **+** next to *My Scripts* and select the folder
   `goik/cad/fusion/GOIK_Joints` (once).
2. Select **GOIK_Joints** and click **Run**, either in a new, empty design or in a design where you already opened or
   imported `<name>.step`.
3. Pick `cad/out/<name>_joints.json`. The script imports `<name>.step` from the same folder (unless the design
   already contains it), groups the parts, adds a joint per servo (for example `leg0_femur (servo 2)`), and grounds the
   body. A message box reports how many joints it created.
4. Use *Assemble > Drive Joints* (or a Motion Study) to move a leg. Angles are relative to the rest pose the pod was
   exported in.

Run it on a design only once: a second run adds a second set of rigid groups and joints.

## How it works

* Each group of parts that move together becomes a rigid group. A servo belongs to the link its case is mounted on:
  the coxa servos to the body, the femur servo to the coxa link, the tibia servo to the femur link.
* Servos are found by their position, since Fusion gives every servo the same name (the STEP file stores the servo
  model once).
* Each joint is a revolute as-built joint around its servo's axis. The axes are drawn as construction lines in the
  sketch "GOIK joint axes".
* Fusion's API ignores a custom rotation axis on as-built joints (`JointDirections.CustomJointDirection`) and uses the
  joint geometry's Z axis instead. So each joint's geometry is made from its axis line
  (`JointGeometry.createByCurve`), whose Z axis follows the line, and the script checks that Fusion uses the intended
  axis. A mismatch is listed in the message box.
