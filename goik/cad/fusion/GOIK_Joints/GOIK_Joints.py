# Copyright 2025 Hans Jørgen Grimstad
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Fusion 360 script: imports a GOIK CAD export and recreates its joints.

STEP files don't carry joints into Fusion, so the GOIK export writes them to <name>_joints.json
next to <name>.step. Run this script and select the joints file. It
1. imports <name>.step (unless the design already contains it, for example because it was opened
   or imported by hand),
2. makes a rigid group of every set of parts that move together (the body, and the coxa, femur
   and tibia links of each leg),
3. adds a revolute as-built joint for every servo, on the joint's axis (sketch "GOIK joint axes"),
4. grounds the body.
Then use Drive Joints (or Motion Study) to move the legs. Angles are relative to the rest pose
the pod was exported in.
"""

import json
import os
import traceback

import adsk.core
import adsk.fusion

MM = 0.1               # Fusion's API uses cm
TOLERANCE = 0.02       # cm, when finding servos by their position


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        dialog = ui.createFileDialog()
        dialog.title = "Select a GOIK joints file (<name>_joints.json)"
        dialog.filter = "GOIK joints (*_joints.json);;All files (*.*)"
        if dialog.showOpen() != adsk.core.DialogResults.DialogOK:
            return
        with open(dialog.filename, encoding="utf-8") as f:
            data = json.load(f)
        if data.get("format") != "goik-joints" or data.get("version") != 1:
            ui.messageBox(f"{dialog.filename} is not a GOIK joints file (version 1)")
            return
        step = dialog.filename[:-len("_joints.json")] + ".step"
        if not os.path.isfile(step):
            ui.messageBox(f"Can't find the STEP file next to the joints file:\n{step}")
            return

        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Open (or create) a Fusion design first")
            return
        root = design.rootComponent

        occurrences = existing_parts(root, data)
        imported = not occurrences
        if imported:
            occurrences = import_step(app, root, step)
        warnings = []
        groups = find_groups(data, occurrences, warnings)
        make_rigid_groups(root, groups)
        made = make_joints(root, data, groups, warnings)

        body = groups.get("body")
        if body:
            body[0].isGrounded = True

        source = f"Imported {os.path.basename(step)}" if imported else "Used the parts already in the design"
        message = f"{source} and created {made} of {len(data['joints'])} joints."
        if warnings:
            message += "\n\nWarnings:\n" + "\n".join(warnings[:20])
            if len(warnings) > 20:
                message += f"\n... and {len(warnings) - 20} more"
        ui.messageBox(message)
    except Exception:
        if ui:
            ui.messageBox("GOIK joints failed:\n{}".format(traceback.format_exc()))


def import_step(app, root, step):
    """Imports the STEP file into the root component. Returns the new occurrences (root context)"""
    before = set(o.fullPathName for o in root.allOccurrences)
    options = app.importManager.createSTEPImportOptions(step)
    app.importManager.importToTarget(options, root)
    return [o for o in root.allOccurrences if o.fullPathName not in before]


def existing_parts(root, data):
    """All occurrences, if the design already contains the parts of this export, else []"""
    names = set(part for group in data["groups"] for part in group["parts"])
    occurrences = list(root.allOccurrences)
    if any(base_name(o) in names for o in occurrences):
        return occurrences
    return []


def base_name(occurrence):
    """Occurrence name without Fusion's ':n' suffix"""
    return occurrence.name.rsplit(":", 1)[0]


def translation(occurrence):
    t = occurrence.transform2.translation
    return (t.x, t.y, t.z)


def find_groups(data, occurrences, warnings):
    """{group name: [occurrences]}. Parts are found by name, servos by their position (a STEP file
    stores a servo model once, so every servo may get the same name)"""
    by_name = {}
    for o in occurrences:
        by_name.setdefault(base_name(o), []).append(o)

    groups = {}
    used = set()
    for group in data["groups"]:
        members = []
        for part in group["parts"]:
            found = by_name.get(part)
            if found:
                members.append(found[0])
                used.add(found[0].fullPathName)
            else:
                warnings.append(f"part {part} not found")
        groups[group["name"]] = members

    for group in data["groups"]:
        for servo in group["servos"]:
            target = tuple(v * MM for v in servo["origin"])
            match = None
            for o in occurrences:
                if o.fullPathName in used:
                    continue
                if all(abs(a - b) < TOLERANCE for a, b in zip(translation(o), target)):
                    match = o
                    break
            if match is None:
                warnings.append(f"servo {servo['name']} not found at {servo['origin']} mm")
                continue
            used.add(match.fullPathName)
            groups[group["name"]].append(match)
    return groups


def make_rigid_groups(root, groups):
    for name, members in groups.items():
        if len(members) < 2:
            continue
        collection = adsk.core.ObjectCollection.create()
        for o in members:
            collection.add(o)
        group = root.rigidGroups.add(collection, False)
        group.name = name


def make_joints(root, data, groups, warnings):
    """Revolute as-built joints between the groups, around axes drawn in a sketch.

    as-built joints ignore JointDirections.CustomJointDirection and turn around the Z axis of
    their joint geometry instead. So the geometry is made from the axis line itself (its Z axis
    follows the line), and the joint turns around that Z axis. Every joint's axis is checked."""
    sketch = root.sketches.add(root.xYConstructionPlane)
    sketch.name = "GOIK joint axes"
    made = 0
    for j in data["joints"]:
        parent, child = groups.get(j["parent"]), groups.get(j["child"])
        if not parent or not child:
            warnings.append(f"joint {j['name']}: missing parts ({j['parent']} or {j['child']})")
            continue

        origin = [v * MM for v in j["origin"]]
        tip = [o + a * 2.0 for o, a in zip(origin, j["axis"])]  # 2 cm along the axis
        axis = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(*origin),
                                                              adsk.core.Point3D.create(*tip))
        axis.isConstruction = True

        geometry = adsk.fusion.JointGeometry.createByCurve(axis, adsk.fusion.JointKeyPointTypes.StartKeyPoint)
        joint_input = root.asBuiltJoints.createInput(child[0], parent[0], geometry)
        joint_input.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        joint = root.asBuiltJoints.add(joint_input)
        joint.name = f"{j['name']} (servo {j['servo_id']})"
        made += 1

        actual = rotation_axis(joint)
        if actual is not None and abs(sum(a * b for a, b in zip(actual, j["axis"]))) < 0.999:
            warnings.append(f"joint {j['name']}: Fusion turns it around {tuple(round(v, 3) for v in actual)}, "
                            f"expected {tuple(j['axis'])}")
    return made


def rotation_axis(joint):
    """The direction Fusion rotates the joint around, or None if it can't be read"""
    try:
        v = joint.jointMotion.rotationAxisVector
        length = (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5
        return (v.x / length, v.y / length, v.z / length)
    except Exception:
        return None
