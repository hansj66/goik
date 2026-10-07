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

"""Builds a STEP assembly (and STL) of a pod from a GOIK CAD assembly description.

The description is written by the simulator's export_cad command. Every servo is placed at its
joint, either as a vendor STEP model (when the file is in cad/vendor/ and its mount transform is
known, see servos.json) or as a simple case + horn. Servos are centred on their leg plane: the case
mid-plane goes through the joint.

For servo models with mounting features in servos.json, the pod gets printable parts (brackets.py):
body plates, coxa and femur brackets and tibias. Other models get a placeholder base plate, rods and
ball feet. All parts are checked for collisions in the exported (rest) pose. Units are mm, Z up.

Outputs: <name>.step (the assembly; every servo model is stored once and instanced), one STL per
printable part in <name>_stl/, each in its own frame lying on Z = 0 (servos are left out), and
<name>_joints.json: the parts that move together and the revolute joints between them, for the
Fusion 360 script in cad/fusion (STEP files don't carry joints into Fusion).

    python goik_cad.py ../cad/out/hexapod.json -o ../cad/out
"""

import argparse
import json
import math
import os
import sys

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf

import brackets

HERE = os.path.dirname(os.path.abspath(__file__))

ROD_DIAMETER = 8.0
FOOT_DIAMETER = 14.0

COLOURS = {
    "servo": cq.Color(0.15, 0.15, 0.15),
    "horn": cq.Color(0.8, 0.8, 0.8),
    "link": cq.Color(0.9, 0.55, 0.1),
    "foot": cq.Color(0.2, 0.2, 0.2),
    "plate": cq.Color(0.3, 0.45, 0.7),
}


def location(m):
    """cq.Location from a row major 4x4 matrix"""
    t = gp_Trsf()
    t.SetValues(m[0][0], m[0][1], m[0][2], m[0][3],
                m[1][0], m[1][1], m[1][2], m[1][3],
                m[2][0], m[2][1], m[2][2], m[2][3])
    return cq.Location(t)


def transform_point(m, p):
    return [sum(m[r][c] * p[c] for c in range(3)) + m[r][3] for r in range(3)]


def case_corners(servo):
    """The 8 corners of the servo case in the servo frame"""
    c, h = servo["case"], servo["horn"]
    xs = (c["axis_from_end"] - c["length"], c["axis_from_end"])
    ys = (-c["width"] / 2, c["width"] / 2)
    zs = (-h["thickness"] - c["depth"], -h["thickness"])
    return [(x, y, z) for x in xs for y in ys for z in zs]


def simple_servo(servo):
    """Case and horn in the servo frame. Returns (case, horn)"""
    c, h = servo["case"], servo["horn"]
    case = cq.Workplane("XY").box(c["length"], c["width"], c["depth"], centered=False).translate(
        (c["axis_from_end"] - c["length"], -c["width"] / 2, -h["thickness"] - c["depth"]))
    horn = cq.Workplane("XY").circle(h["diameter"] / 2).extrude(h["thickness"]).translate((0, 0, -h["thickness"]))
    return case, horn


def centring(servo):
    """Distance from the horn face to the case mid-plane: servos are placed with the mid-plane on the joint"""
    if "mounting" in servo:
        return -servo["mounting"]["mid_plane"]
    return servo["horn"]["thickness"] + servo["case"]["depth"] / 2


def servo_location(joint, servo):
    """Placement of the servo frame (origin on the horn face) for a centred servo"""
    m = np.array(joint["servo"], dtype=float) @ np.array(
        [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, centring(servo)], [0, 0, 0, 1]], dtype=float)
    return location(m.tolist())


def collisions(parts):
    """Pairs of parts that overlap (volume in mm^3), using bounding boxes to skip distant pairs"""
    shapes = [(name, part.val() if hasattr(part, "val") else part) for name, part in parts]
    boxes = [s.BoundingBox() for _, s in shapes]
    found = []
    for i in range(len(shapes)):
        for j in range(i + 1, len(shapes)):
            a, b = boxes[i], boxes[j]
            if a.xmax < b.xmin or b.xmax < a.xmin or a.ymax < b.ymin or b.ymax < a.ymin or a.zmax < b.zmin or b.zmax < a.zmin:
                continue
            volume = shapes[i][1].intersect(shapes[j][1]).Volume()
            if volume > 0.5:
                found.append((volume, shapes[i][0], shapes[j][0]))
    return sorted(found, reverse=True)


def print_frame(part, frame):
    """The part in its own frame (inverse of frame), lifted to lie on Z = 0"""
    m = np.linalg.inv(np.array(frame, dtype=float))
    shape = part.val().moved(location(m.tolist()))
    return cq.Workplane(obj=shape.moved(cq.Location(cq.Vector(0, 0, -shape.BoundingBox().zmin))))


def part_group(name):
    """The group of parts that move together: the body, or the coxa, femur or tibia link of a leg.

    A servo's case belongs to the link it is mounted on (the coxa servos to the body)."""
    if not name.startswith("leg"):
        return "body"
    leg, rest = name.split("_", 1)
    for servo, group in (("coxa_servo", "body"), ("femur_servo", f"{leg}_coxa"), ("tibia_servo", f"{leg}_femur")):
        if rest.startswith(servo):
            return group
    for link in ("coxa", "femur", "tibia"):
        if rest.startswith(link):
            return f"{leg}_{link}"
    if rest.startswith("foot"):
        return f"{leg}_tibia"
    raise ValueError(f"no group for part {name}")


def joints_description(assembly, servo, part_names):
    """Groups of parts that move together and the revolute joints between them (mm, Z up).

    Servos are listed with the origin of their placement too: a STEP file stores a servo model once,
    so CAD programs may not keep the individual names of the servos."""
    groups = {}
    for name in part_names:
        groups.setdefault(part_group(name), {"parts": [], "servos": []})["parts"].append(name)
    for leg in assembly["legs"]:
        for joint in leg["joints"]:
            name = f"leg{leg['index']}_{joint['name']}_servo_id{joint['servo_id']}"
            origin = (np.array(joint["servo"], dtype=float) @ np.array([0, 0, centring(servo), 1.0]))[:3]
            groups[part_group(name)]["servos"].append({"name": name, "origin": [round(v, 4) for v in origin]})

    joints = []
    for leg in assembly["legs"]:
        prefix = f"leg{leg['index']}"
        parents = ("body", f"{prefix}_coxa", f"{prefix}_femur")
        for joint, parent in zip(leg["joints"], parents):
            frame = np.array(joint["frame"], dtype=float)
            joints.append({
                "name": f"{prefix}_{joint['name']}",
                "servo_id": joint["servo_id"],
                "parent": parent,
                "child": f"{prefix}_{joint['name']}",
                "origin": [round(v, 4) for v in frame[:3, 3]],
                # Positive rotation around this axis is a positive joint angle in GOIK
                "axis": [round(v, 6) for v in frame[:3, 2]],
                "rest_angle": joint["angle"],
            })
    return {"format": "goik-joints", "version": 1, "name": assembly["name"], "units": "mm",
            "groups": [{"name": n, **g} for n, g in groups.items()], "joints": joints}


def vendor_servo(servo, vendor_dir):
    """The vendor model in the servo frame, or None if it is missing or its mount transform is unknown"""
    vendor = servo.get("vendor_step") or {}
    path = os.path.join(vendor_dir, vendor.get("file", ""))
    if not vendor.get("transform") or not os.path.isfile(path):
        return None
    # A vendor file usually contains several solids (case, horn, screws ...)
    model = cq.Compound.makeCompound(cq.importers.importStep(path).vals())
    return model.moved(location(vendor["transform"]))


def rod(p, q, diameter):
    direction = cq.Vector(q[0] - p[0], q[1] - p[1], q[2] - p[2])
    if direction.Length < 1e-6:
        return None
    return cq.Workplane(obj=cq.Solid.makeCylinder(diameter / 2, direction.Length, cq.Vector(*p), direction))


def convex_hull(points):
    """2D convex hull, counter clockwise (Andrew's monotone chain)"""
    points = sorted(set(points))
    if len(points) < 3:
        return points

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def base_plate(assembly, servo):
    """Plate on the case side of the coxa servos, outlined around them plus a margin"""
    corners = []
    horn_up = 0
    for leg in assembly["legs"]:
        m = (np.array(leg["joints"][0]["servo"]) @ np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, centring(servo)], [0, 0, 0, 1]])).tolist()
        corners += [transform_point(m, c) for c in case_corners(servo)]
        horn_up += 1 if m[2][2] > 0 else -1

    plate = assembly["plate"]
    hull = convex_hull([(round(x, 6), round(y, 6)) for x, y, _ in corners])
    zs = [z for _, _, z in corners]
    # With the horns facing down, the cases are above the horns: put the plate on top of them
    z0 = min(zs) - plate["thickness"] if horn_up > 0 else max(zs)
    return (cq.Workplane("XY").workplane(offset=z0).polyline(hull).close()
            .offset2D(plate["margin"]).extrude(plate["thickness"]))


def build(assembly, servos, vendor_dir):
    model = assembly["servo_model"]
    if model not in servos:
        sys.exit(f"No geometry for servo model '{model}' in servos.json")
    servo = servos[model]

    notes = []
    vendor = vendor_servo(servo, vendor_dir)
    if vendor is None:
        case, horn = simple_servo(servo)
        notes.append(f"{model}: using the simple case + horn (no vendor model with a known mount transform)")
    if not servo.get("verified"):
        notes.append(f"{model}: the servo geometry is not verified against a drawing. Check servos.json")

    if vendor is not None:
        # One object for all servos: the STEP export stores a part once if the same object is reused
        vendor = cq.Workplane(obj=vendor)

    assy = cq.Assembly(name=assembly["name"])
    # Parts that are meant to be printed (name, shape in assembly coordinates, frame for printing)
    printable = []

    for leg in assembly["legs"]:
        for joint in leg["joints"]:
            loc = servo_location(joint, servo)
            name = f"leg{leg['index']}_{joint['name']}_servo_id{joint['servo_id']}"
            if vendor is not None:
                assy.add(vendor, name=name, loc=loc, color=COLOURS["servo"])
            else:
                assy.add(case, name=name, loc=loc, color=COLOURS["servo"])
                assy.add(horn, name=name + "_horn", loc=loc, color=COLOURS["horn"])

    if "mounting" in servo:
        try:
            parts = brackets.build(assembly, servo)
        except ValueError as e:
            sys.exit(f"Can't build printable brackets for this pod: {e}")
        for name, part, frame in parts:
            assy.add(part, name=name, color=COLOURS["plate" if name.startswith("body") else "link"])
            printable.append((name, part, frame))
    else:
        notes.append(f"{model}: no mounting features in servos.json, so the parts are placeholders, not printable brackets")
        plate = base_plate(assembly, servo)
        assy.add(plate, name="base_plate", color=COLOURS["plate"])
        printable.append(("base_plate", plate, np.eye(4)))
        for leg in assembly["legs"]:
            prefix = f"leg{leg['index']}"
            points = leg["points"]
            for i, link in enumerate(("coxa", "femur", "tibia")):
                r = rod(points[i], points[i + 1], ROD_DIAMETER)
                if r is not None:
                    assy.add(r, name=f"{prefix}_{link}_link", color=COLOURS["link"])
                    printable.append((f"{prefix}_{link}_link", r, np.eye(4)))
            foot = cq.Workplane(obj=cq.Solid.makeSphere(FOOT_DIAMETER / 2, cq.Vector(*points[3]), angleDegrees1=-90, angleDegrees2=90))
            assy.add(foot, name=f"{prefix}_foot", color=COLOURS["foot"])
            printable.append((f"{prefix}_foot", foot, np.eye(4)))

    return assy, printable, notes


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("assembly", help="CAD assembly description written by the simulator (export_cad)")
    parser.add_argument("-o", "--output", default=None, help="output folder (default: next to the description)")
    parser.add_argument("--servos", default=os.path.join(HERE, "servos.json"), help="servo geometry")
    parser.add_argument("--vendor", default=os.path.join(HERE, "vendor"), help="folder with vendor STEP files")
    parser.add_argument("--no-stl", action="store_true", help="don't write STL files for the printable parts")
    parser.add_argument("--assembly-stl", action="store_true", help="also write the whole assembly (servos included) as one STL")
    parser.add_argument("--no-check", action="store_true", help="skip the collision check")
    args = parser.parse_args()

    with open(args.assembly, encoding="utf-8") as f:
        assembly = json.load(f)
    if assembly.get("format") != "goik-cad-assembly" or assembly.get("version") != 1:
        sys.exit(f"{args.assembly} is not a GOIK CAD assembly description (version 1)")
    with open(args.servos, encoding="utf-8") as f:
        servos = json.load(f)

    output = args.output or os.path.dirname(os.path.abspath(args.assembly))
    os.makedirs(output, exist_ok=True)
    base = os.path.join(output, assembly["name"])

    assy, printable, notes = build(assembly, servos, args.vendor)
    for note in notes:
        print("Note:", note)

    assy.export(base + ".step")
    print("Wrote", base + ".step")

    servo = servos[assembly["servo_model"]]
    part_names = [name for name, _, _ in printable]
    if vendor_servo(servo, args.vendor) is None:
        # The simple servo model is a case and a horn per servo
        part_names += [f"leg{leg['index']}_{j['name']}_servo_id{j['servo_id']}{suffix}"
                       for leg in assembly["legs"] for j in leg["joints"] for suffix in ("", "_horn")]
    with open(base + "_joints.json", "w", encoding="utf-8") as f:
        json.dump(joints_description(assembly, servo, part_names), f, indent=2)
    print("Wrote", base + "_joints.json")
    if not args.no_stl:
        folder = base + "_stl"
        os.makedirs(folder, exist_ok=True)
        for name, part, frame in printable:
            cq.exporters.export(print_frame(part, frame), os.path.join(folder, name + ".stl"))
        print(f"Wrote {len(printable)} printable parts to {folder}")

    if not args.no_check and "mounting" in servos[assembly["servo_model"]]:
        parts = brackets.servo_solids(assembly, servos[assembly["servo_model"]]) + [(n, p) for n, p, _ in printable]
        found = collisions(parts)
        if found:
            print(f"Collisions in the rest pose ({len(found)}):")
            for volume, a, b in found:
                print(f"    {a} and {b}: {volume:.1f} mm^3")
        else:
            print("No collisions in the rest pose")
    if args.assembly_stl:
        cq.exporters.export(assy.toCompound(), base + ".stl")
        print("Wrote", base + ".stl")


if __name__ == "__main__":
    main()
