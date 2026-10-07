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

"""Printable brackets for the CAD export.

All parts are generated from the joint frames in the GOIK assembly description and the servo's
mounting features in servos.json ("mounting"). Every servo is centred on its leg plane: its case
mid-plane goes through the joint. In a servo's "centred" coordinates (the placement matrix in the
assembly description) the case mid-plane is z = 0, the horn face is z = -mid_plane, and so on.

A link from joint j to joint j+1 is built in the link frame Lk = joint frame j rotated by the joint
angle: origin on joint j, Z along joint j's axis and X towards joint j+1. The horn (and the idler on
the other side of the servo) turns with this frame, so that is where a bracket attached to them lives.

Parts (all screws M2 through clearance holes, into the servo's own nuts and tapped horn):
    body plates   two plates clamping the coxa servo cases (one on each case face)
    coxa bracket  arms on the coxa horn and idler, side plates screwed to the femur servo case
    femur bracket two halves (horn side and idler side), each an arm on the femur horn or idler
                  with a pad screwed to the tibia servo case. The servos complete the box
    tibia         arms on the tibia horn and idler, a beam and a foot
"""

import math

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf

# Wall thicknesses and clearances (mm)
ARM = 3.0             # arms on the horn and the idler
STANDOFF = 2.0        # boss between the idler and its arm, so the body plate can be thicker
SIDE_PLATE = 4.0      # coxa bracket plates on the femur servo case
BODY_HORN_SIDE = 4.0  # body plate on the case face on the horn side of the coxa servos
BODY_IDLER_SIDE = 4.5
ARM_RADIUS = 14.0     # arm disc around the horn
CLEARANCE = 0.5       # between parts that move relative to each other
FIT = 0.2             # extra room where a servo has to slide into a printed part
PAD_MARGIN = 4.5      # material around screw holes
BODY_MARGIN = 6.0
TIBIA_BEAM = (12.0, 10.0)  # width (in the leg plane) and thickness of the tibia beam
FOOT_RADIUS = 7.0
# Only case holes this far (or further) from the axis are used for mounting a servo case, so the
# mounting parts stay clear of that servo's own horn, idler and the bracket that turns on them
MIN_HOLE_DISTANCE = 20.0


def to_np(m):
    return np.array(m, dtype=float)


def location(m):
    t = gp_Trsf()
    t.SetValues(*[float(m[r][c]) for r in range(3) for c in range(4)])
    return cq.Location(t)


def place(workplane, m):
    """The workplane's shape moved by the 4x4 matrix"""
    return cq.Workplane(obj=workplane.val().moved(location(m)))


def rz(angle_degrees):
    a = math.radians(angle_degrees)
    m = np.eye(4)
    m[0, 0], m[0, 1], m[1, 0], m[1, 1] = math.cos(a), -math.sin(a), math.sin(a), math.cos(a)
    return m


def apply(m, p):
    return (m @ np.array([p[0], p[1], p[2], 1.0]))[:3]


class Servo:
    """Mounting features of a servo model, in centred coordinates (case mid-plane at z = 0)"""

    def __init__(self, servo):
        m = servo["mounting"]
        self.offset = -m["mid_plane"]                       # servo frame z -> centred z: z + offset
        self.horn_face = self.offset
        self.case_faces = sorted(z + self.offset for z in m["case_faces"])  # [idler side, horn side]
        self.idler_face = m["idler_face"] + self.offset
        self.idler_boss = m["idler_boss_diameter"] / 2
        self.idler_screw = m["idler_screw_clearance"]
        self.horn_radius = servo["horn"]["diameter"] / 2
        self.horn_screws = m["horn_screws"]
        self.horn_centre = m["horn_centre_clearance"]
        self.case_holes = m["case_holes"]
        self.screw = m["screw_clearance"]
        c = servo["case"]
        self.case_x = (c["axis_from_end"] - c["length"], c["axis_from_end"])
        self.case_y = c["width"] / 2

    def mounting_holes(self):
        """Case holes far enough from the axis to be used for mounting the case"""
        return [h for h in self.case_holes if math.hypot(*h) >= MIN_HOLE_DISTANCE]

    def simple(self):
        """Case, horn and idler hub in centred coordinates, for the collision check"""
        lo, hi = self.case_faces
        case = cq.Workplane("XY").box(self.case_x[1] - self.case_x[0], 2 * self.case_y, hi - lo, centered=False) \
            .translate((self.case_x[0], -self.case_y, lo))
        horn = cq.Workplane("XY").workplane(offset=hi).circle(self.horn_radius).extrude(self.horn_face - hi)
        hub = cq.Workplane("XY").workplane(offset=self.idler_face).circle(self.idler_boss).extrude(lo - self.idler_face)
        return case.union(horn).union(hub)


def link_frame(joint):
    """Joint frame rotated by the joint angle: the frame the horn (and its bracket) turns with"""
    return to_np(joint["frame"]) @ rz(joint["angle"])


def horn_side(joint, lk):
    """+1 if the horn points along +Z of the link frame, -1 if it points along -Z"""
    return 1.0 if (np.linalg.inv(lk) @ to_np(joint["servo"]))[2, 2] > 0 else -1.0


def arms(servo, side, x_end, half_width, horn_holes):
    """Arms on the horn and the idler of the servo at the link frame origin, reaching x_end.

    The idler arm stands off the idler hub by FIT, so the servo slides in between the arms.
    Returns ((horn arm, idler arm), (z of the horn arm's inner face, z of the idler arm's inner face))
    """
    def outline(z0, thickness):
        disc = cq.Workplane("XY").workplane(offset=z0).circle(ARM_RADIUS).extrude(thickness)
        bar = cq.Workplane("XY").workplane(offset=z0).center(x_end / 2, 0).rect(x_end, 2 * half_width).extrude(thickness)
        return disc.union(bar)

    def span(a, b):
        return min(a, b), abs(b - a)

    horn_inner = side * servo.horn_face
    z0, t = span(horn_inner, horn_inner + side * ARM)
    horn_arm = outline(z0, t)
    for x, y in horn_holes:
        horn_arm = horn_arm.cut(cq.Workplane("XY").workplane(offset=z0 - 1).center(x, y).circle(servo.screw / 2).extrude(t + 2))
    horn_arm = horn_arm.cut(cq.Workplane("XY").workplane(offset=z0 - 1).circle(servo.horn_centre / 2).extrude(t + 2))

    idler_face = side * servo.idler_face - side * FIT
    idler_inner = idler_face - side * STANDOFF
    z0, t = span(idler_inner, idler_inner - side * ARM)
    idler_arm = outline(z0, t)
    s0, st = span(idler_face, idler_inner)
    idler_arm = idler_arm.union(cq.Workplane("XY").workplane(offset=s0).circle(servo.idler_boss + 1).extrude(st))
    lo, hi = span(idler_face, idler_inner - side * ARM)
    idler_arm = idler_arm.cut(cq.Workplane("XY").workplane(offset=lo - 1).circle(servo.idler_screw / 2).extrude(hi + 2))

    return (horn_arm, idler_arm), (horn_inner, idler_inner)


def horn_holes_in_link_frame(servo, joint, lk):
    """Positions of the horn screws in the link frame (the horn turns with it)"""
    to_link = np.linalg.inv(lk) @ to_np(joint["frame"]) @ rz(joint["angle"]) @ np.linalg.inv(to_np(joint["frame"])) @ to_np(joint["servo"])
    return [tuple(apply(to_link, (x, y, servo.horn_face))[:2]) for x, y in servo.horn_screws]


def case_holes_in_link_frame(servo, next_joint, lk):
    """The next servo's mounting holes (on both case faces) in the link frame, and its axis there"""
    to_link = np.linalg.inv(lk) @ to_np(next_joint["servo"])
    holes = [[apply(to_link, (x, y, z)) for x, y in servo.mounting_holes()] for z in servo.case_faces]
    axis = to_link[:3, 2]
    return holes, axis


def coxa_bracket(servo, joint, next_joint):
    """Arms on the coxa horn and idler, and side plates screwed to the femur servo's case faces"""
    lk = link_frame(joint)
    side = horn_side(joint, lk)
    faces, axis = case_holes_in_link_frame(servo, next_joint, lk)
    if abs(axis[1]) < 0.99:
        raise ValueError("coxa bracket: the femur axis must be horizontal and perpendicular to the coxa link")

    points = [p for face in faces for p in face]
    x0 = min(p[0] for p in points) - PAD_MARGIN
    x1 = max(p[0] for p in points) + PAD_MARGIN
    half_width = max(abs(p[1]) for p in points) + SIDE_PLATE

    (horn_arm, idler_arm), (horn_inner, idler_inner) = arms(servo, side, x1, half_width + FIT,
                                                             horn_holes_in_link_frame(servo, joint, lk))
    part = horn_arm.union(idler_arm)
    z_lo = min(horn_inner, idler_inner) - ARM
    z_hi = max(horn_inner, idler_inner) + ARM

    for face in faces:
        s = 1.0 if face[0][1] > 0 else -1.0
        y_face = face[0][1] + s * FIT
        y0 = min(y_face, y_face + s * SIDE_PLATE)
        plate = cq.Workplane("XY").box(x1 - x0, SIDE_PLATE, z_hi - z_lo, centered=False).translate((x0, y0, z_lo))
        for p in face:
            plate = plate.cut(cq.Workplane("XZ").workplane(offset=-(y0 - 1)).center(p[0], p[2])
                              .circle(servo.screw / 2).extrude(-(SIDE_PLATE + 2)))
        part = part.union(plate)
    return part, lk


def femur_bracket(servo, joint, next_joint):
    """Arms on the femur horn and idler, with pads screwed to the tibia servo's case faces"""
    lk = link_frame(joint)
    side = horn_side(joint, lk)
    faces, axis = case_holes_in_link_frame(servo, next_joint, lk)
    if abs(axis[2]) < 0.99:
        raise ValueError("femur bracket: the tibia axis must be parallel to the femur axis")

    points = [p for face in faces for p in face]
    x0 = min(p[0] for p in points) - PAD_MARGIN
    x1 = max(p[0] for p in points) + PAD_MARGIN
    half_width = max(abs(p[1]) for p in points) + PAD_MARGIN

    (horn_arm, idler_arm), (horn_inner, idler_inner) = arms(servo, side, x1, half_width,
                                                             horn_holes_in_link_frame(servo, joint, lk))
    halves = {"horn_side": horn_arm, "idler_side": idler_arm}
    for face in faces:
        z_face = face[0][2]
        horn = np.sign(z_face) == np.sign(horn_inner)
        arm_face = horn_inner if horn else idler_inner
        lo, hi = min(z_face, arm_face), max(z_face, arm_face)
        pad = cq.Workplane("XY").box(x1 - x0, 2 * half_width, hi - lo, centered=False).translate((x0, -half_width, lo))
        key = "horn_side" if horn else "idler_side"
        halves[key] = halves[key].union(pad)
    for key in halves:
        for p in faces[0]:
            halves[key] = halves[key].cut(cq.Workplane("XY").workplane(offset=-60).center(p[0], p[1])
                                          .circle(servo.screw / 2).extrude(120))
    return halves, lk


def tibia(servo, joint, foot):
    """Arms on the tibia horn and idler, joined into a beam that ends in a foot"""
    lk = link_frame(joint)
    side = horn_side(joint, lk)
    length = apply(np.linalg.inv(lk), foot)[0]
    start = math.hypot(servo.case_x[1], servo.case_y) + CLEARANCE + 2  # clear of the servo case for any angle
    width, thickness = TIBIA_BEAM

    (horn_arm, idler_arm), (horn_inner, idler_inner) = arms(servo, side, start + 8, width,
                                                             horn_holes_in_link_frame(servo, joint, lk))
    part = horn_arm.union(idler_arm)
    z_lo = min(horn_inner, idler_inner) - ARM
    z_hi = max(horn_inner, idler_inner) + ARM
    part = part.union(cq.Workplane("XY").box(8, 2 * width, z_hi - z_lo, centered=False).translate((start, -width, z_lo)))
    beam_end = length - FOOT_RADIUS
    part = part.union(cq.Workplane("XY").box(beam_end - start, width, thickness, centered=False)
                      .translate((start, -width / 2, -thickness / 2)))
    part = part.union(cq.Workplane(obj=cq.Solid.makeSphere(FOOT_RADIUS, cq.Vector(beam_end, 0, 0),
                                                            angleDegrees1=-90, angleDegrees2=90)))
    return part, lk


def body_plates(servo, assembly):
    """Plates on both case faces of the coxa servos, screwed into the case holes"""
    coxas = [to_np(leg["joints"][0]["servo"]) for leg in assembly["legs"]]
    links = [link_frame(leg["joints"][0]) for leg in assembly["legs"]]

    # Outline: around all coxa servo cases, with a rounded nose at each coxa so the coxa bracket
    # (its side plates start further out) can turn freely
    corners = []
    for m in coxas:
        for x in servo.case_x:
            for y in (-servo.case_y, servo.case_y):
                corners.append(tuple(apply(m, (x, y, 0))[:2]))
    hull = convex_hull(corners)

    radius = nose_radius(servo, assembly)
    plates = []
    for face_index, (thickness, cutout) in enumerate(((BODY_IDLER_SIDE, servo.idler_boss + STANDOFF + CLEARANCE),
                                                     (BODY_HORN_SIDE, servo.horn_radius + CLEARANCE))):
        z_face = servo.case_faces[face_index]
        z_world = [apply(m, (0, 0, z_face))[2] for m in coxas]
        if max(z_world) - min(z_world) > 1e-6:
            raise ValueError("body plates: all coxa servos must be at the same height")
        outward = z_world[0] - apply(coxas[0], (0, 0, 0))[2]
        z0 = z_world[0] if outward > 0 else z_world[0] - thickness

        plate = cq.Workplane("XY").workplane(offset=z0).polyline(hull).close().offset2D(BODY_MARGIN).extrude(thickness)
        for m, lk in zip(coxas, links):
            axis = apply(m, (0, 0, 0))
            # Everything around the front of the coxa axis (where the bracket turns, up to +/- 90
            # degrees), beyond the nose radius
            behind = SIDE_PLATE + servo.case_y + CLEARANCE
            front = cq.Workplane("XY").workplane(offset=z0 - 1).center(100 - behind, 0).rect(200, 400).extrude(thickness + 2)
            nose = front.cut(cq.Workplane("XY").workplane(offset=z0 - 2).circle(radius).extrude(thickness + 4))
            plate = plate.cut(place(nose, plane_frame(lk, axis)))
            plate = plate.cut(cq.Workplane("XY").workplane(offset=z0 - 1).center(axis[0], axis[1]).circle(cutout).extrude(thickness + 2))
            for x, y in servo.case_holes:
                p = apply(m, (x, y, 0))
                if math.hypot(x, y) > cutout + servo.screw:
                    plate = plate.cut(cq.Workplane("XY").workplane(offset=z0 - 1).center(p[0], p[1])
                                      .circle(servo.screw / 2).extrude(thickness + 2))
        plates.append(plate)
    return plates


def nose_radius(servo, assembly):
    """Body plates stay inside this radius around the front of a coxa axis: the coxa bracket's side
    plates (on the femur servo case) turn just outside it"""
    far_hole = max(-x for x, _ in servo.mounting_holes())
    radius = min(
        math.hypot(math.dist(leg["points"][0], leg["points"][1]) - far_hole - PAD_MARGIN, servo.case_y)
        for leg in assembly["legs"]) - 2 * CLEARANCE
    near_corner = math.hypot(servo.case_x[1], servo.case_y)
    if radius < near_corner:
        raise ValueError(f"the coxa is too short: the femur servo's mounting reaches within {radius:.1f} mm of the "
                         f"coxa axis, but the coxa servo case needs {near_corner:.1f} mm to turn")
    return radius


def plane_frame(lk, origin):
    """Rotation of the link frame about the vertical axis through origin (for 2D cutouts in XY)"""
    x = lk[:3, 0].copy()
    x[2] = 0
    x /= np.linalg.norm(x)
    m = np.eye(4)
    m[:3, 0] = x
    m[:3, 1] = [-x[1], x[0], 0]
    m[:3, 3] = [origin[0], origin[1], 0]
    return m


def convex_hull(points):
    points = sorted(set((round(x, 6), round(y, 6)) for x, y in points))

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


def build(assembly, servo_data):
    """All printable parts: [(name, part in world coordinates, part frame for printing)]"""
    servo = Servo(servo_data)
    parts = []
    for i, plate in enumerate(body_plates(servo, assembly)):
        parts.append((f"body_plate_{'idler' if i == 0 else 'horn'}_side", plate, np.eye(4)))

    for leg in assembly["legs"]:
        coxa, femur, tib = leg["joints"]
        prefix = f"leg{leg['index']}"
        part, lk = coxa_bracket(servo, coxa, femur)
        parts.append((f"{prefix}_coxa_bracket", place(part, lk), lk))
        halves, lk = femur_bracket(servo, femur, tib)
        for half, part in halves.items():
            parts.append((f"{prefix}_femur_bracket_{half}", place(part, lk), lk))
        part, lk = tibia(servo, tib, leg["points"][3])
        parts.append((f"{prefix}_tibia", place(part, lk), lk))
    return parts


def servo_solids(assembly, servo_data):
    """Simple servo solids (case, horn, idler hub) in world coordinates, for the collision check"""
    servo = Servo(servo_data)
    shape = servo.simple()
    return [(f"leg{leg['index']}_{j['name']}_servo", place(shape, to_np(j["servo"])))
            for leg in assembly["legs"] for j in leg["joints"]]
