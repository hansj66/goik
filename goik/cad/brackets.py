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

Joints can be twisted (see robot/twist.go): the femur axis turned about the coxa link, the tibia axis about the
femur link. The arms still turn in the link frame, but the next servo is turned about the link (X). The parts screwed
to the next servo's case (the coxa bracket's side plates, the femur bracket's pads) are built in the twisted frame
Lt = Lk Rx(twist), where that servo sits as it does without a twist, then tilted into place and trimmed to the arms.
The pads become wedges between an arm and the tilted case face. A twist is only possible as long as the next servo
stays clear of the arms (check_fit).

Mounting features can differ per model: the case holes may differ between the horn side and the
idler side (the STS3215), the idler may be a hub with one centre screw (AX-12A) or a disc with a
screw pattern like the horn (STS3215), and the arms can stand off the horn and idler, so that plates
fit between the case and a turning arm when the horn sits close to the case.

Parts (screws through clearance holes, into the servo's own nuts, holes and tapped horn):
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
STANDOFF = 2.0        # default boss between the idler and its arm, so the body plate can be thicker
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


def rx(angle_degrees):
    a = math.radians(angle_degrees)
    m = np.eye(4)
    m[1, 1], m[1, 2], m[2, 1], m[2, 2] = math.cos(a), -math.sin(a), math.sin(a), math.cos(a)
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
        self.idler_screws = m.get("idler_screws", [])        # screw pattern on the idler, if any
        self.idler_standoff = m.get("idler_standoff", STANDOFF)
        self.horn_radius = servo["horn"]["diameter"] / 2
        self.horn_screws = m["horn_screws"]
        self.horn_centre = m["horn_centre_clearance"]
        self.horn_standoff = m.get("horn_standoff", 0.0)
        self.screw = m["screw_clearance"]
        self.horn_screw = m.get("horn_screw_clearance", self.screw)
        # Case holes per case face: [idler side, horn side], like case_faces
        holes = m["case_holes"]
        if isinstance(holes, dict):
            self.case_holes = [holes["idler_side"], holes["horn_side"]]
        else:
            self.case_holes = [holes, holes]
        c = servo["case"]
        self.case_x = (c["axis_from_end"] - c["length"], c["axis_from_end"])
        self.case_y = c["width"] / 2

    def mounting_holes(self, face):
        """Holes in a case face (0: idler side, 1: horn side) far enough from the axis to be used for
        mounting the case from another link"""
        return [h for h in self.case_holes[face] if math.hypot(*h) >= MIN_HOLE_DISTANCE]

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


def arm_inner_faces(servo, side):
    """z (link frame) of the inner faces of the arms on the horn and on the idler of the servo at the link frame
    origin. The next servo sits between them"""
    horn_inner = side * servo.horn_face + side * servo.horn_standoff
    idler_inner = side * servo.idler_face - side * FIT - side * servo.idler_standoff
    return horn_inner, idler_inner


def joint_twist(servo, next_joint, lk, untwisted, name):
    """The twist (degrees) of the next joint's axis about the link frame's X axis. untwisted is the axis's direction
    without a twist: "y" (the femur axis in the coxa link frame) or "z" (the tibia axis in the femur link frame)"""
    _, axis = case_holes_in_link_frame(servo, next_joint, lk)
    if abs(axis[0]) > 1e-6:
        raise ValueError(f"the {name} axis must be square to the link before it")
    if untwisted == "y":
        a = math.degrees(math.atan2(axis[2], axis[1]))
    else:
        a = math.degrees(math.atan2(-axis[1], axis[2]))
    # The axis points either way (inverted servos)
    while a > 90:
        a -= 180
    while a <= -90:
        a += 180
    return 0.0 if abs(a) < 1e-9 else a


def y_span(part, z_a, z_b):
    """(lowest, highest) y of the part between the heights z_a and z_b"""
    slab = cq.Workplane("XY").box(2000, 2000, abs(z_b - z_a), centered=False).translate((-1000, -1000, min(z_a, z_b)))
    box = part.intersect(slab).val().BoundingBox()
    return box.ymin, box.ymax


def servo_points(servo):
    """Points on the surface of a servo's case, horn and idler hub (centred coordinates), for check_fit"""
    lo, hi = servo.case_faces
    xs = np.linspace(servo.case_x[0], servo.case_x[1], 26)
    ys = np.linspace(-servo.case_y, servo.case_y, 9)
    zs = np.linspace(lo, hi, 9)
    points = [(x, y, z) for x in xs for y in (-servo.case_y, servo.case_y) for z in zs]
    points += [(x, y, z) for x in xs for y in ys for z in (lo, hi)]
    points += [(x, y, z) for x in servo.case_x for y in ys for z in zs]
    for a in np.linspace(0, 2 * math.pi, 72, endpoint=False):
        for z in (hi, servo.horn_face):
            points.append((servo.horn_radius * math.cos(a), servo.horn_radius * math.sin(a), z))
        for z in (lo, servo.idler_face):
            points.append((servo.idler_boss * math.cos(a), servo.idler_boss * math.sin(a), z))
    return np.array(points)


def check_fit(servo, joint, next_joint, lk, twist, x_end, name, bracket):
    """The next servo (turned by its twist) must stay CLEARANCE inside the arms of the bracket on the servo at the
    link frame origin, where the arms are (up to x_end)"""
    inner = sorted(arm_inner_faces(servo, horn_side(joint, lk)))
    tilt = rx(twist)
    # The next servo in the twisted frame, where it sits as without a twist
    untwisted = np.linalg.inv(lk @ tilt) @ to_np(next_joint["servo"])
    points = np.array([apply(untwisted, p) for p in servo_points(servo)])

    def fits(angle):
        r = rx(angle)[:3, :3]
        turned = points @ r.T
        near = turned[turned[:, 0] <= x_end + CLEARANCE]
        return len(near) == 0 or (near[:, 2].min() >= inner[0] + CLEARANCE and near[:, 2].max() <= inner[1] - CLEARANCE)

    if fits(twist):
        return
    largest, step = 0.0, 0.5 if twist > 0 else -0.5
    while abs(largest + step) <= abs(twist) and fits(largest + step):
        largest += step
    raise ValueError(f"the {name} twist of {twist:.1f} degrees turns the {name} servo into the arms of the {bracket} bracket. "
                     f"With this servo, at most {abs(largest):.1f} degrees fit")


def arms(servo, side, x_end, width, horn_holes, idler_holes, taper=None):
    """Arms on the horn and the idler of the servo at the link frame origin, reaching x_end.

    width is the arms' half width, or ((y0, y1) of the horn arm, (y0, y1) of the idler arm) for arms that are
    as wide as what they hold (twisted joints). taper = (x_start, half width at the servo) makes such arms only
    that wide at the servo, widening to their full width at x_start: neighbouring legs are closest near the body.
    The arms stand off the horn and idler on bosses (if the model asks for it), and the idler arm
    stands off the idler by FIT, so the servo slides in between the arms.
    Returns ((horn arm, idler arm), (z of the horn arm's inner face, z of the idler arm's inner face))
    """
    if isinstance(width, tuple):
        horn_span, idler_span = width
    else:
        horn_span = idler_span = (-width, width)

    def outline(z0, thickness, ys):
        disc = cq.Workplane("XY").workplane(offset=z0).circle(ARM_RADIUS).extrude(thickness)
        if taper is None:
            bar = cq.Workplane("XY").workplane(offset=z0).center(x_end / 2, (ys[0] + ys[1]) / 2) \
                .rect(x_end, ys[1] - ys[0]).extrude(thickness)
        else:
            x_start, base = taper
            bar = cq.Workplane("XY").workplane(offset=z0).polyline(
                [(0, -base), (x_start, ys[0]), (x_end, ys[0]), (x_end, ys[1]), (x_start, ys[1]), (0, base)]) \
                .close().extrude(thickness)
        return disc.union(bar)

    def span(a, b):
        """(start, length) of the range between a and b"""
        return min(a, b), abs(b - a)

    horn_face = side * servo.horn_face
    horn_inner, idler_inner = arm_inner_faces(servo, side)
    z0, t = span(horn_inner, horn_inner + side * ARM)
    horn_arm = outline(z0, t, horn_span)
    if servo.horn_standoff > 0:
        s0, st = span(horn_face, horn_inner)
        horn_arm = horn_arm.union(cq.Workplane("XY").workplane(offset=s0).circle(servo.horn_radius + 1).extrude(st))
    lo, length = span(horn_face, horn_inner + side * ARM)
    for x, y in horn_holes:
        horn_arm = horn_arm.cut(cq.Workplane("XY").workplane(offset=lo - 1).center(x, y).circle(servo.horn_screw / 2).extrude(length + 2))
    horn_arm = horn_arm.cut(cq.Workplane("XY").workplane(offset=lo - 1).circle(servo.horn_centre / 2).extrude(length + 2))

    idler_face = side * servo.idler_face - side * FIT
    z0, t = span(idler_inner, idler_inner - side * ARM)
    idler_arm = outline(z0, t, idler_span)
    if servo.idler_standoff > 0:
        s0, st = span(idler_face, idler_inner)
        idler_arm = idler_arm.union(cq.Workplane("XY").workplane(offset=s0).circle(servo.idler_boss + 1).extrude(st))
    lo, length = span(idler_face, idler_inner - side * ARM)
    idler_arm = idler_arm.cut(cq.Workplane("XY").workplane(offset=lo - 1).circle(servo.idler_screw / 2).extrude(length + 2))
    for x, y in idler_holes:
        idler_arm = idler_arm.cut(cq.Workplane("XY").workplane(offset=lo - 1).center(x, y).circle(servo.horn_screw / 2).extrude(length + 2))

    return (horn_arm, idler_arm), (horn_inner, idler_inner)


def screw_holes_in_link_frame(servo, joint, lk):
    """Positions of the horn and idler screws in the link frame (the horn and idler turn with it)"""
    to_link = np.linalg.inv(lk) @ to_np(joint["frame"]) @ rz(joint["angle"]) @ np.linalg.inv(to_np(joint["frame"])) @ to_np(joint["servo"])
    horn = [tuple(apply(to_link, (x, y, servo.horn_face))[:2]) for x, y in servo.horn_screws]
    idler = [tuple(apply(to_link, (x, y, servo.idler_face))[:2]) for x, y in servo.idler_screws]
    return horn, idler


def case_holes_in_link_frame(servo, next_joint, lk):
    """The next servo's mounting holes in the link frame, one list per case face, and its axis there"""
    to_link = np.linalg.inv(lk) @ to_np(next_joint["servo"])
    holes = [[apply(to_link, (x, y, z)) for x, y in servo.mounting_holes(face)]
             for face, z in enumerate(servo.case_faces)]
    axis = to_link[:3, 2]
    return holes, axis


def coxa_bracket(servo, joint, next_joint):
    """Arms on the coxa horn and idler, and side plates screwed to the femur servo's case faces.

    The side plates are built in the twisted frame (where the femur servo sits as without a femur twist), tilted into
    place and trimmed to the arms. The arms are as wide as the tilted plates"""
    lk = link_frame(joint)
    side = horn_side(joint, lk)
    twist = joint_twist(servo, next_joint, lk, "y", "femur")
    tilt = rx(twist)
    faces, _ = case_holes_in_link_frame(servo, next_joint, lk @ tilt)

    points = [p for face in faces for p in face]
    x0 = min(p[0] for p in points) - PAD_MARGIN
    x1 = max(p[0] for p in points) + PAD_MARGIN
    check_fit(servo, joint, next_joint, lk, twist, x1, "femur", "coxa")

    horn_inner, idler_inner = arm_inner_faces(servo, side)
    z_lo = min(horn_inner, idler_inner) - ARM
    z_hi = max(horn_inner, idler_inner) + ARM
    between_arms = cq.Workplane("XY").box(x1 - x0, 400, z_hi - z_lo, centered=False).translate((x0, -200, z_lo))

    plates = None
    for face in faces:
        s = 1.0 if face[0][1] > 0 else -1.0
        y_face = face[0][1] + s * FIT
        y0 = min(y_face, y_face + s * SIDE_PLATE)
        plate = cq.Workplane("XY").box(x1 - x0, SIDE_PLATE, 400, centered=False).translate((x0, y0, -200))
        for p in face:
            plate = plate.cut(cq.Workplane("XZ").workplane(offset=-(y0 - 1)).center(p[0], p[2])
                              .circle(servo.screw / 2).extrude(-(SIDE_PLATE + 2)))
        plate = place(plate, tilt).intersect(between_arms)
        plates = plate if plates is None else plates.union(plate)

    # Each arm as wide as the plates where they meet it (tilted plates meet the two arms at different places)
    spans = tuple(y_span(plates, z, z + d * ARM) for z, d in ((horn_inner, side), (idler_inner, -side)))
    untwisted = max(abs(face[0][1]) for face in faces) + FIT + SIDE_PLATE
    (horn_arm, idler_arm), _ = arms(servo, side, x1, spans, *screw_holes_in_link_frame(servo, joint, lk),
                                    taper=(x0, untwisted))
    return horn_arm.union(idler_arm).union(plates), lk


def femur_bracket(servo, joint, next_joint):
    """Arms on the femur horn and idler, with pads screwed to the tibia servo's case faces.

    The pads are built in the twisted frame (where the tibia servo sits as without a tibia twist), from the case face
    outwards, tilted into place and trimmed at the arm: with a tibia twist they are wedges"""
    lk = link_frame(joint)
    side = horn_side(joint, lk)
    twist = joint_twist(servo, next_joint, lk, "z", "tibia")
    tilt = rx(twist)
    faces, _ = case_holes_in_link_frame(servo, next_joint, lk @ tilt)

    points = [p for face in faces for p in face]
    x0 = min(p[0] for p in points) - PAD_MARGIN
    x1 = max(p[0] for p in points) + PAD_MARGIN
    pad_half_width = max(abs(p[1]) for p in points) + PAD_MARGIN
    check_fit(servo, joint, next_joint, lk, twist, x1, "tibia", "femur")

    horn_inner, idler_inner = arm_inner_faces(servo, side)
    pads = []
    for face in faces:
        z_face = face[0][2]
        horn = np.sign(z_face) == np.sign(horn_inner)
        arm_face = horn_inner if horn else idler_inner
        # From the case face outwards (twisted frame), up to the arm (link frame)
        s = 1.0 if z_face > 0 else -1.0
        outwards = cq.Workplane("XY").box(x1 - x0, 2 * pad_half_width, 200, centered=False) \
            .translate((x0, -pad_half_width, z_face if s > 0 else z_face - 200))
        to_arm = cq.Workplane("XY").box(x1 - x0, 400, 200, centered=False) \
            .translate((x0, -200, arm_face - 200 if s > 0 else arm_face))
        pad = place(outwards, tilt).intersect(to_arm)
        holes = None
        for p in face:
            hole = cq.Workplane("XY").workplane(offset=-60).center(p[0], p[1]).circle(servo.screw / 2).extrude(120)
            holes = hole if holes is None else holes.union(hole)
        pads.append(("horn_side" if horn else "idler_side", pad, place(holes, tilt)))

    # Each arm as wide as its pad where it meets the arm
    face_of = {key: (horn_inner if key == "horn_side" else idler_inner) for key, _, _ in pads}
    span_of = {key: y_span(pad, face_of[key] - 1, face_of[key] + 1) for key, pad, _ in pads}
    (horn_arm, idler_arm), _ = arms(servo, side, x1, (span_of["horn_side"], span_of["idler_side"]),
                                    *screw_holes_in_link_frame(servo, joint, lk), taper=(x0, pad_half_width))
    halves = {"horn_side": horn_arm, "idler_side": idler_arm}
    for key, pad, holes in pads:
        halves[key] = halves[key].union(pad).cut(holes)
    return halves, lk


def tibia(servo, joint, foot):
    """Arms on the tibia horn and idler, joined into a beam that ends in a foot"""
    lk = link_frame(joint)
    side = horn_side(joint, lk)
    length = apply(np.linalg.inv(lk), foot)[0]
    start = math.hypot(servo.case_x[1], servo.case_y) + CLEARANCE + 2  # clear of the servo case for any angle
    width, thickness = TIBIA_BEAM

    (horn_arm, idler_arm), (horn_inner, idler_inner) = arms(servo, side, start + 8, width,
                                                             *screw_holes_in_link_frame(servo, joint, lk))
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
    # Cut outs around the horn and the idler, and the bosses the arms stand off them on
    horn_cutout = servo.horn_radius + (1 if servo.horn_standoff > 0 else 0) + CLEARANCE
    for face_index, (thickness, cutout) in enumerate(((BODY_IDLER_SIDE, servo.idler_boss + STANDOFF + CLEARANCE),
                                                     (BODY_HORN_SIDE, horn_cutout))):
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
            # degrees), beyond the nose radius. The cut starts behind the axis, but not so far behind that the nose
            # comes loose from the plate (with a femur twist the nose radius can be small). Nothing of the
            # bracket comes closer to the axis than the nose radius, so the neck that joins the nose stays clear
            behind = min(SIDE_PLATE + servo.case_y + CLEARANCE, radius - 1)
            front = cq.Workplane("XY").workplane(offset=z0 - 1).center(100 - behind, 0).rect(200, 400).extrude(thickness + 2)
            nose = front.cut(cq.Workplane("XY").workplane(offset=z0 - 2).circle(radius).extrude(thickness + 4))
            plate = plate.cut(place(nose, plane_frame(lk, axis)))
            plate = plate.cut(cq.Workplane("XY").workplane(offset=z0 - 1).center(axis[0], axis[1]).circle(cutout).extrude(thickness + 2))
            for x, y in servo.case_holes[face_index]:
                p = apply(m, (x, y, 0))
                if math.hypot(x, y) > cutout + servo.screw:
                    plate = plate.cut(cq.Workplane("XY").workplane(offset=z0 - 1).center(p[0], p[1])
                                      .circle(servo.screw / 2).extrude(thickness + 2))
        plates.append(plate)
    return plates


def nose_radius(servo, assembly):
    """Body plates stay inside this radius around the front of a coxa axis: the coxa bracket's side
    plates (on the femur servo case) turn just outside it"""
    far_hole = max(-x for face in (0, 1) for x, _ in servo.mounting_holes(face))
    # Height (link frame) of the body plates' outer faces
    lo, hi = servo.case_faces
    z_body = max(abs(lo - BODY_IDLER_SIDE), abs(hi + BODY_HORN_SIDE)) + 1
    radius = math.inf
    twisted = False
    for leg in assembly["legs"]:
        coxa, femur = leg["joints"][0], leg["joints"][1]
        lk = link_frame(coxa)
        # Nearest the coxa axis: the inner edge of a side plate. Tilted by a femur twist, the plates' inner edges
        # come closer to the axis at the height of the body plates
        twist = joint_twist(servo, femur, lk, "y", "femur")
        twisted = twisted or twist != 0
        faces, _ = case_holes_in_link_frame(servo, femur, lk @ rx(twist))
        g = math.radians(twist)
        inner = servo.case_y
        for face in faces:
            y = abs(face[0][1]) + FIT
            for z in (-z_body, z_body):
                inner = min(inner, abs(y - math.copysign(1, face[0][1]) * z * math.sin(g)) / math.cos(g))
        radius = min(radius, math.hypot(math.dist(leg["points"][0], leg["points"][1]) - far_hole - PAD_MARGIN, inner))
    radius -= 2 * CLEARANCE
    near_corner = math.hypot(servo.case_x[1], servo.case_y)
    if radius < near_corner:
        hint = (". A femur twist tilts the coxa bracket's side plates towards the coxa axis: make the coxa longer or "
                "the twist smaller") if twisted else ""
        raise ValueError(f"the coxa is too short: the femur servo's mounting reaches within {radius:.1f} mm of the "
                         f"coxa axis, but the coxa servo case needs {near_corner:.1f} mm to turn{hint}")
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

    for name, part, _ in parts:
        if len(part.solids().vals()) != 1:
            raise ValueError(f"{name} falls apart into {len(part.solids().vals())} pieces")
    return parts


def servo_solids(assembly, servo_data):
    """Simple servo solids (case, horn, idler hub) in world coordinates, for the collision check"""
    servo = Servo(servo_data)
    shape = servo.simple()
    return [(f"leg{leg['index']}_{j['name']}_servo", place(shape, to_np(j["servo"])))
            for leg in assembly["legs"] for j in leg["joints"]]
