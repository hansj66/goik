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

"""Maintains the non-Go dependencies of the CAD export.

    python cad/tools.py setup              create cad/.venv and install CadQuery (cad/requirements.txt)
    python cad/tools.py vendor             download the vendor STEP files that can be downloaded automatically,
                                           and explain how to get the others (see docs/servo-models.md)
    python cad/tools.py check              show what is installed, downloaded and measured
    python cad/tools.py measure <file>     suggest the mount transform for a vendor STEP file (servos.json)

setup only needs a Python 3.9 - 3.12 installation. The other commands run with any Python, and
re-run themselves with the virtual environment's Python when they need CadQuery.
"""

import hashlib
import io
import json
import math
import os
import subprocess
import sys
import urllib.request
import venv
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
VENV = os.path.join(HERE, ".venv")
VENDOR = os.path.join(HERE, "vendor")
SERVOS = os.path.join(HERE, "servos.json")
REQUIREMENTS = os.path.join(HERE, "requirements.txt")


def venv_python():
    if os.name == "nt":
        return os.path.join(VENV, "Scripts", "python.exe")
    return os.path.join(VENV, "bin", "python")


def in_venv():
    return os.path.abspath(sys.prefix) == os.path.abspath(VENV)


def rerun_in_venv():
    """Re-run this command with the virtual environment's Python (which has CadQuery)"""
    if in_venv():
        return
    if not os.path.isfile(venv_python()):
        sys.exit("The CAD environment is not installed. Run: python cad/tools.py setup")
    sys.exit(subprocess.call([venv_python(), os.path.abspath(__file__)] + sys.argv[1:]))


def load_servos():
    with open(SERVOS, encoding="utf-8") as f:
        return json.load(f)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def setup():
    if sys.version_info < (3, 9) or sys.version_info >= (3, 13):
        print(f"Warning: CadQuery wheels are available for Python 3.9 - 3.12, this is {sys.version.split()[0]}")
    if not os.path.isfile(venv_python()):
        print("Creating", VENV)
        venv.create(VENV, with_pip=True)
    subprocess.check_call([venv_python(), "-m", "pip", "install", "--upgrade", "pip"])
    subprocess.check_call([venv_python(), "-m", "pip", "install", "-r", REQUIREMENTS])
    print("Done. Check the installation with: python cad/tools.py check")


def vendor():
    os.makedirs(VENDOR, exist_ok=True)
    for model, servo in load_servos().items():
        if model.startswith("_"):
            continue
        step = servo.get("vendor_step") or {}
        path = os.path.join(VENDOR, step.get("file", ""))
        if os.path.isfile(path):
            print(f"{model}: {path} already exists")
            continue

        download = step.get("download")
        if not download:
            print(f"{model}: download it manually from {step.get('page', 'the vendor (see docs/servo-models.md)')}")
            print(f"    and save it as {path}")
            continue

        print(f"{model}: downloading {download['url']}")
        with urllib.request.urlopen(download["url"]) as response:
            data = response.read()
        if "zip_member" in download:
            data = zipfile.ZipFile(io.BytesIO(data)).read(download["zip_member"])
        with open(path, "wb") as f:
            f.write(data)
        print(f"    saved as {path}")
        check_file(model, step, path)


def check_file(model, step, path):
    expected = step.get("sha256")
    actual = sha256(path)
    if expected and actual != expected:
        print(f"    WARNING: {model}: the file differs from the one the mount transform was measured on")
        print(f"    (sha256 {actual}, expected {expected}). Check the transform with: python cad/tools.py measure {path}")
    elif not expected:
        print(f"    sha256 {actual} (not recorded in servos.json yet)")


def check():
    print("Python:", sys.version.split()[0], sys.executable)
    if os.path.isfile(venv_python()):
        result = subprocess.run([venv_python(), "-c", "import cadquery; print(cadquery.__version__)"],
                                capture_output=True, text=True)
        print("CadQuery:", result.stdout.strip() if result.returncode == 0 else "NOT INSTALLED (python cad/tools.py setup)")
    else:
        print("CadQuery: no virtual environment (python cad/tools.py setup)")

    for model, servo in load_servos().items():
        if model.startswith("_"):
            continue
        step = servo.get("vendor_step") or {}
        path = os.path.join(VENDOR, step.get("file", ""))
        state = []
        state.append("geometry verified" if servo.get("verified") else "geometry NOT verified")
        if os.path.isfile(path):
            state.append("vendor model downloaded")
            if step.get("transform"):
                state.append("mount transform known")
            else:
                state.append("mount transform missing (python cad/tools.py measure)")
        else:
            state.append("no vendor model (python cad/tools.py vendor)")
        print(f"{model}: " + ", ".join(state))
        if os.path.isfile(path):
            check_file(model, step, path)


def measure(path):
    """Suggests the transform from the vendor file's coordinates to the servo frame.

    The horn is assumed to be the largest full circle (radius < 15 mm) on the outermost face of the
    model along one of the axes. The case extends from the horn axis towards the far end of the
    model's longest remaining side. Always check the result in a CAD program.
    """
    rerun_in_venv()
    import cadquery as cq

    shape = cq.Compound.makeCompound(cq.importers.importStep(path).vals())
    bb = shape.BoundingBox()
    lo = {"x": bb.xmin, "y": bb.ymin, "z": bb.zmin}
    hi = {"x": bb.xmax, "y": bb.ymax, "z": bb.zmax}
    print(f"{path}: {len(shape.Solids())} solids, size {bb.xlen:.2f} x {bb.ylen:.2f} x {bb.zlen:.2f} mm")
    print(f"    sha256 {sha256(path)}")

    # Candidate horns: full circles facing along an axis, lying on the outermost face in that
    # direction. CAD programs often store a circle as several arcs, so arcs with the same centre,
    # radius and axis are added up. Rounded edges are partial arcs and never add up to a circle.
    arcs = {}
    for edge in shape.Edges():
        if edge.geomType() != "CIRCLE" or edge.radius() > 15:
            continue
        n, c, r = edge.normal(), edge.arcCenter(), edge.radius()
        axis_index = max(range(3), key=lambda k: abs((n.x, n.y, n.z)[k]))
        if abs(abs((n.x, n.y, n.z)[axis_index]) - 1) > 0.01:
            continue
        key = (round(c.x, 2), round(c.y, 2), round(c.z, 2), round(r, 2), axis_index)
        arcs[key] = arcs.get(key, 0.0) + edge.Length() / r

    best = None
    for (cx, cy, cz, radius, i), angle in arcs.items():
        if angle < 2 * math.pi - 0.05:
            continue
        axis = "xyz"[i]
        value = (cx, cy, cz)[i]
        for sign, extreme in ((1, hi[axis]), (-1, lo[axis])):
            # Within 1 mm of the outermost face: the horn's outer edge is often chamfered
            if abs(value - extreme) < 1.0 and (best is None or radius > best[0]):
                best = (radius, axis, sign, (cx, cy, cz))

    if best is not None:
        # The horn face is the outermost surface of the solids that are at least as wide as the horn
        # (the horn and the case), ignoring narrow parts like a protruding centre screw
        radius, axis, sign, centre = best
        i = "xyz".index(axis)
        others = [k for k in range(3) if k != i]
        faces = []
        for solid in shape.Solids():
            b = solid.BoundingBox()
            lo_s, hi_s = (b.xmin, b.ymin, b.zmin), (b.xmax, b.ymax, b.zmax)
            covers = all(lo_s[k] <= centre[k] - radius * 0.9 and hi_s[k] >= centre[k] + radius * 0.9 for k in others)
            if covers:
                faces.append(hi_s[i] if sign > 0 else lo_s[i])
        if faces:
            centre = list(centre)
            centre[i] = max(faces) if sign > 0 else min(faces)
            best = (radius, axis, sign, tuple(centre))
    if best is None:
        sys.exit("No horn found: no circle on an outer face of the model. Measure the transform in a CAD program")

    radius, axis, sign, centre = best
    print(f"Horn: diameter {2 * radius:.2f} mm, centre ({centre[0]:.2f}, {centre[1]:.2f}, {centre[2]:.2f}), pointing {'+' if sign > 0 else '-'}{axis}")

    axes = "xyz"
    unit = {a: [1.0 if b == a else 0.0 for b in axes] for a in axes}
    z_servo = [sign * v for v in unit[axis]]

    # The case extends along the longest of the other two axes, towards its far end
    others = [a for a in axes if a != axis]
    long_axis = max(others, key=lambda a: hi[a] - lo[a])
    i = axes.index(long_axis)
    near_plus = hi[long_axis] - centre[i] < centre[i] - lo[long_axis]
    x_servo = [(1.0 if near_plus else -1.0) * v for v in unit[long_axis]]
    y_servo = [z_servo[1] * x_servo[2] - z_servo[2] * x_servo[1],
               z_servo[2] * x_servo[0] - z_servo[0] * x_servo[2],
               z_servo[0] * x_servo[1] - z_servo[1] * x_servo[0]]

    # Rows are the servo axes in vendor coordinates: p_servo = R (p_vendor - centre)
    rows = [x_servo, y_servo, z_servo]
    transform = [[r[0], r[1], r[2], -sum(r[k] * centre[k] for k in range(3))] for r in rows] + [[0, 0, 0, 1]]
    transform = [[round(v, 3) + 0.0 for v in row] for row in transform]

    axis_from_end = min(hi[long_axis] - centre[i], centre[i] - lo[long_axis])
    print(f"Case: {hi[long_axis] - lo[long_axis]:.2f} mm along {long_axis}, horn axis {axis_from_end:.2f} mm from the near end")
    print("Suggested vendor_step.transform for servos.json (check it in a CAD program):")
    print("    " + json.dumps(transform))


def main():
    commands = {"setup": setup, "vendor": vendor, "check": check}
    if len(sys.argv) == 3 and sys.argv[1] == "measure":
        measure(sys.argv[2])
    elif len(sys.argv) == 2 and sys.argv[1] in commands:
        commands[sys.argv[1]]()
    else:
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main()
