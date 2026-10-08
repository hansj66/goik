# Interactive pod designer (plan)

Pods are designed with shell commands today: see [designing-a-pod.md](designing-a-pod.md). This is the plan for
designing them by pointing and clicking in the simulator window. The design model (`robot/podDesign.go`) and the
stance (`robot/stance.go`) don't depend on the GUI, so the interactive designer is a front end for them.

## Design mode

`design` in the shell (or a key) switches the window to a design mode, and back to the simulator with the pod rebuilt.

* **Canvas** (most of the window): a top view with the front up and the symmetry axis as a dashed line. You edit the
  +X half and see the mirror image live. Zoom with the mouse wheel, pan with the middle button, a key to fit the pod.
  Snap to a grid (5 mm by default).
* **Outline:** start from a template (rectangle, round) or click points. Drag points, double-click an edge to insert a
  point, delete a point. The end points stay on the axis.
* **Legs:** click near the outline to add a leg. It snaps to the outline, and its mount angle defaults to the
  outline's outward normal (an arrow). Drag the leg along the edge, drag the arrow tip to turn it (Shift snaps to
  15 degrees). Dropped on the axis it becomes a single leg.
* **Property panel** for the selected leg: position, mount angle, segment lengths and rest angles, typed in exactly,
  and "copy to all legs".
* **Leg side view:** the selected leg in its own plane. Drag the body up and down to change the stance's height, and
  drag the foot to change the leg's reach.
* **Readouts:** standing height and foot spread. Later the servo footprints and the checks from the "Validate a
  design" roadmap item.
* **Undo and redo:** the design is a small value, so a snapshot per change is enough.
* **Try it:** walk the new pod right away.

## Decisions

* Symmetry about the Y axis only. Radial layouts are templates (`design round`), not a symmetry mode.
* Leg parameters per leg (or mirrored pair), with "copy to all legs", instead of reusable leg types.
* The mount angle of a new leg defaults to the outline's normal, not to the direction from the centre.
* The designer lives in the simulator window. The property panel uses
  [debugui](https://github.com/ebitengine/debugui) (an immediate mode UI for Ebitengine by its author), which needs a
  newer Ebitengine than the v2.6.7 in go.mod.
* The outline is meant to become the shape of the printed body plates in the [CAD export](cad-export.md).

## Phases

1. **Design model and shell commands** (done): `PodDesign`, mirroring, leg numbering, import, templates, generated
   gaits for any number of legs, design aware leg commands, saving designs in pod files, and the stance.
2. **Design mode, body:** Ebitengine upgrade and debugui, canvas with zoom and pan, outline editing, legs with
   mirroring, property panel, undo. The XY and isometric views draw the outline.
3. **Legs:** side view, dragging the body height and the feet (the stance), try it.
4. **Validation overlay:** servo case footprints at the coxa mounts, coxa clearance (needs the servo dimensions in
   Go, see the roadmap).
5. **CAD:** the outline becomes the body plate shape in the CAD export (today the plates are the convex hull of the
   coxa servo cases plus a margin), with a check that the servos fit inside it.
