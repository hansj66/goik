# Roadmap

## R & D / TODO

1. Smooth gait transitions
1. Smooth transitions between walking sequences (example: from walking to rotating / turning etc). How to chain motion primitives togeteher, so that the full gait seems natural ?
1. Add a new transformer for the body frame. Rotate X/Y/Z. Translate X/Y/Z. This maps nicely to pitch/yaw/roll and shifting the center of gravity 
1. Script / interpreter functionality for composing walking sequences / motion primitives / moves
1. Parametrize number of interpolation steps (have to be odd)
1. Motionplanner 
1. Full hexapod step file generation (with input of servo step files)
1. "X-march" / arc pattern (walk in a circle. Not rotation). Piece de resistance: Arc with spin ?
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
