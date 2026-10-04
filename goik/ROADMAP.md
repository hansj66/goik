# Roadmap

## R & D / TODO

1. Smooth gait transitions
1. Smooth transitions between walking sequences (example: from walking to rotating / turning etc). How to chain motion primitives togeteher, so that the full gait seems natural ?
1. Add a new transformer for the body frame. Rotate X/Y/Z. Translate X/Y/Z. This maps nicely to pitch/yaw/roll and shifting the center of gravity 
1. Script / interpreter functionality for composing walking sequences / motion primitives / moves
1. Parametrize number of interpolation steps (have to be odd)
1. Motionplanner 
1. Get list of stored moves / primitives from controller (via UDP)
1. Full hexapod step file generation (with input of servo step files)
1. "X-march" / arc pattern (walk in a circle. Not rotation). Piece de resistance: Arc with spin ?
