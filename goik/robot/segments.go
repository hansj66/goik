// Copyright 2025 Hans Jørgen Grimstad
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package robot

import (
	"fmt"
	"math"
)

/*
	Segmented bodies (centipedes)

	A segmented body is a chain of segments, the head first. Each leg belongs to a segment (LegSegments in the body
	definition), and its mount position and angle are in that segment's frame: origin at the segment's centre, +Y
	forward, like the frame of a one piece body. Neighbouring segments are joined by a yaw joint halfway between them.

	The segments follow the head ("follow the leader"): the head walks like a one piece pod, and every other segment
	follows the path the head has walked, Spacing mm behind the segment in front of it. A segment's heading is the
	direction of the path through it (from the joint behind it to the joint in front of it), so a joint's angle is the
	difference between the headings of the segments it joins. When the head turns on the spot, only the first joint
	bends.

	The gait engine keeps each foot in its own segment's frame. A grounded foot is moved by its segment's motion, so
	it stays where it is in the world.
*/

// SegmentedBody describes a body made of a chain of segments
type SegmentedBody struct {
	// Number of segments
	Count int `json:"Count"`
	// Distance (mm) between the centres of neighbouring segments. The joints are halfway between them
	Spacing float64 `json:"Spacing"`
	// Size (mm) of a segment, for drawing it as an ellipse: length (along the body) and width
	Length float64 `json:"Length"`
	Width  float64 `json:"Width"`
	// Largest angle (degrees) of a joint between two segments, either way
	MaxJointAngle float64 `json:"MaxJointAngle"`
}

// Validate checks the body's sizes
func (s *SegmentedBody) Validate() error {
	if s.Count < 1 {
		return fmt.Errorf("a segmented body needs at least 1 segment")
	}
	if s.Spacing <= 0 || s.Length <= 0 || s.Width <= 0 {
		return fmt.Errorf("the segments' spacing, length and width must be positive")
	}
	if s.MaxJointAngle <= 0 || s.MaxJointAngle >= 90 {
		return fmt.Errorf("the joints' largest angle must be between 0 and 90 degrees")
	}
	return nil
}

// SegmentPose is a segment's position (mm) and heading (radians: the rotation of its frame, counter clockwise from
// the frame it is given in)
type SegmentPose struct {
	X, Y, Heading float64
}

// ToParent transforms a point in the segment's frame to the frame the pose is given in
func (p SegmentPose) ToParent(c Coordinate) Coordinate {
	s, co := math.Sincos(p.Heading)
	return Coordinate{X: p.X + co*c.X - s*c.Y, Y: p.Y + s*c.X + co*c.Y, Z: c.Z}
}

// FromParent transforms a point in the frame the pose is given in to the segment's frame
func (p SegmentPose) FromParent(c Coordinate) Coordinate {
	s, co := math.Sincos(-p.Heading)
	dx, dy := c.X-p.X, c.Y-p.Y
	return Coordinate{X: co*dx - s*dy, Y: s*dx + co*dy, Z: c.Z}
}

// Relative returns the pose q seen from this pose (both given in the same frame)
func (p SegmentPose) Relative(q SegmentPose) SegmentPose {
	c := p.FromParent(Coordinate{X: q.X, Y: q.Y})
	return SegmentPose{X: c.X, Y: c.Y, Heading: wrapRadians(q.Heading - p.Heading)}
}

// RestSegmentPoses returns the segments' poses in the head's frame when the body is straight
func (s *SegmentedBody) RestSegmentPoses() []SegmentPose {
	poses := make([]SegmentPose, s.Count)
	for k := range poses {
		poses[k] = SegmentPose{Y: -float64(k) * s.Spacing}
	}
	return poses
}

// RestJoints returns the positions of the joints between the segments, in the head's frame, when the body is straight
func (s *SegmentedBody) RestJoints() []Coordinate {
	joints := make([]Coordinate, s.Count-1)
	for k := range joints {
		joints[k] = Coordinate{Y: -(float64(k) + 0.5) * s.Spacing}
	}
	return joints
}

// pathPoint is a point (world, mm) on the path the head has walked, at s mm along it
type pathPoint struct {
	x, y, s float64
}

// segmentChain moves the segments of a segmented body along the path the head walks
type segmentChain struct {
	body SegmentedBody
	// The head's path, oldest first, where the head is, and how far along the path it is (mm)
	path []pathPoint
	head SegmentPose
	arc  float64
	// World poses of the segments
	poses []SegmentPose
}

// Shortest distance (mm) between recorded path points
const PATH_STEP = 0.5

func newSegmentChain(body SegmentedBody, head SegmentPose) *segmentChain {
	// A straight path behind the head, long enough for the whole body
	length := float64(body.Count+1) * body.Spacing
	s, c := math.Sincos(head.Heading)
	back := Coordinate{X: head.X + s*length, Y: head.Y - c*length}
	ch := &segmentChain{
		body: body,
		path: []pathPoint{{x: back.X, y: back.Y, s: 0}, {x: head.X, y: head.Y, s: length}},
		head: head,
		arc:  length,
	}
	ch.updatePoses()
	return ch
}

// headArc returns how far (mm) along the path the head is
func (ch *segmentChain) headArc() float64 {
	return ch.arc
}

// pointAt returns the point s mm along the path (extrapolated before its start, and towards the head after its end)
func (ch *segmentChain) pointAt(s float64) (float64, float64) {
	p := ch.path
	// Before the start: along the first stretch of the path
	if s <= p[0].s {
		a, b := p[0], p[1]
		d := math.Max(b.s-a.s, 1e-9)
		return a.x + (b.x-a.x)*(s-a.s)/d, a.y + (b.y-a.y)*(s-a.s)/d
	}
	last := p[len(p)-1]
	if s >= last.s {
		d := ch.headArc() - last.s
		if d < 1e-9 {
			return last.x, last.y
		}
		t := math.Min((s-last.s)/d, 1)
		return last.x + (ch.head.X-last.x)*t, last.y + (ch.head.Y-last.y)*t
	}
	// Binary search for the stretch containing s
	lo, hi := 0, len(p)-1
	for hi-lo > 1 {
		mid := (lo + hi) / 2
		if p[mid].s <= s {
			lo = mid
		} else {
			hi = mid
		}
	}
	a, b := p[lo], p[hi]
	t := (s - a.s) / math.Max(b.s-a.s, 1e-9)
	return a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t
}

// moveHead moves the head to a new world pose, records its path, and moves the other segments along the path
func (ch *segmentChain) moveHead(head SegmentPose) {
	// How far the head moved along its own forward axis: backwards is negative
	s, c := math.Sincos(head.Heading)
	step := (head.X-ch.head.X)*-s + (head.Y-ch.head.Y)*c
	ch.head = head
	ch.arc += step
	if step >= 0 {
		last := ch.path[len(ch.path)-1]
		if d := math.Hypot(head.X-last.x, head.Y-last.y); d >= PATH_STEP {
			ch.path = append(ch.path, pathPoint{x: head.X, y: head.Y, s: ch.arc})
		}
	} else {
		// Backing up: the body retraces the path it came along (past its start, along a straight line)
		for len(ch.path) > 2 && ch.path[len(ch.path)-1].s > ch.arc {
			ch.path = ch.path[:len(ch.path)-1]
		}
	}
	// Forget the path behind the tail, but keep a stretch to extrapolate along
	tail := ch.headArc() - float64(ch.body.Count+1)*ch.body.Spacing
	keep := 0
	for keep < len(ch.path)-2 && ch.path[keep+1].s < tail {
		keep++
	}
	if keep > 64 {
		ch.path = append([]pathPoint(nil), ch.path[keep:]...)
	}
	ch.updatePoses()
}

// updatePoses places the segments on the path behind the head
func (ch *segmentChain) updatePoses() {
	if len(ch.poses) != ch.body.Count {
		ch.poses = make([]SegmentPose, ch.body.Count)
	}
	ch.poses[0] = ch.head
	headArc := ch.headArc()
	half := ch.body.Spacing / 2
	for k := 1; k < ch.body.Count; k++ {
		s := headArc - float64(k)*ch.body.Spacing
		x, y := ch.pointAt(s)
		fx, fy := ch.pointAt(s + half)
		bx, by := ch.pointAt(s - half)
		// The frame's +Y axis points forward, along the path: rotate +Y onto (fx - bx, fy - by)
		ch.poses[k] = SegmentPose{X: x, Y: y, Heading: math.Atan2(-(fx - bx), fy-by)}
	}
}

// joints returns the world positions of the joints between the segments
func (ch *segmentChain) joints() []Coordinate {
	joints := make([]Coordinate, ch.body.Count-1)
	headArc := ch.headArc()
	for k := range joints {
		x, y := ch.pointAt(headArc - (float64(k)+0.5)*ch.body.Spacing)
		joints[k] = Coordinate{X: x, Y: y}
	}
	return joints
}

// JointAngles returns the angles (degrees) of the joints between the segments: positive when the segment in front
// is turned counter clockwise from the one behind it
func (ch *segmentChain) jointAngles() []float64 {
	angles := make([]float64, ch.body.Count-1)
	for k := range angles {
		angles[k] = wrapRadians(ch.poses[k].Heading-ch.poses[k+1].Heading) * 180 / math.Pi
	}
	return angles
}

// wrapRadians returns the angle in [-pi, pi)
func wrapRadians(a float64) float64 {
	a = math.Mod(a+math.Pi, 2*math.Pi)
	if a < 0 {
		a += 2 * math.Pi
	}
	return a - math.Pi
}
