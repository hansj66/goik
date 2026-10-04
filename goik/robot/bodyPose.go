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

/*
	Notes regarding the body pose

	The gait engine keeps the feet in the *ground* reference frame: the base reference frame of the
	pod when it stands in its neutral stance. The body pose describes how the body is rotated and
	moved relative to that frame, while the feet stay where they are. Before solving IK, each foot is
	transformed into the body's reference frame.

	Axes (the README defines +Y as forward, and the feet are at +Z):
		Pitch: rotation around X. Positive pitch raises the front (+Y) of the body.
		Roll:  rotation around Y. Positive roll lowers the +X side of the body.
		Yaw:   rotation around Z. Positive yaw turns the body from +X towards +Y.
		X/Y:   body shift in mm. This also moves the centre of gravity.
		Z:     body height offset in mm. Positive Z is towards the ground, so "up" is negative Z.
*/

import (
	"fmt"
	"math"
)

// Largest body rotation (degrees) and translation (mm) that can be requested
const MAX_POSE_ANGLE = 45.0
const MAX_POSE_TRANSLATION = 100.0

// BodyPose is the body's rotation (degrees) and translation (mm) relative to the neutral stance
type BodyPose struct {
	Pitch float64
	Roll  float64
	Yaw   float64
	X     float64
	Y     float64
	Z     float64
}

func (b BodyPose) String() string {
	return fmt.Sprintf("[pitch %2.1f, roll %2.1f, yaw %2.1f, x %2.1f, y %2.1f, z %2.1f]", b.Pitch, b.Roll, b.Yaw, b.X, b.Y, b.Z)
}

// IsZero returns true for the neutral pose
func (b BodyPose) IsZero() bool {
	return b == BodyPose{}
}

// ToGround transforms a point from the body's reference frame to the ground reference frame
func (b BodyPose) ToGround(c Coordinate) Coordinate {
	c = rotateY(c, -b.Roll*math.Pi/180)
	c = rotateX(c, -b.Pitch*math.Pi/180)
	c = rotateZ(c, b.Yaw*math.Pi/180)
	return NewCoordinate(c.X+b.X, c.Y+b.Y, c.Z+b.Z)
}

// ToBody transforms a point from the ground reference frame to the body's reference frame
func (b BodyPose) ToBody(c Coordinate) Coordinate {
	c = NewCoordinate(c.X-b.X, c.Y-b.Y, c.Z-b.Z)
	c = rotateZ(c, -b.Yaw*math.Pi/180)
	c = rotateX(c, b.Pitch*math.Pi/180)
	return rotateY(c, b.Roll*math.Pi/180)
}

func rotateX(c Coordinate, a float64) Coordinate {
	return NewCoordinate(c.X, c.Y*math.Cos(a)-c.Z*math.Sin(a), c.Y*math.Sin(a)+c.Z*math.Cos(a))
}

func rotateY(c Coordinate, a float64) Coordinate {
	return NewCoordinate(c.X*math.Cos(a)+c.Z*math.Sin(a), c.Y, -c.X*math.Sin(a)+c.Z*math.Cos(a))
}

func rotateZ(c Coordinate, a float64) Coordinate {
	return NewCoordinate(c.X*math.Cos(a)-c.Y*math.Sin(a), c.X*math.Sin(a)+c.Y*math.Cos(a), c.Z)
}

// SetBodyPose sets the body pose the engine will move towards
func (e *GaitEngine) SetBodyPose(b BodyPose) error {
	for _, a := range []float64{b.Pitch, b.Roll, b.Yaw} {
		if math.Abs(a) > MAX_POSE_ANGLE {
			return fmt.Errorf("body rotation is limited to +/- %2.0f degrees", MAX_POSE_ANGLE)
		}
	}
	for _, t := range []float64{b.X, b.Y, b.Z} {
		if math.Abs(t) > MAX_POSE_TRANSLATION {
			return fmt.Errorf("body translation is limited to +/- %2.0f mm", MAX_POSE_TRANSLATION)
		}
	}
	e.targetPose = b
	e.PoseLimited = false
	return nil
}

// TargetBodyPose returns the body pose the engine is moving towards
func (e *GaitEngine) TargetBodyPose() BodyPose {
	return e.targetPose
}

// BodyPose returns the current body pose
func (e *GaitEngine) BodyPose() BodyPose {
	return e.pose
}

// ToGround transforms a point from the body's reference frame to the ground reference frame
// using the current body pose
func (e *GaitEngine) ToGround(c Coordinate) Coordinate {
	return e.pose.ToGround(c)
}

// rampPose moves the body pose towards the target pose within the rate limits.
// Returns true if the pose changed.
func (e *GaitEngine) rampPose(dt float64) bool {
	e.previousPose = e.pose
	rotation := e.PoseRotationRate * dt
	translation := e.PoseTranslationRate * dt

	e.pose.Pitch = approach(e.pose.Pitch, e.targetPose.Pitch, rotation)
	e.pose.Roll = approach(e.pose.Roll, e.targetPose.Roll, rotation)
	e.pose.Yaw = approach(e.pose.Yaw, e.targetPose.Yaw, rotation)
	e.pose.X = approach(e.pose.X, e.targetPose.X, translation)
	e.pose.Y = approach(e.pose.Y, e.targetPose.Y, translation)
	e.pose.Z = approach(e.pose.Z, e.targetPose.Z, translation)

	return e.pose != e.previousPose
}

// centreOfGravity returns the centre of gravity in the ground reference frame (projected on XY).
// It is assumed to be at the origin of the body's reference frame.
func (e *GaitEngine) centreOfGravity() Coordinate {
	c := e.pose.ToGround(Coordinate{})
	return NewCoordinate(c.X, c.Y, 0)
}
