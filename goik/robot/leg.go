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
	"math"

	"gonum.org/v1/gonum/mat"
)

// The number of joints in a leg (Coxa, Femur, Tibia, End effector)
const NUM_JOINTS = 4

// Offsets in leg joint array
const COXA_ORIGIN_INDEX = 0
const FEMUR_ORIGIN_INDEX = 1
const TIBIA_ORIGIN_INDEX = 2
const EFFECTOR_ORIGIN_INDEX = 3

// SegmentLengths defines the lengths of each segment in the robot leg
// The length is calculated from the origin of one reference frame to
// the origin of the next reference frame in the kinematic chain.
type SegmentLengths struct {
	Coxa  float64 `json:"C"`
	Femur float64 `json:"F"`
	Tibia float64 `json:"T"`
}

type Leg struct {
	// Leg index is displayed in the simualtor views and
	// is also used to calculate the servo ID representing
	// a specific joint (please refer to docs/servos.md
	// for the numbering convention used).
	Index int
	// The robot legs are arranged around the body of the robot
	// Sepration angle defines the separation in degrees
	// between one leg and the next.
	CoxaSeparationAngle float64
	// The offset transformation matrix is used for calculating
	// the positioning of the Coxa reference frame origin in
	// the robot body base reference frame.
	OffsetTransformationMatrix *mat.Dense
	// All robot legs are not necessarily created equal
	// Most hexapods have identical leg topologies, but it
	// never hurts to prepare for other form factors :)
	SegmentLengths SegmentLengths
	// Twists of the joints' rotation axes (see twist.go). All 0 is the standard leg
	Twists JointTwists
	// A fixed coxa: the leg has two joints (femur and tibia) and moves in one plane
	FixedCoxa bool
	// The kinematic chain for the numeric inverse kinematics of twisted legs
	model legModel
	// The Joints array contain the location of the reference
	// frame origin for each joint in the base reference frame
	// coordinate system.
	Joints [NUM_JOINTS]Coordinate
	// ServoAngles represent the angles for the current state of the leg
	// (moving or stationary)
	ServoAngles ServoAngles
	// NeutralEffectorCoordinate defines the end effector's coordinate in
	// the base reference frame when the leg is in a neutral / rest position
	NeutralEffectorCoordinate Coordinate
	// Debug messages sent to the debug channel and will appear in the chat ui
	debugChannel chan string
}

// GetJointOrigin is a helper function for extracting coordinates from
// a homogeneus transformation matrix (after multiplication).
func (l *Leg) GetJointOrigin(H *mat.Dense) Coordinate {
	return Coordinate{X: H.At(0, 3), Y: H.At(1, 3), Z: H.At(2, 3)}
}

// RecalculateForwardKinematics is necessary for viewing the robot representation
// The robot center defines the base reference frame and each joint (including the end effector)
// has a reference frame.
// All rotations are done around the Z-axis of the reference frame and each reference frame
// is displaced from the previous frame.
// By starting with the base reference frame we define a displacement vector and a rotation
// matrix for determining the origin of the next reference frame in the kinematic chain
// (center of robot -> coxa origin -> femur origin -> tibia origin -> end effector)
// We then create a homogeneous tranformation matrix containing the rotation matrix and
// the displacement vector for each new reference frame in the chain.
// By multiplying these matrices together, we can extract the coordinates for each frame (coxa, femur, tibia, end effector)
// from the last column in the resulting 4x4 matrix. This gives us the data we need to represent the
// robot in a 2/3D view.
func (l *Leg) RecalculateForwardKinematics(angles ServoAngles) {
	l.ServoAngles = angles

	H0_1, H1_2, H2_3 := l.transforms(angles)

	l.Joints[COXA_ORIGIN_INDEX] = l.GetJointOrigin(l.OffsetTransformationMatrix)
	l.Joints[FEMUR_ORIGIN_INDEX] = l.GetJointOrigin(H0_1)
	l.Joints[TIBIA_ORIGIN_INDEX] = l.GetJointOrigin(H1_2)
	l.Joints[EFFECTOR_ORIGIN_INDEX] = l.GetJointOrigin(H2_3)
}

// transforms returns the homogeneous transformation matrices from the base reference frame to
// the femur origin (H0_1), the tibia origin (H1_2) and the end effector (H2_3)
func (l *Leg) transforms(angles ServoAngles) (*mat.Dense, *mat.Dense, *mat.Dense) {
	// The femur axis is turned 90 degrees (plus its twist) about the coxa, the tibia axis by its twist about the femur
	P_Coxa := rotXDense(90 + l.Twists.Femur)
	P_Femur := rotXDense(l.Twists.Tibia)
	P_Tibia := mat.NewDense(3, 3, []float64{1, 0, 0, 0, 1, 0, 0, 0, 1}) // Identity

	H_Coxa := HomogeneousTransformationMatrix(P_Coxa, angles.Coxa*math.Pi/180.0, l.SegmentLengths.Coxa)
	H_Femur := HomogeneousTransformationMatrix(P_Femur, angles.Femur*math.Pi/180.0, l.SegmentLengths.Femur)
	H_Tibia := HomogeneousTransformationMatrix(P_Tibia, angles.Tibia*math.Pi/180.0, l.SegmentLengths.Tibia)

	var H0_1 mat.Dense
	var H1_2 mat.Dense
	var H2_3 mat.Dense

	H0_1.Mul(l.OffsetTransformationMatrix, H_Coxa)
	H1_2.Mul(&H0_1, H_Femur)
	H2_3.Mul(&H1_2, H_Tibia)
	return &H0_1, &H1_2, &H2_3
}

// ServoFrames returns the reference frames the coxa, femur and tibia servos are mounted in (base
// reference frame, current angles). Each frame's origin is on its joint and its Z axis is the joint's
// rotation axis. The servo case is fixed in the frame, and its horn turns around Z by the joint angle.
// X points along the link the servo is mounted on (for the coxa servo: along the leg at coxa angle 0).
func (l *Leg) ServoFrames() [NUM_JOINTS - 1]*mat.Dense {
	H0_1, H1_2, _ := l.transforms(l.ServoAngles)
	return [NUM_JOINTS - 1]*mat.Dense{mat.DenseCopyOf(l.OffsetTransformationMatrix), H0_1, H1_2}
}

// NewLeg returns a new leg
func NewLeg(
	// 0 - NUM_LEGS-1
	Index int,
	// Separation angle (in degrees) from previous leg
	CoxaSeparationAngle float64,
	// This defines the offset and rotation in relation to the center of the robot.
	// This also represent the start of the kinematic chain for the leg
	OffsetTransformationMatrix *mat.Dense,
	// Servo angles for the leg in rest / neutral position
	// (If all angles are zero, the kinematic chain forms a straight line)
	ServoAngles ServoAngles,
	// The distance between reference frames (coxa == distance from coxa reference frame origin to femur reference frame origin)
	SegmentLengths SegmentLengths,
	// Twists of the joint axes (all 0: the standard leg). The coxa twist must be part of OffsetTransformationMatrix (see MountMatrix)
	Twists JointTwists,
	// output channel for debug messages
	debugChannel chan string) *Leg {
	l := Leg{
		OffsetTransformationMatrix: OffsetTransformationMatrix,
		SegmentLengths:             SegmentLengths,
		Twists:                     Twists,
		Index:                      Index,
		ServoAngles:                ServoAngles,
		CoxaSeparationAngle:        CoxaSeparationAngle,
		debugChannel:               debugChannel,
	}

	origin := l.GetJointOrigin(OffsetTransformationMatrix)
	l.model = newLegModel(origin, CoxaSeparationAngle, SegmentLengths, Twists)

	l.RecalculateForwardKinematics(ServoAngles)

	l.NeutralEffectorCoordinate = l.Joints[EFFECTOR_ORIGIN_INDEX]

	return &l
}

// rotXDense returns a rotation about X (degrees) as a matrix
func rotXDense(degrees float64) *mat.Dense {
	r := rotX(degrees)
	return mat.NewDense(3, 3, []float64{r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2]})
}

// MountMatrix returns the transformation from the body to a leg's coxa frame: the coxa joint at position, the
// leg pointing at the mount angle (degrees) and the coxa axis turned by the coxa twist (degrees)
func MountMatrix(position Coordinate, mountAngle float64, coxaTwist float64) *mat.Dense {
	r := rotZ(mountAngle * math.Pi / 180).mul(rotX(coxaTwist))
	return mat.NewDense(4, 4, []float64{
		r[0][0], r[0][1], r[0][2], position.X,
		r[1][0], r[1][1], r[1][2], position.Y,
		r[2][0], r[2][1], r[2][2], position.Z,
		0, 0, 0, 1,
	})
}

// reaches returns true if the leg can put its foot on the target. A two joint leg only reaches targets in its plane,
// so a target outside the plane counts as reached if the foot is no further from it than the plane is
func (l *Leg) reaches(target Coordinate, debugChannel chan string) bool {
	angles, err := SolveEffectorIK(l, target, debugChannel)
	if err != nil {
		return false
	}
	if !l.FixedCoxa {
		return true
	}
	const toRadians = math.Pi / 180
	joints, axes := l.model.forward([3]float64{angles.Coxa * toRadians, angles.Femur * toRadians, angles.Tibia * toRadians})
	t := vec3{target.X, target.Y, target.Z}
	outside := math.Abs(t.sub(joints[1]).dot(axes[1]))
	return t.sub(joints[3]).length() <= outside+1
}
