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

// Given an end effector target coordinate, SolveEffectorIK will attempt to find a solution for
// the coxa, femur and tibia angle that results in the end effector moving to the effectorTarget coordinate
// If a solution can not be found, the function returns an error.
func SolveEffectorIK(leg *Leg, effectorTarget Coordinate, debugChannel chan string) (ServoAngles, error) {
	if leg.FixedCoxa {
		return solveFixedCoxaIK(leg, effectorTarget)
	}
	if !leg.Twists.IsZero() {
		return solveTwistedIK(leg, effectorTarget)
	}

	var servoAngles ServoAngles
	// Inverse kinematics equation 1 (ref readme.md)
	x := effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
	y := effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y

	// servoAngles.Coxa = (180.0/math.Pi)*math.Atan2(y, x) + 360.0 - leg.CoxaSeparationAngle*float64(leg.Index)
	servoAngles.Coxa = (180.0/math.Pi)*math.Atan2(y, x) + 360.0 - leg.CoxaSeparationAngle

	if servoAngles.Coxa >= 180 {
		servoAngles.Coxa = servoAngles.Coxa - 360
	}

	// Inverse kinematics equation 2 (ref readme.md)
	dx := effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
	dy := effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y

	L1 := math.Sqrt((dx*dx + dy*dy)) - leg.SegmentLengths.Coxa
	L2 := effectorTarget.Z - leg.Joints[FEMUR_ORIGIN_INDEX].Z
	L := math.Sqrt(L2*L2 + L1*L1)

	// Inverse kinematics equation 3 (ref readme.md)
	alpha_1 := math.Acos(L2 / L)

	if math.IsNaN(alpha_1) {
		return servoAngles, fmt.Errorf("[IK Solver] ERROR: Unable to find a solution. Target is too far away.")
	}

	// Inverse kinematics equation 4 (ref readme.md)
	alpha_2 := math.Acos(
		(leg.SegmentLengths.Tibia*leg.SegmentLengths.Tibia -
			leg.SegmentLengths.Femur*leg.SegmentLengths.Femur -
			L*L) /
			(-2 * leg.SegmentLengths.Femur * L))

	if math.IsNaN(alpha_2) {
		return servoAngles, fmt.Errorf("[IK Solver] ERROR: Unable to find a solution. Target is too far away.")
	}

	// Inverse kinematics equation 5 (ref readme.md)
	servoAngles.Femur = 90 - (180.0/math.Pi)*(alpha_1+alpha_2)

	// Inverse kinematics equation 6 (ref readme.md)
	servoAngles.Tibia = 180 - (180.0/math.Pi)*math.Acos((L*L-leg.SegmentLengths.Femur*leg.SegmentLengths.Femur-leg.SegmentLengths.Tibia*leg.SegmentLengths.Tibia)/(-2*leg.SegmentLengths.Tibia*leg.SegmentLengths.Femur))

	return servoAngles, nil
}

// solveTwistedIK solves a twisted leg numerically (see twist.go), starting from the leg's current angles
func solveTwistedIK(leg *Leg, target Coordinate) (ServoAngles, error) {
	const toRadians = math.Pi / 180
	seed := [3]float64{leg.ServoAngles.Coxa * toRadians, leg.ServoAngles.Femur * toRadians, leg.ServoAngles.Tibia * toRadians}
	q, err := leg.model.solve(vec3{target.X, target.Y, target.Z}, seed)
	if err != nil {
		return leg.ServoAngles, err
	}
	return ServoAngles{Coxa: wrapDegrees(q[0] / toRadians), Femur: wrapDegrees(q[1] / toRadians), Tibia: wrapDegrees(q[2] / toRadians)}, nil
}

// Largest distance (mm) between where a two joint leg's foot should be and where it can be. A two joint leg moves in
// one plane, so a foot that should move sideways (a segment of a segmented body turning) slips a little instead
const MAX_FOOT_SLIP = 15.0

// solveFixedCoxaIK solves a two joint leg (femur and tibia) numerically, with the coxa held at its rest angle. The
// foot goes as close to the target as the leg's plane allows
func solveFixedCoxaIK(leg *Leg, target Coordinate) (ServoAngles, error) {
	const toRadians = math.Pi / 180
	seed := [3]float64{leg.ServoAngles.Coxa * toRadians, leg.ServoAngles.Femur * toRadians, leg.ServoAngles.Tibia * toRadians}
	q, err := leg.model.solveWith(vec3{target.X, target.Y, target.Z}, seed, [3]bool{true, false, false}, MAX_FOOT_SLIP)
	if err != nil {
		return leg.ServoAngles, err
	}
	return ServoAngles{Coxa: leg.ServoAngles.Coxa, Femur: wrapDegrees(q[1] / toRadians), Tibia: wrapDegrees(q[2] / toRadians)}, nil
}
