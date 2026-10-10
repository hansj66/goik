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
	Twisted joints

	Each joint's rotation axis can be turned ("twisted") about the link leading into the joint. The joint still
	turns in a plane, but the plane doesn't have to be horizontal or vertical. All twists 0 is the standard leg:

		Coxa twist:  the coxa axis turned about the mount direction (the direction the coxa points at a coxa
		             angle of 0). 0: the coxa axis is vertical
		Femur twist: the femur axis turned about the coxa. 0: the femur axis is horizontal, square to the coxa
		Tibia twist: the tibia axis turned about the femur. 0: the tibia axis is parallel to the femur axis

	A positive twist turns the axis counter clockwise, seen from the end of the link (looking back towards the
	body), which is the right hand rule about the outward link. In the leg's kinematic chain (see Leg.transforms)
	the twists are rotations about X:

		mount        T(position) Rz(mount angle) Rx(coxa twist)
		coxa joint   Rz(coxa angle) Tx(coxa) Rx(90 + femur twist)
		femur joint  Rz(femur angle) Tx(femur) Rx(tibia twist)
		tibia joint  Rz(tibia angle) Tx(tibia)

	The mirror image of a leg (in a design, mirrored in the YZ plane) has the opposite twists, the opposite coxa
	angle and the same femur and tibia angles. Mirroring turns every rotation about Z and X the other way, which
	leaves a 180 degree turn at the femur joint (Rx(-90 - t) = Rx(90 - t) Rx(180)); moving that turn to the end
	of the chain, where it moves no joint, turns the femur and tibia angles back.

	Inverse kinematics for a twisted leg has no simple closed form, so it is solved numerically (see solve):
	damped least squares (Levenberg-Marquardt) with the exact Jacobian, starting from the leg's current angles.
	Starting there keeps the solution on the same branch (the knee stays on the same side) from tick to tick,
	and it converges in a few iterations. Untwisted legs use the closed form solution in solver.go.
*/

// The largest twist (degrees) a joint can have
const MAX_TWIST = 45.0

// JointTwists turns the joints' rotation axes (degrees, see the notes above). All 0 is the standard leg
type JointTwists struct {
	Coxa  float64 `json:"Coxa,omitempty"`
	Femur float64 `json:"Femur,omitempty"`
	Tibia float64 `json:"Tibia,omitempty"`
}

// IsZero returns true for an untwisted leg
func (t JointTwists) IsZero() bool {
	return t == JointTwists{}
}

// Mirrored returns the twists of the leg's mirror image
func (t JointTwists) Mirrored() JointTwists {
	return JointTwists{Coxa: 0 - t.Coxa, Femur: 0 - t.Femur, Tibia: 0 - t.Tibia}
}

// Validate checks that the twists are within MAX_TWIST
func (t JointTwists) Validate() error {
	for _, j := range []struct {
		name  string
		twist float64
	}{{"coxa", t.Coxa}, {"femur", t.Femur}, {"tibia", t.Tibia}} {
		if math.Abs(j.twist) > MAX_TWIST {
			return fmt.Errorf("the %s twist (%.1f degrees) is larger than %.0f degrees", j.name, j.twist, MAX_TWIST)
		}
	}
	return nil
}

func (t JointTwists) String() string {
	return fmt.Sprintf("coxa %.1f, femur %.1f, tibia %.1f", t.Coxa, t.Femur, t.Tibia)
}

// Small fixed size vectors and rotation matrices for the leg model (no allocations, so the solver is fast)
type vec3 [3]float64
type mat3 [3][3]float64

func (a vec3) add(b vec3) vec3        { return vec3{a[0] + b[0], a[1] + b[1], a[2] + b[2]} }
func (a vec3) sub(b vec3) vec3        { return vec3{a[0] - b[0], a[1] - b[1], a[2] - b[2]} }
func (a vec3) scale(s float64) vec3   { return vec3{a[0] * s, a[1] * s, a[2] * s} }
func (a vec3) dot(b vec3) float64     { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }
func (a vec3) length() float64        { return math.Sqrt(a.dot(a)) }
func (a vec3) coordinate() Coordinate { return Coordinate{X: a[0], Y: a[1], Z: a[2]} }

func (a vec3) cross(b vec3) vec3 {
	return vec3{a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]}
}

func (m mat3) mul(n mat3) mat3 {
	var r mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			r[i][j] = m[i][0]*n[0][j] + m[i][1]*n[1][j] + m[i][2]*n[2][j]
		}
	}
	return r
}

func (m mat3) apply(v vec3) vec3 {
	return vec3{m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2], m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2], m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2]}
}

// column returns column j (for a rotation matrix: where the frame's axis j points)
func (m mat3) column(j int) vec3 {
	return vec3{m[0][j], m[1][j], m[2][j]}
}

func rotX(degrees float64) mat3 {
	s, c := math.Sincos(degrees * math.Pi / 180)
	return mat3{{1, 0, 0}, {0, c, -s}, {0, s, c}}
}

func rotZ(radians float64) mat3 {
	s, c := math.Sincos(radians)
	return mat3{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}
}

// legModel is the leg's kinematic chain, for fast forward kinematics and the numeric inverse kinematics
type legModel struct {
	// The coxa joint, and its frame (the mount: Rz(mount angle) Rx(coxa twist))
	origin vec3
	mount  mat3
	// Segment lengths
	coxa, femur, tibia float64
	// The femur and tibia axes relative to the link before them: Rx(90 + femur twist) and Rx(tibia twist)
	toFemur, toTibia mat3
}

func newLegModel(origin Coordinate, mountAngle float64, segments SegmentLengths, twists JointTwists) legModel {
	return legModel{
		origin:  vec3{origin.X, origin.Y, origin.Z},
		mount:   rotZ(mountAngle * math.Pi / 180).mul(rotX(twists.Coxa)),
		coxa:    segments.Coxa,
		femur:   segments.Femur,
		tibia:   segments.Tibia,
		toFemur: rotX(90 + twists.Femur),
		toTibia: rotX(twists.Tibia),
	}
}

// forward returns the joint positions (coxa, femur, tibia, foot) and the joint axes (coxa, femur, tibia) for
// the joint angles q (radians)
func (m *legModel) forward(q [3]float64) (joints [NUM_JOINTS]vec3, axes [3]vec3) {
	r := m.mount
	joints[0] = m.origin
	axes[0] = r.column(2)

	r = r.mul(rotZ(q[0]))
	joints[1] = joints[0].add(r.column(0).scale(m.coxa))
	r = r.mul(m.toFemur)
	axes[1] = r.column(2)

	r = r.mul(rotZ(q[1]))
	joints[2] = joints[1].add(r.column(0).scale(m.femur))
	r = r.mul(m.toTibia)
	axes[2] = r.column(2)

	r = r.mul(rotZ(q[2]))
	joints[3] = joints[2].add(r.column(0).scale(m.tibia))
	return joints, axes
}

// Numeric inverse kinematics settings
const (
	// The foot must be this close (mm) to the target
	IK_TOLERANCE = 1e-7
	// Damping (mm) of the least squares steps: keeps steps small near singular poses (leg stretched out)
	IK_DAMPING = 0.5
	// Largest change (radians) of a joint angle in one iteration, so the solution stays on its branch
	IK_MAX_STEP       = 0.3
	IK_MAX_ITERATIONS = 50
)

// solve returns the joint angles (radians) that put the foot on target, starting from seed. It fails if the
// target is out of reach.
func (m *legModel) solve(target vec3, seed [3]float64) ([3]float64, error) {
	return m.solveWith(target, seed, [3]bool{}, 10*IK_TOLERANCE)
}

// solveWith is solve with some joints held at their seed angles. With a joint held, the foot may not be able to
// reach the target at all: it then ends as close to it as it can get, and that may be up to maxMiss mm off.
func (m *legModel) solveWith(target vec3, seed [3]float64, held [3]bool, maxMiss float64) ([3]float64, error) {
	q := seed
	for i := 0; i < IK_MAX_ITERATIONS; i++ {
		joints, axes := m.forward(q)
		e := target.sub(joints[3])
		if e.length() < IK_TOLERANCE {
			return q, nil
		}

		// Jacobian: column j is how the foot moves when joint j turns (axis x lever arm). A held joint doesn't
		var J mat3
		for j := 0; j < 3; j++ {
			if held[j] {
				continue
			}
			c := axes[j].cross(joints[3].sub(joints[j]))
			J[0][j], J[1][j], J[2][j] = c[0], c[1], c[2]
		}

		// Damped least squares: dq = J^T (J J^T + d^2 I)^-1 e
		var A mat3
		for r := 0; r < 3; r++ {
			for c := 0; c < 3; c++ {
				A[r][c] = J[r][0]*J[c][0] + J[r][1]*J[c][1] + J[r][2]*J[c][2]
			}
			A[r][r] += IK_DAMPING * IK_DAMPING
		}
		y, ok := solve3(A, e)
		if !ok {
			break
		}
		var dq [3]float64
		largest := 0.0
		for j := 0; j < 3; j++ {
			dq[j] = J[0][j]*y[0] + J[1][j]*y[1] + J[2][j]*y[2]
			largest = math.Max(largest, math.Abs(dq[j]))
		}
		if largest > IK_MAX_STEP {
			for j := range dq {
				dq[j] *= IK_MAX_STEP / largest
			}
		}
		for j := range q {
			q[j] += dq[j]
		}
		// Converged as close as it gets (with held joints the target may be out of reach)
		if largest < 1e-10 {
			break
		}
	}

	joints, _ := m.forward(q)
	if d := target.sub(joints[3]).length(); d > maxMiss {
		return q, fmt.Errorf("[IK Solver] ERROR: Unable to find a solution. The foot gets no closer than %.2f mm to the target", d)
	}
	return q, nil
}

// solve3 solves A x = b for a 3x3 matrix (Cramer's rule)
func solve3(A mat3, b vec3) (vec3, bool) {
	det := A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1]) - A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0]) + A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0])
	if math.Abs(det) < 1e-12 {
		return vec3{}, false
	}
	var x vec3
	for i := 0; i < 3; i++ {
		M := A
		for r := 0; r < 3; r++ {
			M[r][i] = b[r]
		}
		x[i] = (M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1]) - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0]) + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0])) / det
	}
	return x, true
}

// wrapDegrees returns the angle in [-180, 180)
func wrapDegrees(a float64) float64 {
	a = math.Mod(a+180, 360)
	if a < 0 {
		a += 360
	}
	return a - 180
}
