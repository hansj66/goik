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
	The stance sets the femur and tibia rest angles from where the feet should be: how high the body stands
	above the ground (the height of the coxa and femur joints), and how far out each foot stands (the reach,
	measured horizontally from the femur joint). By default the tibia stands vertical, so the reach follows
	from the height. The coxa rest angles are not affected.

	While a pod has a stance, the femur and tibia rest angles are derived: changing a segment length or the
	design recomputes them, so the body keeps its height.
*/

// Stance describes the pod's rest stance
type Stance struct {
	// Height of the body (the coxa and femur joints) above the ground (mm)
	Height float64 `json:"Height"`
	// Horizontal distance from the femur joint to the foot (mm). 0: the tibia stands vertical
	Reach float64 `json:"Reach,omitempty"`
	// Reach per leg (mm), overriding Reach. 0: use Reach. Reset when the number of legs changes
	LegReach []float64 `json:"LegReach,omitempty"`
}

// The foot may be at most this fraction of femur + tibia from the femur joint, so the leg isn't locked straight
const STANCE_MAX_STRETCH = 0.95

// The feet must be able to step at least this far (mm) around the neutral stance, with the default step height
const STANCE_MIN_STEP = 20.0

// Resolution (mm) of the height range search
const STANCE_HEIGHT_STEP = 5.0

// Clone returns a deep copy of the stance
func (s *Stance) Clone() *Stance {
	c := *s
	c.LegReach = append([]float64(nil), s.LegReach...)
	return &c
}

// LegReachSetting returns the reach set for a leg (0: the tibia stands vertical)
func (s *Stance) LegReachSetting(leg int) float64 {
	if leg < len(s.LegReach) && s.LegReach[leg] > 0 {
		return s.LegReach[leg]
	}
	return s.Reach
}

// StanceRestAngles computes the rest angles that put the feet where the body definition's stance says,
// and checks that the stance is possible: the feet within reach, the knees above the feet, the joint angles
// within the servos' range and soft limits, and room for the feet to step.
func StanceRestAngles(b *BodyDefinition) ([]ServoAngles, error) {
	s := b.Stance
	if s == nil {
		return nil, fmt.Errorf("the pod has no stance")
	}
	if s.Height < MIN_GROUND_CLEARANCE {
		return nil, fmt.Errorf("the body must be at least %.0f mm above the ground", MIN_GROUND_CLEARANCE)
	}
	if s.Reach < 0 {
		return nil, fmt.Errorf("the reach can not be negative")
	}
	if len(s.LegReach) != 0 && len(s.LegReach) != b.NumLegs {
		return nil, fmt.Errorf("the stance has a reach for %d legs, the pod has %d", len(s.LegReach), b.NumLegs)
	}

	mapping := b.Servos
	if mapping == nil {
		mapping = NewDefaultServoMapping(b.NumLegs)
	}
	checkRange := mapping.Validate(b.NumLegs) == nil

	h := s.Height
	angles := make([]ServoAngles, b.NumLegs)
	for l := range angles {
		f, t := b.Segments[l].Femur, b.Segments[l].Tibia
		r := s.LegReachSetting(l)
		if r == 0 {
			// Tibia vertical: the knee is a tibia length above the foot
			knee := h - t
			if math.Abs(knee) >= f {
				return nil, fmt.Errorf("leg %d: the tibia (%.0f mm) can't stand vertical at a height of %.0f mm with a %.0f mm femur (only between %.0f and %.0f mm, not at the ends). Give a reach (stance <height> <reach>)",
					l, t, h, f, math.Max(t-f, 0), t+f)
			}
			r = math.Sqrt(f*f - knee*knee)
		}

		d := math.Hypot(r, h)
		if d > STANCE_MAX_STRETCH*(f+t) {
			return nil, fmt.Errorf("leg %d: the foot would be %.1f mm from the femur joint, but the leg only reaches %.1f mm (%.0f%% of femur + tibia, so it isn't locked straight)",
				l, d, STANCE_MAX_STRETCH*(f+t), 100*STANCE_MAX_STRETCH)
		}
		if d <= math.Abs(f-t) {
			return nil, fmt.Errorf("leg %d: the foot would be %.1f mm from the femur joint, but the leg can't fold closer than %.1f mm", l, d, math.Abs(f-t))
		}

		// The same equations as SolveEffectorIK, in the leg's plane (r outwards, h down)
		a1 := math.Acos(h / d)
		a2 := math.Acos((t*t - f*f - d*d) / (-2 * f * d))
		femur := 90 - (a1+a2)*180/math.Pi
		tibia := 180 - math.Acos((d*d-f*f-t*t)/(-2*t*f))*180/math.Pi
		kneeZ := f * math.Sin(femur*math.Pi/180) // femur angles are positive downwards

		// A twisted leg isn't planar: solve it numerically, starting from the untwisted solution
		if twists := b.LegTwists(l); !twists.IsZero() {
			var err error
			if femur, tibia, kneeZ, err = twistedStance(b, l, h, r, femur, tibia); err != nil {
				return nil, err
			}
		}
		angles[l] = ServoAngles{Coxa: b.RestAngles[l].Coxa, Femur: femur, Tibia: tibia}

		if kneeZ >= h {
			return nil, fmt.Errorf("leg %d: the knee would be at or below the foot", l)
		}

		if checkRange {
			for _, joint := range []struct {
				name  string
				angle float64
			}{{"femur", femur}, {"tibia", tibia}} {
				j, _ := mapping.Legs[l].Joint(joint.name)
				if _, clamped := mapping.ToRaw(*j, joint.angle); clamped {
					return nil, fmt.Errorf("leg %d: the %s angle (%.1f degrees) is outside the %s's range or the joint's soft limits", l, joint.name, joint.angle, mapping.Model)
				}
			}
		}
	}

	// Room to walk
	test := *b
	test.RestAngles = angles
	test.Stance = nil
	if step := NewPod(&test).ReachRadius(Z_LIFT); step < STANCE_MIN_STEP {
		return nil, fmt.Errorf("at a height of %.0f mm the feet can only step %.0f mm around the neutral stance (at least %.0f mm, with the default step height of %.0f mm)",
			h, step, STANCE_MIN_STEP, Z_LIFT)
	}
	return angles, nil
}

// twistedStance returns the femur and tibia angles (degrees) that put a twisted leg's foot h below the body
// plane and reach out from the femur joint (horizontally), with the coxa at its rest angle, and the knee's Z.
// The femur and tibia angles of the untwisted leg are the starting point.
func twistedStance(b *BodyDefinition, leg int, h float64, reach float64, femur float64, tibia float64) (float64, float64, float64, error) {
	const toRadians = math.Pi / 180
	m := newLegModel(b.CoxaCoordinates[leg], b.CoxaAngles[leg], b.Segments[leg], b.LegTwists(leg))
	q := [3]float64{b.RestAngles[leg].Coxa * toRadians, femur * toRadians, tibia * toRadians}

	// Newton's method on two equations: foot height - h = 0 and horizontal distance from the femur joint - reach = 0
	for i := 0; i < IK_MAX_ITERATIONS; i++ {
		joints, axes := m.forward(q)
		foot, femurJoint := joints[3], joints[1]
		out := vec3{foot[0] - femurJoint[0], foot[1] - femurJoint[1], 0}
		distance := out.length()
		r1, r2 := foot[2]-h, distance-reach
		if math.Hypot(r1, r2) < IK_TOLERANCE {
			return q[1] / toRadians, q[2] / toRadians, joints[2][2], nil
		}
		if distance < 1e-9 {
			break
		}

		// How the foot moves when the femur (j = 1) and the tibia (j = 2) turn
		var J [2][2]float64
		for j := 1; j <= 2; j++ {
			d := axes[j].cross(foot.sub(joints[j]))
			J[0][j-1] = d[2]
			J[1][j-1] = (out[0]*d[0] + out[1]*d[1]) / distance
		}
		det := J[0][0]*J[1][1] - J[0][1]*J[1][0]
		if math.Abs(det) < 1e-9 {
			break
		}
		dq := [2]float64{-(J[1][1]*r1 - J[0][1]*r2) / det, -(-J[1][0]*r1 + J[0][0]*r2) / det}
		if largest := math.Max(math.Abs(dq[0]), math.Abs(dq[1])); largest > IK_MAX_STEP {
			dq[0], dq[1] = dq[0]*IK_MAX_STEP/largest, dq[1]*IK_MAX_STEP/largest
		}
		q[1] += dq[0]
		q[2] += dq[1]
	}
	return 0, 0, 0, fmt.Errorf("leg %d: with its twisted joints, no femur and tibia angles put the foot %.0f mm below the body and %.0f mm out from the femur joint", leg, h, reach)
}

// StanceHeightRange returns the lowest and highest body heights (in steps of STANCE_HEIGHT_STEP) the body
// definition's stance works at, with its reach settings. ok is false if no height works.
func StanceHeightRange(b *BodyDefinition) (lowest float64, highest float64, ok bool) {
	if b.Stance == nil {
		return 0, 0, false
	}
	longest := 0.0
	for _, s := range b.Segments {
		longest = math.Max(longest, s.Femur+s.Tibia)
	}

	test := *b
	for h := STANCE_HEIGHT_STEP; h <= longest; h += STANCE_HEIGHT_STEP {
		s := b.Stance.Clone()
		s.Height = h
		test.Stance = s
		if _, err := StanceRestAngles(&test); err != nil {
			continue
		}
		if !ok {
			lowest, ok = h, true
		}
		highest = h
	}
	return lowest, highest, ok
}

// applyStance sets the body definition's stance and the rest angles it gives. Its design (if any) gets the
// new rest angles too. The body definition is unchanged if the stance is not possible.
func applyStance(b *BodyDefinition, s *Stance) error {
	test := *b
	test.Stance = s
	angles, err := StanceRestAngles(&test)
	if err != nil {
		if lowest, highest, ok := StanceHeightRange(&test); ok {
			return fmt.Errorf("%w. Possible heights with this reach: %.0f to %.0f mm", err, lowest, highest)
		}
		return fmt.Errorf("%w. No height works with this reach", err)
	}

	if b.Design != nil {
		d := b.Design.Clone()
		for l, a := range angles {
			if err := d.SetLeg(l, b.Segments[l], a, b.LegTwists(l)); err != nil {
				return err
			}
		}
		b.Design = d
	}
	b.Stance = s
	b.RestAngles = angles
	return nil
}

// SetStance sets the pod's stance and the femur and tibia rest angles it gives. nil removes the stance, and
// the rest angles stay as they are. The pod is unchanged if the stance is not possible.
func (p *Pod) SetStance(s *Stance) error {
	if s == nil {
		p.BodyDefinition.Stance = nil
		return nil
	}
	b := *p.BodyDefinition
	if err := applyStance(&b, s); err != nil {
		return err
	}
	p.LoadBodyDefinition(&b)
	return nil
}
