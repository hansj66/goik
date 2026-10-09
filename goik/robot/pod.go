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

// Z_LIFT is the default maximum height (in mm) of the arc described by the end effector in the swing phase
var Z_LIFT float64 = 50.0

// Pod defines a <n>pod (hexapod, pentapod, heptapod etc)
type Pod struct {
	// Array containing a representation of robot legs
	Legs []*Leg
	// Contains the parameters defining the robot topology
	// Number of legs, rest angles, segment length etc
	BodyDefinition *BodyDefinition
	// Column of the gait pattern table matching the gait engine's clock (used for visualization)
	CurrentGaitIndex int
	debugChannel     chan string
	// Engine is the gait engine moving the legs. It is created on demand (see NewGaitEngine)
	// and dropped whenever the pod's structure changes, since it holds per leg state.
	Engine *GaitEngine
}

// SetDebugChannel sets the output channel for debug messages
func (p *Pod) SetDebugChannel(channel chan string) {
	p.debugChannel = channel
}

// setLeg changes the segment lengths, rest angles and joint twists of a leg. When the pod has a design, the
// change is made to the design, so the leg's mirror image changes with it, and the pod is rebuilt from the
// design. When the pod has a stance, the stance sets the femur and tibia rest angles. The pod is unchanged if
// the change fails.
func (p *Pod) setLeg(legNum int, change func(segments *SegmentLengths, rest *ServoAngles, twists *JointTwists)) error {
	b := p.BodyDefinition
	if legNum < 0 || legNum >= b.NumLegs {
		return fmt.Errorf("Unable to modify leg %d. The current body definition only has %d legs", legNum, b.NumLegs)
	}

	segments, rest, twists := b.Segments[legNum], b.RestAngles[legNum], b.LegTwists(legNum)
	change(&segments, &rest, &twists)
	if err := twists.Validate(); err != nil {
		return err
	}
	if b.Stance != nil && (rest.Femur != b.RestAngles[legNum].Femur || rest.Tibia != b.RestAngles[legNum].Tibia) {
		return fmt.Errorf("the pod's stance sets the femur and tibia rest angles. Change the stance ('stance <height> [reach]') or remove it ('stance off')")
	}

	if b.Design != nil {
		d := b.Design.Clone()
		if err := d.SetLeg(legNum, segments, rest, twists); err != nil {
			return err
		}
		return p.ApplyDesign(d)
	}

	changed := *b
	changed.Segments = append([]SegmentLengths(nil), b.Segments...)
	changed.RestAngles = append([]ServoAngles(nil), b.RestAngles...)
	changed.Twists = make([]JointTwists, b.NumLegs)
	for l := range changed.Twists {
		changed.Twists[l] = b.LegTwists(l)
	}
	changed.Segments[legNum], changed.RestAngles[legNum], changed.Twists[legNum] = segments, rest, twists
	if !changed.HasTwists() {
		changed.Twists = nil
	}
	if changed.Stance != nil {
		if err := applyStance(&changed, changed.Stance); err != nil {
			return fmt.Errorf("not possible with the pod's stance (change the stance, or remove it with 'stance off'): %w", err)
		}
	}
	p.LoadBodyDefinition(&changed)
	return nil
}

// SetCoxaLength redefines the length of the coxa leg segment
func (p *Pod) SetCoxaLength(legNum int, length float64) error {
	return p.setLeg(legNum, func(s *SegmentLengths, _ *ServoAngles, _ *JointTwists) { s.Coxa = length })
}

// SetFemurLength redefines the length of the femur leg segment
func (p *Pod) SetFemurLength(legNum int, length float64) error {
	return p.setLeg(legNum, func(s *SegmentLengths, _ *ServoAngles, _ *JointTwists) { s.Femur = length })
}

// SetTibiaLength redefines the length of the tibia segment
func (p *Pod) SetTibiaLength(legNum int, length float64) error {
	return p.setLeg(legNum, func(s *SegmentLengths, _ *ServoAngles, _ *JointTwists) { s.Tibia = length })
}

// SetCoxaAngle redefines the rest angle of the coxa joint
func (p *Pod) SetCoxaAngle(legNum int, angle float64) error {
	return p.setLeg(legNum, func(_ *SegmentLengths, a *ServoAngles, _ *JointTwists) { a.Coxa = angle })
}

// SetFemurAngle redefines the rest angle of the femur joint
func (p *Pod) SetFemurAngle(legNum int, angle float64) error {
	return p.setLeg(legNum, func(_ *SegmentLengths, a *ServoAngles, _ *JointTwists) { a.Femur = angle })
}

// SetTibiaAngle redefines the rest angle of the tibia joint
func (p *Pod) SetTibiaAngle(legNum int, angle float64) error {
	return p.setLeg(legNum, func(_ *SegmentLengths, a *ServoAngles, _ *JointTwists) { a.Tibia = angle })
}

// SetCoxaTwist sets the twist (degrees) of the coxa axis about the mount direction (see twist.go)
func (p *Pod) SetCoxaTwist(legNum int, twist float64) error {
	return p.setLeg(legNum, func(_ *SegmentLengths, _ *ServoAngles, t *JointTwists) { t.Coxa = twist })
}

// SetFemurTwist sets the twist (degrees) of the femur axis about the coxa (see twist.go)
func (p *Pod) SetFemurTwist(legNum int, twist float64) error {
	return p.setLeg(legNum, func(_ *SegmentLengths, _ *ServoAngles, t *JointTwists) { t.Femur = twist })
}

// SetTibiaTwist sets the twist (degrees) of the tibia axis about the femur (see twist.go)
func (p *Pod) SetTibiaTwist(legNum int, twist float64) error {
	return p.setLeg(legNum, func(_ *SegmentLengths, _ *ServoAngles, t *JointTwists) { t.Tibia = twist })
}

// ApplyDesign rebuilds the pod from a design, keeping the gait type and the servo mapping when they fit the
// new number of legs (see PodDesign.BodyDefinition), and the stance. The pod is unchanged if the design is
// invalid or the stance isn't possible with it.
func (p *Pod) ApplyDesign(d *PodDesign) error {
	b, err := d.BodyDefinition(p.BodyDefinition)
	if err != nil {
		return err
	}
	if s := p.BodyDefinition.Stance; s != nil {
		s = s.Clone()
		if b.NumLegs != p.BodyDefinition.NumLegs {
			s.LegReach = nil
		}
		if err := applyStance(b, s); err != nil {
			return fmt.Errorf("not possible with the pod's stance (change the stance, or remove it with 'stance off'): %w", err)
		}
	}
	p.LoadBodyDefinition(b)
	return nil
}

// UpdatePodStructure recalculates the homogeneous transformation matrix for the coxa offsets
// and creates the robot legs based on coxa, femur and tibia segment lengths, leg separation angles (coxa)
// and rest angles. The legs are reset to the rest angles, so the gait engine is dropped.
func (p *Pod) UpdatePodStructure() {
	// The robot body is flat in the XY plane in the base reference frame.
	for l := 0; l < p.BodyDefinition.NumLegs; l++ {
		// Leg offset transformation matrix: where the coxa joint is, and which way the leg points
		twists := p.BodyDefinition.LegTwists(l)
		OffsetTransformationMatrix := MountMatrix(p.BodyDefinition.CoxaCoordinates[l], p.BodyDefinition.CoxaAngles[l], twists.Coxa)

		p.Legs[l] = NewLeg(l,
			p.BodyDefinition.CoxaAngles[l],
			OffsetTransformationMatrix,
			p.BodyDefinition.RestAngles[l],
			p.BodyDefinition.Segments[l],
			twists,
			p.debugChannel)
	}
	p.Engine = nil
}

func (p *Pod) LoadBodyDefinition(BodyDefinition *BodyDefinition) {
	p.Legs = make([]*Leg, BodyDefinition.NumLegs)
	p.BodyDefinition = BodyDefinition

	p.UpdatePodStructure()
}

// NewPod creates a new pod from a bodydefinition and implicitly triggers
// the forward kinematic calculations necessary for visualizing the pod
func NewPod(BodyDefinition *BodyDefinition) *Pod {
	p := Pod{}
	p.LoadBodyDefinition(BodyDefinition)
	return &p
}

// GetEndEffectorPositions retrieves the current coordinates for the
// end effectors of the pod (in the body's reference frame)
func (p *Pod) GetEndEffectorPositions() []Coordinate {
	positions := make([]Coordinate, p.BodyDefinition.NumLegs)
	for i := range p.Legs {
		positions[i] = p.Legs[i].Joints[EFFECTOR_ORIGIN_INDEX]
	}
	return positions
}

// GroundJoints returns the joint positions of all legs in the ground reference frame,
// i.e. with the body pose (pitch, roll, yaw and height) applied. Use this for visualization.
func (p *Pod) GroundJoints() [][NUM_JOINTS]Coordinate {
	joints := make([][NUM_JOINTS]Coordinate, len(p.Legs))
	for l, leg := range p.Legs {
		for j, c := range leg.Joints {
			if p.Engine != nil {
				c = p.Engine.ToGround(c)
			}
			joints[l][j] = c
		}
	}
	return joints
}

// IsSwingPhase returns true if the leg with index == legIndex is currently in the swing phase
func (p *Pod) IsSwingPhase(legIndex int) bool {
	return p.Engine != nil && p.Engine.Legs[legIndex].Swinging
}

// GaitCycles returns the number of gait cycles the engine has completed
func (p *Pod) GaitCycles() float64 {
	if p.Engine == nil {
		return 0
	}
	return p.Engine.Cycles()
}

// Update advances the gait engine (if any) one tick
func (p *Pod) Update() {
	if p.Engine == nil {
		return
	}

	p.Engine.Tick(ENGINE_DT)
	if p.Engine.IsIdle() {
		return
	}

	p.CurrentGaitIndex = p.Engine.PatternIndex(p.BodyDefinition.Gait.NumIndicesInPattern)
}

// ReachRadius returns the largest radius (in mm, 5 mm resolution) around the neutral end effector
// positions that every leg can reach, both on the ground and with the foot lifted by lift mm.
func (p *Pod) ReachRadius(lift float64) float64 {
	const directions = 16
	reach := 0.0
	for r := 5.0; r <= 200; r += 5 {
		for _, l := range p.Legs {
			n := l.NeutralEffectorCoordinate
			for d := 0; d < directions; d++ {
				a := 2 * math.Pi * float64(d) / directions
				for _, z := range []float64{n.Z, n.Z - lift} {
					if _, err := SolveEffectorIK(l, NewCoordinate(n.X+r*math.Cos(a), n.Y+r*math.Sin(a), z), p.debugChannel); err != nil {
						return reach
					}
				}
			}
		}
		reach = r
	}
	return reach
}
