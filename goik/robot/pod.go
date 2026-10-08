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

	"gonum.org/v1/gonum/mat"
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

// SetCoxaLength redefines the length of the coxa leg segment
func (p *Pod) SetCoxaLength(legNum int, length float64) error {
	if legNum > p.BodyDefinition.NumLegs-1 {
		return fmt.Errorf("Unable to modify leg %d. The current body definition only has %d legs", legNum, p.BodyDefinition.NumLegs)
	}

	p.BodyDefinition.Segments[legNum].Coxa = length
	p.UpdatePodStructure()
	return nil
}

// SetFemurLength redefines the length of the femur leg segment
func (p *Pod) SetFemurLength(legNum int, length float64) error {
	if legNum > p.BodyDefinition.NumLegs-1 {
		return fmt.Errorf("Unable to modify leg %d. The current body definition only has %d legs", legNum, p.BodyDefinition.NumLegs)
	}

	p.BodyDefinition.Segments[legNum].Femur = length
	p.UpdatePodStructure()
	return nil
}

// SetTibiaLength redefines the length of the tibia segment
func (p *Pod) SetTibiaLength(legNum int, length float64) error {
	if legNum > p.BodyDefinition.NumLegs-1 {
		return fmt.Errorf("Unable to modify leg %d. The current body definition only has %d legs", legNum, p.BodyDefinition.NumLegs)
	}

	p.BodyDefinition.Segments[legNum].Tibia = length
	p.UpdatePodStructure()
	return nil
}

// SetFemurAngle redefines the angle of the femur joint
func (p *Pod) SetCoxaAngle(legNum int, angle float64) error {
	if legNum > p.BodyDefinition.NumLegs-1 {
		return fmt.Errorf("Unable to modify leg %d. The current body definition only has %d legs", legNum, p.BodyDefinition.NumLegs)
	}

	p.BodyDefinition.RestAngles[legNum].Coxa = angle
	p.UpdatePodStructure()
	return nil
}

// SetFemurAngle redefines the angle of the femur joint
func (p *Pod) SetFemurAngle(legNum int, angle float64) error {
	if legNum > p.BodyDefinition.NumLegs-1 {
		return fmt.Errorf("Unable to modify leg %d. The current body definition only has %d legs", legNum, p.BodyDefinition.NumLegs)
	}

	p.BodyDefinition.RestAngles[legNum].Femur = angle
	p.UpdatePodStructure()
	return nil
}

// SetTibiaAngle redefines the angle of the tibia joint
func (p *Pod) SetTibiaAngle(legNum int, angle float64) error {
	if legNum > p.BodyDefinition.NumLegs-1 {
		return fmt.Errorf("Unable to modify leg %d. The current body definition only has %d legs", legNum, p.BodyDefinition.NumLegs)
	}

	p.BodyDefinition.RestAngles[legNum].Tibia = angle
	p.UpdatePodStructure()
	return nil
}

// UpdatePodStructure recalculates the homogeneous transformation matrix for the coxa offsets
// and creates the robot legs based on coxa, femur and tibia segment lengths, leg separation angles (coxa)
// and rest angles. The legs are reset to the rest angles, so the gait engine is dropped.
func (p *Pod) UpdatePodStructure() {
	// The robot body is flat in the XY plane in the base reference frame.
	for l := 0; l < p.BodyDefinition.NumLegs; l++ {
		// Pod body is described as an inscribed polygon with a radius r (== distance from center of robot)
		// Leg offset Transformation matrix
		OffsetTransformationMatrix := mat.NewDense(4, 4, []float64{
			math.Cos(p.BodyDefinition.CoxaAngles[l] * math.Pi / 180), -math.Sin(p.BodyDefinition.CoxaAngles[l] * math.Pi / 180), 0, p.BodyDefinition.CoxaCoordinates[l].X,
			math.Sin(p.BodyDefinition.CoxaAngles[l] * math.Pi / 180), math.Cos(p.BodyDefinition.CoxaAngles[l] * math.Pi / 180), 0, p.BodyDefinition.CoxaCoordinates[l].Y,
			0, 0, 1, p.BodyDefinition.CoxaCoordinates[l].Z,
			0, 0, 0, 1,
		})

		p.Legs[l] = NewLeg(l,
			p.BodyDefinition.CoxaAngles[l],
			OffsetTransformationMatrix,
			p.BodyDefinition.RestAngles[l],
			p.BodyDefinition.Segments[l],
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

// Ground moves all end effectors to the given height (Z, in the base reference frame) and
// makes the result the new rest stance
func (p *Pod) Ground(height float64) error {
	angles := make([]ServoAngles, len(p.Legs))
	for i, l := range p.Legs {
		a, err := SolveEffectorIK(l, NewCoordinate(l.Joints[EFFECTOR_ORIGIN_INDEX].X, l.Joints[EFFECTOR_ORIGIN_INDEX].Y, height), p.debugChannel)
		if err != nil {
			return fmt.Errorf("leg %d: %w", i, err)
		}
		angles[i] = a
	}
	copy(p.BodyDefinition.RestAngles, angles)
	p.UpdatePodStructure()
	return nil
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
