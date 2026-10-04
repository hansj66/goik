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

type Direction int

const (
	Forward Direction = 1
	Reverse Direction = -1
)

// Number of interpolation steps for leg movement (Keep this number an odd number).
const INTERPOLATION_STEPS = 21

// Distance from base reference frame Z to end effector Z when the robot is in the rest / neutral stance
var POD_Z_HEIGHT float64

// Z_LIFT defines the maximum height of the arc described by the end effector when it moves in the swing phase
var Z_LIFT float64 = 50.0

// Z_LIFT defines the maximum height of the arc described by the end effector when it moves back to a neutral / rest position
var REVERT_LIFT float64 = 20

type RevertPhase int

const (
	Ground        RevertPhase = 0
	MoveToNeutral RevertPhase = 1
)

// Pod defines a <n>pod (hexapod, pentapod, heptapod etc)
type Pod struct {
	// Array containing a representation of robot legs
	Legs []*Leg
	// Contains the parameters defining the robot topology
	// Number of legs, rest angles, segment length etc
	BodyDefinition *BodyDefinition
	// True if a target vector or rotation has been defined
	HasDefinedStride bool
	// True if the robot is moving
	IsWalking bool
	// How many gait cycle indices should we go through for the current gait ?
	targetGaitCycles int
	// Which gait cycle are we in
	currentGaitCycle int
	// Index in the pattern cycle of the current gait
	CurrentGaitIndex int
	// True of the pod is in the process of moving the legs back to a neutral/ rest stance
	IsReverting bool
	// Index of the Leg that is currently reverting
	RevertingLegIndex int
	// The process of reverting the legs to neutral position consists of two
	// separate phases (grounding + moving back)
	RevertPhase RevertPhase
	// If IsRecording is true, all changes in angles during movement is recorded
	// in a motion set
	IsRecording bool
	// If recording is true, all angle changes will be recorded in MotionPrimitive
	MotionPrimitive *MotionPrimitive
	// direction specifies forward/reverse in the direction of the stride vector
	// or clockwise/anticlockwise for rotation
	direction Direction
	// tick indicates the total number of movement update ticks
	tick         int
	debugChannel chan string
	// Engine is the phase based gait engine. When set, it replaces the table based gait in Update()
	Engine *GaitEngine
}

func (p *Pod) SetDirection(direction Direction) {
	p.direction = direction
}

func (p *Pod) ReverseDirection() {
	if p.direction == Forward {
		p.direction = Reverse
	} else {
		p.direction = Forward
	}
}

func (p *Pod) GetTick() int {
	return p.tick
}

func (p *Pod) ResetTicks() {
	p.tick = 0
}

func (p *Pod) GetCurrentGaitCycle() int {
	return p.currentGaitCycle
}

// SetDebugChannel sets the output channel for debug messages
func (p *Pod) SetDebugChannel(channel chan string) {
	p.debugChannel = channel
}

func (p *Pod) Debug(msg string) {
	p.debugChannel <- msg
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

// Update pod recaluclates the homogeneous transformation matrix for the coxa offsets
// And create the robot legs based on coxa, femur and tibia segment lengths, leg separation angles (coxa)
// and femur and tibia rest angles
func (p *Pod) UpdatePodStructure() {
	// The robot body is flat in the XY plane in the base reference frame.
	for l := 0; l < p.BodyDefinition.NumLegs; l++ {
		// Pod body is described as an inscribed polygon with a radius r (== distance from center of robot)
		// Leg offset Transformation matrix
		OffsetTransformationMatrix := mat.NewDense(4, 4, []float64{
			math.Cos(p.BodyDefinition.CoxaAngles[l] * math.Pi / 180), -math.Sin(p.BodyDefinition.CoxaAngles[l] * math.Pi / 180), 0, p.BodyDefinition.CoxaCoordinates[l].X,
			math.Sin(p.BodyDefinition.CoxaAngles[l] * math.Pi / 180), math.Cos(p.BodyDefinition.CoxaAngles[l] * math.Pi / 180), 0, p.BodyDefinition.CoxaCoordinates[l].Y,
			0, 0, 1, p.BodyDefinition.CoxaCoordinates[l].Z * math.Pi / 360,
			0, 0, 0, 1,
		})

		servoIds := [NUM_JOINTS - 1]int{l*(NUM_JOINTS-1) + 1, l*(NUM_JOINTS-1) + 2, l*(NUM_JOINTS-1) + 3}
		p.Legs[l] = NewLeg(l,
			p.BodyDefinition.CoxaAngles[l],
			OffsetTransformationMatrix,
			p.BodyDefinition.RestAngles[l],
			p.BodyDefinition.Segments[l],
			servoIds,
			p.debugChannel)
	}
}

func (p *Pod) LoadBodyDefinition(BodyDefinition *BodyDefinition) {
	p.Legs = make([]*Leg, BodyDefinition.NumLegs)
	p.direction = Forward
	p.MotionPrimitive = NewMotionPrimitive()
	p.BodyDefinition = BodyDefinition
	// The gait engine holds per leg state, so it has to be recreated for a new body
	p.Engine = nil

	p.UpdatePodStructure()
}

// NewPod creates a new pod from a bodydefinition and implicitly triggers
// the forward kinematic calculations necessary for visualizing the pod
func NewPod(BodyDefinition *BodyDefinition) *Pod {
	p := Pod{}
	p.LoadBodyDefinition(BodyDefinition)

	POD_Z_HEIGHT = p.Legs[0].Joints[EFFECTOR_ORIGIN_INDEX].Z

	return &p
}

// GetEndEffectorPositions retrieves the current coordinates for the
// end effectors of the pod
func (p *Pod) GetEndEffectorPositions() []Coordinate {
	positions := make([]Coordinate, p.BodyDefinition.NumLegs)
	for i, _ := range p.Legs {
		positions[i] = p.Legs[i].Joints[3]
	}
	return positions
}

func (p *Pod) AddTargetGaitCycles(nrepeats int) {
	p.targetGaitCycles = p.targetGaitCycles + nrepeats
}

// SetStrideVector sets up the path of a single step and calculates
// the series of intermediate angles necessary to complete a step
// in the direction of this vector with a stride length equal to
// the length of the vector
func (p *Pod) SetStrideVector(nrepeats int, x float64, y float64) error {
	p.targetGaitCycles = nrepeats

	for l, leg := range p.Legs {
		ee := leg.Joints[EFFECTOR_ORIGIN_INDEX]

		xMax := ee.X + x
		xMin := ee.X - x
		yMax := ee.Y + y
		yMin := ee.Y - y

		// We need this to visualize the end effector trajectory in the simulator
		xStep := (xMax - xMin) / (INTERPOLATION_STEPS - 1)
		yStep := (yMax - yMin) / (INTERPOLATION_STEPS - 1)

		deltaX := 0.0
		deltaY := 0.0
		for i := 0; i < INTERPOLATION_STEPS; i++ {
			swing := NewCoordinate(xMin+deltaX, yMin+deltaY, POD_Z_HEIGHT)
			servoAngles, err := SolveEffectorIK(p.Legs[l], swing, p.debugChannel)
			if err != nil {
				return err
			}
			leg.IntermediateAngles.JointAngle[0][i] = servoAngles.Coxa
			leg.IntermediateAngles.JointAngle[1][i] = servoAngles.Femur
			leg.IntermediateAngles.JointAngle[2][i] = servoAngles.Tibia

			leg.IntermediateEffectorCoordinates[i] = NewCoordinate(deltaX+xMin, deltaY+yMin, 0)

			deltaX += xStep
			deltaY += yStep
		}

	}

	p.HasDefinedStride = true
	return nil
}

// SetRotation is similar to SetStrideVector, but instead of having the
// end effector following a vector, it will follow a curve segment
func (p *Pod) SetRotation(nrepeats int, degrees float64) error {
	p.targetGaitCycles = nrepeats

	for l, leg := range p.Legs {
		ee := leg.Joints[EFFECTOR_ORIGIN_INDEX]

		radius := math.Sqrt(ee.X*ee.X + ee.Y*ee.Y)
		stepRadians := (degrees * math.Pi / 360) / (INTERPOLATION_STEPS - 1)
		angle := math.Atan2(ee.Y, ee.X) - 0.5*degrees*math.Pi/360
		// angle := math.Atan2(ee.Y, ee.X) - degrees*math.Pi/360

		var delta = 0.0

		for i := 0; i < INTERPOLATION_STEPS; i++ {

			x := radius * math.Cos(angle+delta)
			y := radius * math.Sin(angle+delta)
			swing := NewCoordinate(x, y, POD_Z_HEIGHT)
			servoAngles, err := SolveEffectorIK(p.Legs[l], swing, p.debugChannel)
			if err != nil {
				return err
			}
			leg.IntermediateAngles.JointAngle[0][i] = servoAngles.Coxa
			leg.IntermediateAngles.JointAngle[1][i] = servoAngles.Femur
			leg.IntermediateAngles.JointAngle[2][i] = servoAngles.Tibia

			leg.IntermediateEffectorCoordinates[i] = NewCoordinate(x, y, 0)

			delta += stepRadians
		}
	}

	p.HasDefinedStride = true
	return nil
}

// Start allows any calls to Update() to start cycling through the current gait pattern
func (p *Pod) Start() error {
	if !p.HasDefinedStride {
		return fmt.Errorf("no target / stride has been defined")
	}
	p.IsWalking = true
	return nil
}

// Stop will halt the gait cycle
func (p *Pod) Stop() {
	p.targetGaitCycles = p.currentGaitCycle
	p.CurrentGaitIndex = 0
}

// ResetInterpolator reset both the swing and stance interpolation indices
// for all ltegs to the center of their respective interpolation tables
func (p *Pod) ResetInterpolator() {
	for _, l := range p.Legs {
		l.ResetInterpolator()
	}
}

// IsSwingPhase returns true if the leg with index == legIndex is currently in the swing phase
func (p *Pod) IsSwingPhase(legIndex int) bool {
	if p.Engine != nil {
		return p.Engine.Legs[legIndex].Swinging
	}
	return (*p.BodyDefinition.Gait.Pattern)[legIndex][p.CurrentGaitIndex] == 1
}

// UpdateMovement cycles through the set of legs, updates swing and stance interpolation indices,
// recalculates the end effector targets for all legs and solves the inverse kinematic equations
// necessary for mirroring the simulated movement with physical servos
func (p *Pod) UpdateMovement() {

	// // Bail out after we have reached the target number of cycles through the gait pattern
	// // (unless we have specified 0 repetition count. We will then have to stop it using
	// // the "stop command" - and use "revert" to get back to the starting pose)
	// if p.tick >= INTERPOLATION_STEPS*p.targetGaitCycles && p.targetGaitCycles != 0 {
	// 	return
	// }

	var endOfSwing bool
	for i, l := range p.Legs {
		if (*p.BodyDefinition.Gait.Pattern)[i][p.CurrentGaitIndex] == 1 {
			// Swing phase
			endOfSwing = l.UpdateSwing(p.direction)
		} else {
			// Stance phase
			l.UpdateStance(p.direction, p.BodyDefinition.Gait.StanceReturnSpeedFactor)
		}
	}
	// The folling logic is somewhat wonky...
	if endOfSwing && p.direction == Forward {
		p.CurrentGaitIndex += 1
	} else if endOfSwing && p.direction == Reverse {
		p.CurrentGaitIndex -= 1
	}

	p.tick += 1

	// If we are finished with the current index/column in the gait pattern table
	// we proceed to the next. A "cycle" is one full iteration through this table
	// If we reach the target number of cycles, the pod will stop moving
	if p.direction == Forward {
		if p.CurrentGaitIndex >= p.BodyDefinition.Gait.NumIndicesInPattern {
			p.CurrentGaitIndex = 0
			p.currentGaitCycle += 1
			if p.currentGaitCycle > p.targetGaitCycles && p.targetGaitCycles != 0 {
				p.IsWalking = false
			}
		}
	} else if p.direction == Reverse {
		if p.CurrentGaitIndex < 0 {
			p.CurrentGaitIndex = p.BodyDefinition.Gait.NumIndicesInPattern - 1
			p.currentGaitCycle += 1
			if p.currentGaitCycle > p.targetGaitCycles && p.targetGaitCycles != 0 {
				p.IsWalking = false
			}
		}
	}

	// We can use the "record" command from the command line to record all movement
	// and save it as a primitive that we can store on the robot's file system
	// This primitive can later be activated with commands over the network
	if p.IsRecording {
		for _, l := range p.Legs {
			p.MotionPrimitive.Add(l.ServoAngles)
		}
	}
}

// UpdateRevertingToNeutral moves a leg a step closer to the neutral position
// One leg is updated at a time. Once one leg is back to the neutral / rest position,
// the next leg is moved. This continues until the robot is in the neutral / rest stance
func (p *Pod) UpdateRevertingToNeutral() {

	if !p.IsReverting {
		return
	}

	if p.RevertingLegIndex >= p.BodyDefinition.NumLegs {
		p.IsReverting = false
		p.RevertingLegIndex = 0
		p.HasDefinedStride = false
		p.RevertPhase = Ground
		return
	}

	if p.RevertPhase == Ground {
		// Ground all legs simultaneously
		for _, l := range p.Legs {
			l.UpdateRevertPhase0()
			if p.IsRecording {
				p.MotionPrimitive.Add(l.ServoAngles)

				p.debugChannel <- fmt.Sprintf("Should really record leg %d", l.Index)
			}

		}
		p.RevertPhase = MoveToNeutral
	} else if p.RevertPhase == MoveToNeutral {
		// Then move back to the rest position, one leg at a time
		p.RevertingLegIndex = p.Legs[p.RevertingLegIndex].UpdateRevertPhase1()
		if p.IsRecording {
			for _, l := range p.Legs {
				p.MotionPrimitive.Add(l.ServoAngles)
			}
		}
	}
}

// Each call to update will increment the interpolation indices through either
// the gait cycle or the revert cycle (depending on the current state of the
// robot)
func (p *Pod) Update() {
	if p.Engine != nil {
		p.updateEngine()
		return
	}

	if p.IsWalking && !p.IsReverting {
		if p.targetGaitCycles == 0 || (p.currentGaitCycle < p.targetGaitCycles) {
			p.UpdateMovement()
		} else {
			p.Stop()
		}
	}

	if p.HasDefinedStride && p.IsReverting {
		p.UpdateRevertingToNeutral()
	}
}

// updateEngine advances the gait engine one tick
func (p *Pod) updateEngine() {
	p.Engine.Tick(ENGINE_DT)
	if p.Engine.IsIdle() {
		return
	}

	p.tick += 1
	p.CurrentGaitIndex = p.Engine.PatternIndex(p.BodyDefinition.Gait.NumIndicesInPattern)

	if p.IsRecording {
		for _, l := range p.Legs {
			p.MotionPrimitive.Add(l.ServoAngles)
		}
	}
}

// RevertToNutral reverts all legs back to neutral / rest position
func (p *Pod) RevertToNutral() {
	if p.HasDefinedStride {
		p.IsReverting = true
		p.IsWalking = false
	}
	for _, l := range p.Legs {
		l.RevertToNutral()
	}
}

// ClearPrimitives purges all recorded data
func (p *Pod) ClearPrimitives() {
	p.MotionPrimitive.Clear()
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

// Zero resets all servo angles in the robot to 0 degrees.
// This should result in the pod having all legs stretched
// out and each leg forming a straight line away from the robot body
// If it does not, you will have to mechanically adjust the robot body
// so that this condition is satisfied.
// This is a prerequisit for the FK/IK math to make sense in meat space ;)
func (p *Pod) Zero() {
	for _, l := range p.Legs {
		l.Zero()
	}
}
