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
	"sort"
	"strings"
)

// ServoModel describes how joint angles map to raw servo positions for a type of servo
type ServoModel struct {
	Name string
	// Protocol version. Feetech STS servos use a Dynamixel protocol 1.0 compatible framing
	Protocol int
	// Range of motion (in degrees) covered by the raw position values
	RangeDegrees float64
	// Number of raw position values (1024 == 10 bit). The midpoint is the servo's 0 degree position
	Resolution int
	// Control table address of the goal position register
	GoalPositionAddress int
}

// ServoModels contains the supported servo types
var ServoModels = map[string]ServoModel{
	"AX-12A": {Name: "AX-12A", Protocol: 1, RangeDegrees: 300, Resolution: 1024, GoalPositionAddress: 30},
	"XL-320": {Name: "XL-320", Protocol: 2, RangeDegrees: 300, Resolution: 1024, GoalPositionAddress: 30},
	// TODO: Verify against the Feetech STS memory table (goal position address and little endian byte order)
	"STS3215": {Name: "STS3215", Protocol: 1, RangeDegrees: 360, Resolution: 4096, GoalPositionAddress: 42},
}

// DEFAULT_SERVO_MODEL is used for pods without a servo mapping
const DEFAULT_SERVO_MODEL = "AX-12A"

// DEFAULT_BAUD_RATE is the default servo bus speed
const DEFAULT_BAUD_RATE = 1000000

// ServoModelNames returns the names of the supported servo models
func ServoModelNames() []string {
	var names []string
	for name := range ServoModels {
		names = append(names, name)
	}
	sort.Strings(names)
	return names
}

// JointMapping maps one joint to a physical servo
type JointMapping struct {
	// Servo id on the bus
	Id int `json:"Id"`
	// True if the servo horn points in the negative Z direction of the joint's reference frame
	Inverted bool `json:"Inverted"`
	// Added to the joint angle (in degrees) to compensate for how the horn is mounted
	Offset float64 `json:"Offset"`
	// Soft limits in degrees (after offset and inversion). Both 0 means the full servo range
	Min float64 `json:"Min"`
	Max float64 `json:"Max"`
	// CAD export: rotation (degrees) of the servo case around the joint axis. At 0 the case extends
	// back along the link the servo is mounted on (towards the previous joint, or the body for the coxa)
	CaseAngle float64 `json:"CaseAngle,omitempty"`
	// CAD export: distance (mm) from the joint origin to the horn face, along the horn direction
	AxisOffset float64 `json:"AxisOffset,omitempty"`
}

// LegMapping maps the joints of a leg to physical servos
type LegMapping struct {
	Coxa  JointMapping `json:"Coxa"`
	Femur JointMapping `json:"Femur"`
	Tibia JointMapping `json:"Tibia"`
}

// Joint returns the mapping for a joint by name (coxa, femur or tibia)
func (l *LegMapping) Joint(name string) (*JointMapping, error) {
	switch strings.ToLower(name) {
	case "coxa":
		return &l.Coxa, nil
	case "femur":
		return &l.Femur, nil
	case "tibia":
		return &l.Tibia, nil
	}
	return nil, fmt.Errorf("unknown joint '%s' (coxa, femur or tibia)", name)
}

// ServoMapping maps the pod's joints to physical servos. It is used by export and will be used by
// the robot controller, so recordings and live control map angles the same way.
type ServoMapping struct {
	Model    string       `json:"Model"`
	BaudRate int          `json:"BaudRate"`
	Legs     []LegMapping `json:"Legs"`
}

// ServoTarget is a raw goal position for a servo
type ServoTarget struct {
	Id       int
	Position uint16
	// True if the angle was outside the servo range or soft limits and had to be clamped
	Clamped bool
}

// NewDefaultServoMapping creates a mapping using the README addressing scheme
// (leg 0: ids 1, 2, 3, leg 1: ids 4, 5, 6 ...) and no inverted servos
func NewDefaultServoMapping(numLegs int) *ServoMapping {
	m := &ServoMapping{Model: DEFAULT_SERVO_MODEL, BaudRate: DEFAULT_BAUD_RATE}
	for l := 0; l < numLegs; l++ {
		m.Legs = append(m.Legs, LegMapping{
			Coxa:  JointMapping{Id: l*(NUM_JOINTS-1) + 1},
			Femur: JointMapping{Id: l*(NUM_JOINTS-1) + 2},
			Tibia: JointMapping{Id: l*(NUM_JOINTS-1) + 3},
		})
	}
	return m
}

// Validate checks that the mapping matches the pod and uses a known servo model and unique ids
func (m *ServoMapping) Validate(numLegs int) error {
	if _, ok := ServoModels[m.Model]; !ok {
		return fmt.Errorf("unknown servo model '%s'. Supported models: %s", m.Model, strings.Join(ServoModelNames(), ", "))
	}
	if len(m.Legs) != numLegs {
		return fmt.Errorf("servo mapping has %d legs, the pod has %d", len(m.Legs), numLegs)
	}
	used := map[int]string{}
	for l := range m.Legs {
		for _, name := range []string{"coxa", "femur", "tibia"} {
			j, _ := m.Legs[l].Joint(name)
			if j.Id < 0 || j.Id > 253 {
				return fmt.Errorf("leg %d %s: servo id %d is outside 0-253", l, name, j.Id)
			}
			if previous, ok := used[j.Id]; ok {
				return fmt.Errorf("leg %d %s: servo id %d is already used by %s", l, name, j.Id, previous)
			}
			used[j.Id] = fmt.Sprintf("leg %d %s", l, name)
			if j.Min > j.Max {
				return fmt.Errorf("leg %d %s: min limit %2.1f is larger than max limit %2.1f", l, name, j.Min, j.Max)
			}
		}
	}
	return nil
}

// ToRaw converts a joint angle (degrees) to a raw servo position, clamped to the servo range
// and the joint's soft limits
func (m *ServoMapping) ToRaw(j JointMapping, angle float64) (uint16, bool) {
	model := ServoModels[m.Model]
	clamped := false

	a := angle + j.Offset
	if j.Inverted {
		a = -a
	}
	if j.Min != 0 || j.Max != 0 {
		if a < j.Min {
			a, clamped = j.Min, true
		} else if a > j.Max {
			a, clamped = j.Max, true
		}
	}

	raw := math.Round(a/model.RangeDegrees*float64(model.Resolution) + float64(model.Resolution)/2)
	if raw < 0 {
		raw, clamped = 0, true
	} else if raw > float64(model.Resolution-1) {
		raw, clamped = float64(model.Resolution-1), true
	}
	return uint16(raw), clamped
}

// Targets converts the pod's current servo angles to raw goal positions,
// in leg order (coxa, femur, tibia for each leg)
func (m *ServoMapping) Targets(angles []ServoAngles) []ServoTarget {
	targets := make([]ServoTarget, 0, len(angles)*(NUM_JOINTS-1))
	for l, a := range angles {
		legMapping := m.Legs[l]
		for _, joint := range []struct {
			mapping JointMapping
			angle   float64
		}{{legMapping.Coxa, a.Coxa}, {legMapping.Femur, a.Femur}, {legMapping.Tibia, a.Tibia}} {
			raw, clamped := m.ToRaw(joint.mapping, joint.angle)
			targets = append(targets, ServoTarget{Id: joint.mapping.Id, Position: raw, Clamped: clamped})
		}
	}
	return targets
}
