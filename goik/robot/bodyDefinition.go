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
	"encoding/json"
	"fmt"
	"os"
)

// Bodydefinition contains all parameters necessary to define a pod
type BodyDefinition struct {
	// Number of legs in the robot
	NumLegs int `json:"NumLegs"`
	// Gait pattern (Not all gaits are valide for a given number of legs)
	Gait *Gait
	// Each leg point out from the body of the robot. For a hexapod with an even
	// separation of legs around the robot body, the coxa angles are
	// 0, 60, 120, 180, 240, 300. The corresponding leg indices are 0, 1, 2, 3, 4, 5
	CoxaAngles []float64 `json:"CoxaAngles"`
	// This is the origin of the coxa reference frames (The anchor points of the legs
	// to the robot body in the base reference frame which has an origin of the center of gravity
	// in the robot body)
	CoxaCoordinates []Coordinate `json:"CoxaCoordinates"`
	// Segments contains the length of each segment in the leg (distance between
	// reference frame origins in the kinematic chain)
	Segments []SegmentLengths `json:"Segments"`
	// The angles (in degrees) for a robot in a neutral/rest stance
	RestAngles []ServoAngles `json:"Angles"`
	// Maps joints to physical servos. Nil means the default mapping (see ServoMapping())
	Servos *ServoMapping `json:"Servos,omitempty"`
}

// ServoMapping returns the pod's servo mapping, creating the default mapping if none is defined
func (b *BodyDefinition) ServoMapping() *ServoMapping {
	if b.Servos == nil {
		b.Servos = NewDefaultServoMapping(b.NumLegs)
	}
	return b.Servos
}

// Save saves the current body definition to a file.
func (b *BodyDefinition) Save(filename string) error {
	definition, err := json.Marshal(b)
	if err != nil {
		return err
	}

	fo, err := os.Create(filename)
	if err != nil {
		return err
	}

	defer fo.Close()

	if _, err := fo.Write(definition); err != nil {
		return err
	}
	return nil
}

// Load loads a body definition from a saved definition file.
func (b *BodyDefinition) Load(filename string) (*BodyDefinition, error) {

	buf, err := os.ReadFile(filename)
	if err != nil {
		return nil, err
	}
	if len(buf) == 0 {
		return nil, fmt.Errorf("Zero bytes read")
	}

	var definition BodyDefinition
	err = json.Unmarshal(buf, &definition)
	if err != nil {
		return nil, err
	}

	if definition.Servos != nil {
		if err := definition.Servos.Validate(definition.NumLegs); err != nil {
			return nil, fmt.Errorf("%s: %w", filename, err)
		}
	}

	return &definition, nil
}
