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
	"os"
)

// A MotionPrimitive consists of a set of joint motion sequences
type MotionPrimitive struct {
	rawAngles []ServoAngles
}

func NewMotionPrimitive() *MotionPrimitive {
	return &MotionPrimitive{}
}

func (m *MotionPrimitive) Add(angles ServoAngles) {
	m.rawAngles = append(m.rawAngles, angles)
}

func (m *MotionPrimitive) Size() int {
	return len(m.rawAngles)
}

func (m *MotionPrimitive) Clear() {
	m.rawAngles = nil
}

// Export writes the recording as raw servo positions (2 bytes, little endian, per servo in leg
// order coxa, femur, tibia). The frames contain numLegs legs each. Returns the number of clamped values.
func (m *MotionPrimitive) Export(path string, mapping *ServoMapping, numLegs int) (int, error) {
	var data []byte
	clamped := 0
	for frame := 0; frame+numLegs <= len(m.rawAngles); frame += numLegs {
		for _, t := range mapping.Targets(m.rawAngles[frame : frame+numLegs]) {
			data = append(data, uint8(t.Position&0xFF), uint8(t.Position>>8))
			if t.Clamped {
				clamped++
			}
		}
	}
	return clamped, os.WriteFile(path, data, 0644)
}
