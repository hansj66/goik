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

/*
	Notes regarding the CAD export

	The export describes the pod in its rest pose as a JSON "assembly": where every servo goes and
	how it is oriented, plus the points needed for simple placeholder parts. cad/goik_cad.py turns
	it into a STEP assembly with CadQuery (STEP is far too complex to write from Go).

	Coordinates are converted to the usual CAD convention: millimetres, Z up. GOIK has +Z towards the
	ground, so everything is rotated 180 degrees around X: (x, y, z) -> (x, -y, -z). This keeps the
	frames right handed.

	Servo frame (the frame each servo model is described in by cad/servos.json): origin on the horn
	axis at the outer face of the horn, +Z pointing out of the horn, and the case extending along -X.
*/

import (
	"encoding/json"
	"fmt"
	"math"
	"os"

	"gonum.org/v1/gonum/mat"
)

const CAD_ASSEMBLY_FORMAT = "goik-cad-assembly"
const CAD_ASSEMBLY_VERSION = 1

// Default base plate parameters (mm)
const DEFAULT_PLATE_THICKNESS = 4.0
const DEFAULT_PLATE_MARGIN = 10.0

// Matrix4 is a row major 4x4 homogeneous transformation matrix
type Matrix4 [4][4]float64

// CadJoint describes one servo
type CadJoint struct {
	Name    string `json:"name"`
	ServoId int    `json:"servo_id"`
	// Joint angle in the rest pose (degrees)
	Angle    float64 `json:"angle"`
	Inverted bool    `json:"inverted"`
	// Joint frame: origin on the joint, Z along the rotation axis, X along the link the servo is mounted on
	Frame Matrix4 `json:"frame"`
	// Placement of the servo frame (see the notes above): the joint frame with the mounting
	// parameters (axis offset, case angle, inversion) applied
	Servo Matrix4 `json:"servo"`
}

// CadLeg describes one leg
type CadLeg struct {
	Index    int            `json:"index"`
	Segments SegmentLengths `json:"segments"`
	Joints   []CadJoint     `json:"joints"`
	// Joint positions and the foot, from the body outwards
	Points [NUM_JOINTS][3]float64 `json:"points"`
}

// CadPlate describes the base plate the coxa servos are mounted on
type CadPlate struct {
	Thickness float64 `json:"thickness"`
	// Distance from the coxa servo cases to the edge of the plate
	Margin float64 `json:"margin"`
}

// CadAssembly is the description of a pod for the CAD export
type CadAssembly struct {
	Format     string   `json:"format"`
	Version    int      `json:"version"`
	Name       string   `json:"name"`
	Units      string   `json:"units"`
	UpAxis     string   `json:"up_axis"`
	ServoModel string   `json:"servo_model"`
	Plate      CadPlate `json:"plate"`
	Legs       []CadLeg `json:"legs"`
}

// NewCadAssembly describes the pod in its rest pose
func NewCadAssembly(b *BodyDefinition, name string) (*CadAssembly, error) {
	if b.IsSegmented() {
		return nil, fmt.Errorf("the CAD export doesn't support segmented bodies yet")
	}
	// The printable brackets (cad/brackets.py) follow femur and tibia twists. A coxa twist would tilt the coxa
	// servos on the body plates
	for l := 0; l < b.NumLegs; l++ {
		if b.LegTwists(l).Coxa != 0 {
			return nil, fmt.Errorf("the CAD export doesn't support coxa twists yet (leg %d): the body plates are built for upright coxa servos", l)
		}
	}
	mapping := b.ServoMapping()
	if err := mapping.Validate(b.NumLegs); err != nil {
		return nil, err
	}

	// A new pod has its legs in the rest pose, regardless of what the simulated pod is doing
	pod := NewPod(b)

	a := &CadAssembly{
		Format:     CAD_ASSEMBLY_FORMAT,
		Version:    CAD_ASSEMBLY_VERSION,
		Name:       name,
		Units:      "mm",
		UpAxis:     "+Z",
		ServoModel: mapping.Model,
		Plate:      CadPlate{Thickness: DEFAULT_PLATE_THICKNESS, Margin: DEFAULT_PLATE_MARGIN},
	}

	for i, leg := range pod.Legs {
		cadLeg := CadLeg{Index: i, Segments: leg.SegmentLengths}
		for j, c := range leg.Joints {
			p := toCad(c)
			cadLeg.Points[j] = [3]float64{p.X, p.Y, p.Z}
		}

		frames := leg.ServoFrames()
		angles := []float64{leg.ServoAngles.Coxa, leg.ServoAngles.Femur, leg.ServoAngles.Tibia}
		for j, name := range []string{"coxa", "femur", "tibia"} {
			m, _ := mapping.Legs[i].Joint(name)
			frame := cadFrame(frames[j])
			cadLeg.Joints = append(cadLeg.Joints, CadJoint{
				Name:     name,
				ServoId:  m.Id,
				Angle:    angles[j],
				Inverted: m.Inverted,
				Frame:    toMatrix4(frame),
				Servo:    toMatrix4(servoPlacement(frame, *m)),
			})
		}
		a.Legs = append(a.Legs, cadLeg)
	}
	return a, nil
}

// Save writes the assembly as indented JSON
func (a *CadAssembly) Save(path string) error {
	data, err := json.MarshalIndent(a, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, data, 0644)
}

// cadConversion rotates 180 degrees around X: GOIK (+Z towards the ground) to CAD (Z up)
var cadConversion = mat.NewDense(4, 4, []float64{
	1, 0, 0, 0,
	0, -1, 0, 0,
	0, 0, -1, 0,
	0, 0, 0, 1,
})

func toCad(c Coordinate) Coordinate {
	return NewCoordinate(c.X, -c.Y, -c.Z)
}

func cadFrame(m *mat.Dense) *mat.Dense {
	var r mat.Dense
	r.Mul(cadConversion, m)
	return &r
}

// servoPlacement applies the joint's mounting parameters to its frame:
// move the horn face along the axis, rotate the case around the axis, and flip inverted servos
func servoPlacement(frame *mat.Dense, m JointMapping) *mat.Dense {
	a := m.CaseAngle * math.Pi / 180
	local := mat.NewDense(4, 4, []float64{
		math.Cos(a), -math.Sin(a), 0, 0,
		math.Sin(a), math.Cos(a), 0, 0,
		0, 0, 1, m.AxisOffset,
		0, 0, 0, 1,
	})
	if m.Inverted {
		// The horn points along -Z of the joint frame: rotate 180 degrees around X
		// (the offset is still measured along the horn direction)
		flip := mat.NewDense(4, 4, []float64{
			1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, -1, 0,
			0, 0, 0, 1,
		})
		var flipped mat.Dense
		flipped.Mul(flip, local)
		local = &flipped
	}

	var r mat.Dense
	r.Mul(frame, local)
	return &r
}

func toMatrix4(m *mat.Dense) Matrix4 {
	var r Matrix4
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			// Avoid -0 and rounding noise like 1e-17 in the JSON
			v := math.Round(m.At(i, j)*1e9) / 1e9
			if v == 0 {
				v = 0
			}
			r[i][j] = v
		}
	}
	return r
}

// String describes the assembly briefly
func (a *CadAssembly) String() string {
	return fmt.Sprintf("%s: %d legs, %d %s servos", a.Name, len(a.Legs), len(a.Legs)*(NUM_JOINTS-1), a.ServoModel)
}
