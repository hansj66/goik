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
	"math"
	"os"
	"path/filepath"
	"testing"
)

func column(m Matrix4, c int) [3]float64 {
	return [3]float64{m[0][c], m[1][c], m[2][c]}
}

func dot(a, b [3]float64) float64 {
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
}

func cross3(a, b [3]float64) [3]float64 {
	return [3]float64{a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]}
}

func near(a, b [3]float64, tolerance float64) bool {
	return math.Abs(a[0]-b[0]) < tolerance && math.Abs(a[1]-b[1]) < tolerance && math.Abs(a[2]-b[2]) < tolerance
}

// checkRotation fails if the matrix's rotation part is not orthonormal and right handed
func checkRotation(t *testing.T, name string, m Matrix4) {
	t.Helper()
	x, y, z := column(m, 0), column(m, 1), column(m, 2)
	for _, v := range [][3]float64{x, y, z} {
		if math.Abs(dot(v, v)-1) > 1e-6 {
			t.Errorf("%s: axis %v is not a unit vector", name, v)
		}
	}
	if math.Abs(dot(x, y)) > 1e-6 || math.Abs(dot(y, z)) > 1e-6 || math.Abs(dot(x, z)) > 1e-6 {
		t.Errorf("%s: axes are not orthogonal", name)
	}
	if !near(cross3(x, y), z, 1e-6) {
		t.Errorf("%s: frame is not right handed", name)
	}
}

func TestCadAssemblyFollowsKinematics(t *testing.T) {
	b := NewExampleHexapod1()
	a, err := NewCadAssembly(b, "hexapod1")
	if err != nil {
		t.Fatal(err)
	}
	pod := NewPod(NewExampleHexapod1())

	if len(a.Legs) != 6 || a.ServoModel != "AX-12A" {
		t.Fatalf("unexpected assembly: %s", a)
	}
	for l, leg := range a.Legs {
		for j, joint := range leg.Joints {
			checkRotation(t, joint.Name, joint.Frame)
			checkRotation(t, joint.Name+" servo", joint.Servo)

			// The joint frame's origin is the joint, converted to Z up: (x, y, z) -> (x, -y, -z)
			p := pod.Legs[l].Joints[j]
			want := [3]float64{p.X, -p.Y, -p.Z}
			if origin := column(joint.Frame, 3); !near(origin, want, 1e-6) {
				t.Errorf("leg %d %s: frame origin %v, want %v", l, joint.Name, origin, want)
			}
			if !near(leg.Points[j], want, 1e-6) {
				t.Errorf("leg %d %s: point %v, want %v", l, joint.Name, leg.Points[j], want)
			}
			// The servo ids follow the README addressing scheme
			if joint.ServoId != l*3+j+1 {
				t.Errorf("leg %d %s: servo id %d", l, joint.Name, joint.ServoId)
			}
		}

		// X points along the link the servo is mounted on: from the femur joint towards the tibia joint
		femurX := column(leg.Joints[2].Frame, 0)
		femur := [3]float64{leg.Points[2][0] - leg.Points[1][0], leg.Points[2][1] - leg.Points[1][1], leg.Points[2][2] - leg.Points[1][2]}
		length := math.Sqrt(dot(femur, femur))
		if !near(femurX, [3]float64{femur[0] / length, femur[1] / length, femur[2] / length}, 1e-6) {
			t.Errorf("leg %d: tibia servo frame X %v does not point along the femur %v", l, femurX, femur)
		}
	}

	// In the GOIK frame the feet are at +Z (below the body). In CAD they must be below the body: -Z
	if foot := a.Legs[0].Points[EFFECTOR_ORIGIN_INDEX]; foot[2] >= 0 {
		t.Errorf("foot should be below the body (z < 0) in CAD coordinates, got %v", foot)
	}
}

func TestCadServoPlacement(t *testing.T) {
	b := NewExampleHexapod1()
	plain, _ := NewCadAssembly(b, "plain")

	m := b.ServoMapping()
	m.Legs[0].Femur.Inverted = true
	m.Legs[0].Tibia.CaseAngle = 90
	m.Legs[0].Coxa.AxisOffset = 5
	changed, _ := NewCadAssembly(b, "changed")

	frame := plain.Legs[0].Joints[1].Frame
	inverted := changed.Legs[0].Joints[1].Servo
	if want := column(frame, 2); !near(column(inverted, 2), [3]float64{-want[0], -want[1], -want[2]}, 1e-6) {
		t.Errorf("inverted horn direction %v, want %v", column(inverted, 2), want)
	}
	if !near(column(inverted, 0), column(frame, 0), 1e-6) {
		t.Error("an inverted servo's case should still extend back along the link (same X)")
	}

	tibiaFrame := plain.Legs[0].Joints[2].Frame
	rotated := changed.Legs[0].Joints[2].Servo
	if !near(column(rotated, 0), column(tibiaFrame, 1), 1e-6) {
		t.Errorf("case angle 90: servo X %v should be the joint frame's Y %v", column(rotated, 0), column(tibiaFrame, 1))
	}

	coxa := plain.Legs[0].Joints[0].Frame
	shifted := changed.Legs[0].Joints[0].Servo
	z := column(coxa, 2)
	want := column(coxa, 3)
	want = [3]float64{want[0] + 5*z[0], want[1] + 5*z[1], want[2] + 5*z[2]}
	if !near(column(shifted, 3), want, 1e-6) {
		t.Errorf("axis offset 5: horn face at %v, want %v", column(shifted, 3), want)
	}
}

func TestCadAssemblySave(t *testing.T) {
	a, err := NewCadAssembly(NewSpider(), "spider")
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "spider.json")
	if err := a.Save(path); err != nil {
		t.Fatal(err)
	}

	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var loaded CadAssembly
	if err := json.Unmarshal(data, &loaded); err != nil {
		t.Fatal(err)
	}
	if loaded.Format != CAD_ASSEMBLY_FORMAT || len(loaded.Legs) != 8 || loaded.Legs[7].Joints[2].ServoId != 24 {
		t.Errorf("unexpected assembly after loading: %+v", loaded)
	}
}
