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

package simulator

import (
	"GOIK/robot"
	"strings"
	"testing"
)

func TestStanceCommands(t *testing.T) {
	s := newTestShell(t, robot.NewExampleHexapodAX12())

	out := s.mustRun("stance 110")
	for _, want := range []string{"Stance: body 110 mm above the ground, tibia vertical", "The feet can step", "Possible heights with these reach settings: 90 to 185 mm"} {
		if !strings.Contains(out, want) {
			t.Errorf("output lacks '%s':\n%s", want, out)
		}
	}

	// The angles follow the tibia length, and can't be set by hand
	s.mustRun("set_tibia_length ALL 140")
	if _, err := s.run("set_femur_angle 0 10"); err == nil {
		t.Errorf("a femur angle was accepted with a stance")
	}
	for l, leg := range s.Pod.Legs {
		if z := leg.Joints[robot.EFFECTOR_ORIGIN_INDEX].Z; z < 109.999 || z > 110.001 {
			t.Errorf("leg %d: foot at Z = %.3f, want 110", l, z)
		}
	}

	// With a design, a leg's reach is set for its mirror image too
	s.mustRun("design import")
	if out := s.mustRun("stance reach 1 110"); !strings.Contains(out, "Leg 2: femur") || !strings.Contains(out, "foot 110 mm out from the femur joint (set for this leg)") {
		t.Errorf("unexpected output:\n%s", out)
	}
	if r := s.Pod.BodyDefinition.Stance.LegReach; r[1] != 110 || r[2] != 110 || r[0] != 0 {
		t.Errorf("leg reach %v, want 110 for legs 1 and 2", r)
	}
	s.mustRun("stance reach 2 default")
	if r := s.Pod.BodyDefinition.Stance.LegReach; r != nil {
		t.Errorf("leg reach %v, want none", r)
	}

	// An impossible height is rejected with the possible range, and the stance stays
	if _, err := s.run("stance 300"); err == nil || !strings.Contains(err.Error(), "Possible heights") {
		t.Errorf("error = %v, want the possible heights", err)
	}
	if h := s.Pod.BodyDefinition.Stance.Height; h != 110 {
		t.Errorf("height %.0f after a failed change, want 110", h)
	}

	s.mustRun("stance off")
	s.mustRun("set_femur_angle 0 10")
}
