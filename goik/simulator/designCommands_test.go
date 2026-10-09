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
	"math"
	"strings"
	"testing"
)

// testShell runs commands in a shell without the chat UI, and collects what they print
type testShell struct {
	*Shell
	t *testing.T
}

func newTestShell(t *testing.T, body *robot.BodyDefinition) *testShell {
	return &testShell{Shell: NewShell(robot.NewPod(body)), t: t}
}

// run dispatches a command and returns what it printed and its error
func (s *testShell) run(command string) (string, error) {
	err := s.Dispatch(command)
	var output strings.Builder
	for {
		select {
		case line := <-s.outputCh:
			output.WriteString(line + "\n")
		default:
			return output.String(), err
		}
	}
}

// mustRun dispatches a command that should succeed, and returns what it printed
func (s *testShell) mustRun(command string) string {
	s.t.Helper()
	output, err := s.run(command)
	if err != nil {
		s.t.Fatalf("%s: %v", command, err)
	}
	return output
}

func TestDesignCommands(t *testing.T) {
	s := newTestShell(t, robot.NewExampleHexapodAX12())

	if out := s.mustRun("design import"); !strings.Contains(out, "Legs 1 (+X) and 2 (-X): (40.00, 69.28), mount angle 60.00") {
		t.Errorf("design not shown:\n%s", out)
	}

	// ALL changes each mirrored pair once: the +X legs get the angle, their mirror images the opposite
	s.mustRun("set_coxa_angle ALL 15")
	r := s.Pod.BodyDefinition.RestAngles
	if r[0].Coxa != 15 || r[1].Coxa != 15 || r[2].Coxa != -15 || r[3].Coxa != -15 {
		t.Errorf("coxa rest angles %v, want 15 on the +X side and -15 on the -X side", r)
	}

	// An eight legged pod: the servo mapping follows the number of legs
	if out := s.mustRun("design rectangle 4 210 110"); !strings.Contains(out, "servo mapping is reset") {
		t.Errorf("no note about the servo mapping:\n%s", out)
	}
	if n := s.Pod.BodyDefinition.NumLegs; n != 8 {
		t.Fatalf("%d legs, want 8", n)
	}
	if err := s.Pod.BodyDefinition.Servos.Validate(8); err != nil {
		t.Errorf("servo mapping: %v", err)
	}

	// A leg on the axis, at the front: 9 legs, so wave gait
	s.mustRun("design add 0 105 90")
	if b := s.Pod.BodyDefinition; b.NumLegs != 9 || b.Gait.Name != "Wave gait" {
		t.Fatalf("%d legs with %s, want 9 with wave gait", b.NumLegs, b.Gait.Name)
	}

	// Leg 0 is the middle front leg on the +X side (55, 35), and leg 4 its mirror image
	s.mustRun("design move 0 60 30")
	c := s.Pod.BodyDefinition.CoxaCoordinates
	if c[0].X != 60 || c[0].Y != 30 || c[4].X != -60 || c[4].Y != 30 {
		t.Errorf("legs 0 and 4 at %v and %v, want (60, 30) and (-60, 30)", c[0], c[4])
	}

	// A leg can't be moved across the axis, and a failed change leaves the pod as it was
	if _, err := s.run("design move 0 -10 30"); err == nil {
		t.Errorf("a leg was moved to the -X side")
	}
	if c := s.Pod.BodyDefinition.CoxaCoordinates[0]; c.X != 60 {
		t.Errorf("leg 0 at %v after a failed move, want (60, 30)", c)
	}

	// Leg 2 is the front leg on the axis
	s.mustRun("design remove 2")
	if n := s.Pod.BodyDefinition.NumLegs; n != 8 {
		t.Errorf("%d legs, want 8", n)
	}

	s.mustRun("design off")
	if s.Pod.BodyDefinition.Design != nil {
		t.Errorf("the design is still there")
	}
}

func TestDesignImportOfAnAsymmetricPod(t *testing.T) {
	s := newTestShell(t, robot.NewExamplePentapod())
	if _, err := s.run("design import"); err == nil || !strings.Contains(err.Error(), "mirror image") {
		t.Errorf("error = %v, want a missing mirror image", err)
	}
	if out, _ := s.run("design"); !strings.Contains(out, "no design") {
		t.Errorf("unexpected output:\n%s", out)
	}
}

func TestDesignScale(t *testing.T) {
	s := newTestShell(t, robot.NewExampleHexapodSTS3215())
	s.mustRun("design import")
	s.mustRun("set_femur_twist 1 -20")
	s.mustRun("stance 100")
	s.mustRun("design scale 1.25")
	b := s.Pod.BodyDefinition
	if c := b.CoxaCoordinates[0]; c.X != 82.5 || c.Y != 0 {
		t.Errorf("leg 0 at %v, want (82.5, 0)", c)
	}
	if c := b.CoxaCoordinates[1]; c.X != 41.25 || c.Y != 71.45 {
		t.Errorf("leg 1 at %v, want (41.25, 71.45)", c)
	}
	// The legs, their twists and the stance stay
	if b.Segments[1].Coxa != 52 || b.LegTwists(1).Femur != -20 || b.LegTwists(2).Femur != 20 || b.Stance == nil || b.Stance.Height != 100 {
		t.Errorf("segments %+v, twists %v, stance %+v", b.Segments[1], b.Twists, b.Stance)
	}
	if _, err := s.run("design scale 0"); err == nil {
		t.Errorf("a scale of 0 was accepted")
	}

	// The radius: pod 7's legs are mounted 66 mm from the centre, 82.5 after the scale
	if out := s.mustRun("design"); !strings.Contains(out, "Body radius: 82.50 mm") {
		t.Errorf("radius not shown:\n%s", out)
	}
	s.mustRun("design radius 90")
	if c := s.Pod.BodyDefinition.CoxaCoordinates[0]; c.X != 90 || c.Y != 0 {
		t.Errorf("leg 0 at %v, want (90, 0)", c)
	}
	if c := s.Pod.BodyDefinition.CoxaCoordinates[1]; math.Abs(math.Hypot(c.X, c.Y)-90) > 0.01 {
		t.Errorf("leg 1 at %v, want 90 mm from the centre", c)
	}
}
