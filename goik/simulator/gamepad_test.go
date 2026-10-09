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
	"GOIK/control"
	"GOIK/robot"
	"testing"
)

// pad returns a connected gamepad state
func pad() control.GamepadState {
	return control.GamepadState{Connected: true, Name: "Test pad"}
}

// tick applies the gamepad state and advances the pod for a number of engine ticks
func (s *testShell) tick(state control.GamepadState, ticks int) {
	for i := 0; i < ticks; i++ {
		s.applyGamepad(state)
		s.Pod.Update()
	}
}

func TestGamepadWalksAndTurnsRight(t *testing.T) {
	s := newTestShell(t, robot.NewExampleHexapodAX12())
	s.tick(pad(), 1)

	// Forward and turning right (right stick right): the pod curves towards its right side, -X
	state := pad()
	state.Axes[control.LeftStickY] = -1
	state.Axes[control.RightStickX] = 0.6
	s.tick(state, 300)

	e := s.Pod.Engine
	if e == nil {
		t.Fatal("no gait engine")
	}
	x, y, heading := e.Odometry()
	t.Logf("x %.1f, y %.1f, heading %.1f", x, y, heading)
	if y <= 50 || x >= -5 {
		t.Errorf("walked to (%.1f, %.1f), want forward (+Y) and to the right (-X)", x, y)
	}

	// Released: the engine stops
	s.tick(pad(), 400)
	if !e.IsIdle() {
		t.Errorf("still walking after the sticks were released (twist %s)", e.GetTwist().String())
	}
}

func TestGamepadPoseAndButtons(t *testing.T) {
	s := newTestShell(t, robot.NewExampleHexapodAX12())
	s.tick(pad(), 1)

	// Pitch and roll follow the right stick and the triggers
	state := pad()
	state.Axes[control.RightStickY] = 1
	state.Axes[control.RightTrigger] = 1
	s.tick(state, 1)
	if p := s.Pod.Engine.TargetBodyPose(); p.Pitch != 15 || p.Roll != -15 {
		t.Errorf("pose %s, want pitch 15 and roll -15", p.String())
	}
	// ... and spring back
	s.tick(pad(), 1)
	if p := s.Pod.Engine.TargetBodyPose(); p.Pitch != 0 || p.Roll != 0 {
		t.Errorf("pose %s after release, want level", p.String())
	}

	// D-pad right raises the body, Menu levels it
	press := func(b control.Button) {
		state := pad()
		state.Buttons[b] = true
		s.tick(state, 1)
		s.tick(pad(), 1)
	}
	press(control.DPadRight)
	if z := s.Pod.Engine.TargetBodyPose().Z; z != -10 {
		t.Errorf("body Z %.0f, want -10 (10 mm up)", z)
	}
	press(control.ButtonMenu)
	if p := s.Pod.Engine.TargetBodyPose(); !p.IsZero() {
		t.Errorf("pose %s after Menu, want neutral", p.String())
	}

	// Y selects wave gait
	press(control.ButtonY)
	if name := s.Pod.BodyDefinition.Gait.Name; name != "Wave gait" {
		t.Errorf("gait %s, want wave", name)
	}
}

func TestGamepadStopsAScript(t *testing.T) {
	s := newTestShell(t, robot.NewExampleHexapodAX12())
	s.mustRun("walk 0 50 0 for 10")
	if s.Script == nil || s.Script.Done() {
		t.Fatal("no script running")
	}
	state := pad()
	state.Axes[control.LeftStickX] = 1
	s.tick(pad(), 1)
	s.tick(state, 1)
	if !s.Script.Done() {
		t.Errorf("the script is still running")
	}
	if tw := s.Pod.Engine.GetTwist(); tw.X >= 0 {
		t.Errorf("twist %s, want the gamepad's (walking to -X)", tw.String())
	}
}
