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

package control

import (
	"math"
	"strings"
	"testing"
)

func connected() GamepadState {
	return GamepadState{Connected: true, Name: "Test pad"}
}

func withAxis(s GamepadState, a Axis, v float64) GamepadState {
	s.Axes[a] = v
	return s
}

func withButton(s GamepadState, b Button) GamepadState {
	s.Buttons[b] = true
	return s
}

func near(a, b float64) bool {
	return math.Abs(a-b) < 1e-9
}

func TestSticksDriveThePod(t *testing.T) {
	tests := []struct {
		name  string
		state GamepadState
		check func(Command) bool
	}{
		{"left stick up walks forward at top speed", withAxis(connected(), LeftStickY, -1), func(c Command) bool { return near(c.Twist.Y, 75) && near(c.Twist.X, 0) }},
		{"left stick right walks to the robot's right (-X)", withAxis(connected(), LeftStickX, 1), func(c Command) bool { return near(c.Twist.X, -75) }},
		{"right stick right turns right (positive yaw)", withAxis(connected(), RightStickX, 1), func(c Command) bool { return near(c.Twist.Yaw, 45) }},
		{"right stick up tilts the nose down", withAxis(connected(), RightStickY, -1), func(c Command) bool { return near(c.Pitch, -15) }},
		{"left trigger rolls left (positive roll lowers +X)", withAxis(connected(), LeftTrigger, 1), func(c Command) bool { return near(c.Roll, 15) }},
		{"right trigger rolls right", withAxis(connected(), RightTrigger, 1), func(c Command) bool { return near(c.Roll, -15) }},
		{"diagonally at most the top speed", withAxis(withAxis(connected(), LeftStickX, 1), LeftStickY, -1), func(c Command) bool {
			return near(math.Hypot(c.Twist.X, c.Twist.Y), 75) && c.Twist.X < 0 && c.Twist.Y > 0
		}},
		{"half way is slower than half speed (response curve)", withAxis(connected(), LeftStickY, -0.5), func(c Command) bool { return c.Twist.Y > 0 && c.Twist.Y < 75.0/2 }},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			c := NewController()
			c.Update(connected())
			cmd := c.Update(tt.state)
			if !cmd.Drive || !cmd.TakeOver {
				t.Fatalf("drive %v, take over %v: want both", cmd.Drive, cmd.TakeOver)
			}
			if !tt.check(cmd) {
				t.Errorf("unexpected command %+v", cmd)
			}
		})
	}
}

func TestDeadZoneAndRelease(t *testing.T) {
	c := NewController()
	c.Update(connected())

	// A resting stick doesn't creep
	if cmd := c.Update(withAxis(connected(), LeftStickY, -0.1)); cmd.Drive || cmd.TakeOver {
		t.Errorf("a stick inside the dead zone drives: %+v", cmd)
	}

	// Take over once, keep driving, then stop and level once when released
	walk := withAxis(withAxis(connected(), LeftStickY, -1), RightStickY, 1)
	if cmd := c.Update(walk); !cmd.TakeOver {
		t.Errorf("no take over")
	}
	if cmd := c.Update(walk); cmd.TakeOver || !cmd.Drive || !near(cmd.Pitch, 15) {
		t.Errorf("second update: %+v", cmd)
	}
	cmd := c.Update(connected())
	if !cmd.Drive || !cmd.Twist.IsZero() || cmd.Pitch != 0 || cmd.Roll != 0 {
		t.Errorf("release: %+v, want a stop and level pitch and roll", cmd)
	}
	if cmd := c.Update(connected()); cmd.Drive || c.Engaged() {
		t.Errorf("still driving after the release: %+v", cmd)
	}
}

func TestHaltUntilTheSticksAreReleased(t *testing.T) {
	c := NewController()
	c.Update(connected())
	walk := withAxis(connected(), LeftStickY, -1)
	c.Update(walk)

	if cmd := c.Update(withButton(walk, ButtonB)); !cmd.Twist.IsZero() {
		t.Errorf("B didn't halt: %+v", cmd)
	}
	if cmd := c.Update(walk); !cmd.Twist.IsZero() {
		t.Errorf("walking again before the stick was released: %+v", cmd)
	}
	c.Update(connected())
	if cmd := c.Update(walk); !near(cmd.Twist.Y, 75) {
		t.Errorf("not walking after the stick was released and pushed again: %+v", cmd)
	}
}

func TestButtons(t *testing.T) {
	c := NewController()
	c.Update(connected())
	press := func(b Button) Command {
		cmd := c.Update(withButton(connected(), b))
		c.Update(connected())
		return cmd
	}

	// Top speed steps, within the list
	press(DPadUp)
	if cmd := press(DPadUp); c.TopSpeed() != 150 || !strings.Contains(strings.Join(cmd.Messages, " "), "150 mm/s") {
		t.Errorf("top speed %.0f (%v), want 150", c.TopSpeed(), cmd.Messages)
	}
	press(DPadUp)
	if c.TopSpeed() != 150 {
		t.Errorf("top speed %.0f past the last step", c.TopSpeed())
	}
	for i := 0; i < 10; i++ {
		press(DPadDown)
	}
	if c.TopSpeed() != 25 {
		t.Errorf("top speed %.0f, want 25", c.TopSpeed())
	}

	// Body height: left is lower (towards the ground, +Z), right is higher
	if cmd := press(DPadLeft); cmd.HeightChange != 10 {
		t.Errorf("D-pad left: height change %.0f, want 10", cmd.HeightChange)
	}
	if cmd := press(DPadRight); cmd.HeightChange != -10 {
		t.Errorf("D-pad right: height change %.0f, want -10", cmd.HeightChange)
	}

	for b, gait := range map[Button]string{ButtonA: "tripod", ButtonX: "ripple", ButtonY: "wave"} {
		if cmd := press(b); cmd.Gait != gait {
			t.Errorf("button %d: gait '%s', want %s", b, cmd.Gait, gait)
		}
	}
	if cmd := press(ButtonMenu); !cmd.Level {
		t.Errorf("Menu didn't level")
	}

	// A held button only acts once
	c.Update(withButton(connected(), ButtonA))
	if cmd := c.Update(withButton(connected(), ButtonA)); cmd.Gait != "" {
		t.Errorf("a held button acted again")
	}
}

func TestViewButtonTurnsControlOff(t *testing.T) {
	c := NewController()
	c.Update(connected())
	walk := withAxis(connected(), LeftStickY, -1)
	c.Update(walk)

	cmd := c.Update(withButton(walk, ButtonView))
	if c.Enabled || !cmd.Drive || !cmd.Twist.IsZero() {
		t.Errorf("View: enabled %v, command %+v, want off with a stop", c.Enabled, cmd)
	}
	c.Update(walk)
	if cmd := c.Update(walk); cmd.Drive {
		t.Errorf("driving while off: %+v", cmd)
	}
	c.Update(withButton(walk, ButtonView))
	if !c.Enabled {
		t.Errorf("View didn't turn it on again")
	}
}

func TestDisconnectHalts(t *testing.T) {
	c := NewController()
	if cmd := c.Update(connected()); !strings.Contains(strings.Join(cmd.Messages, " "), "connected: Test pad") {
		t.Errorf("messages %v", cmd.Messages)
	}
	c.Update(withAxis(connected(), LeftStickY, -1))
	cmd := c.Update(GamepadState{})
	if !cmd.Drive || !cmd.Twist.IsZero() || c.Engaged() {
		t.Errorf("disconnect: %+v, want a stop", cmd)
	}
	if c.Status() != "Gamepad: not connected" {
		t.Errorf("status '%s'", c.Status())
	}
}
