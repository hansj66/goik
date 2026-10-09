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

// Package control turns gamepad input into gait engine commands. It doesn't depend on Ebitengine or any other
// input library, so the simulator and the robot controller (Overlord) can share it: they read the gamepad their
// own way, and pass its state to a Controller.
package control

import (
	"GOIK/robot"
	"fmt"
	"math"
)

/*
	Mapping (Xbox controller names):

		Left stick         walk: push direction = walking direction, deflection = speed
		Right stick left/right   turn (yaw rate). Right turns right
		Right stick up/down      pitch: up tilts the nose down, down tilts it up
		LT / RT            roll left / right (lowers that side)
		D-pad up/down      top speed, in steps
		D-pad left/right   body lower / higher
		A / X / Y          tripod / ripple / wave gait
		B                  halt: stop walking until the sticks are released
		Menu               level: back to the neutral body pose
		View               gamepad control on/off

	Directions are the robot's own: +Y is forward, and +X is the robot's left side (Z points down), so "right" is -X.
	Pitch and roll follow the sticks and triggers, and spring back to level when they are released.

	The controller takes over when a stick, trigger or button is used, and hands back once everything is released:
	it then stops the pod and levels the pitch and roll once, so shell commands and scripts work as before.
*/

// Axis is a gamepad axis. Sticks go from -1 to 1 (up and left are -1), triggers from 0 (released) to 1
type Axis int

const (
	LeftStickX Axis = iota
	LeftStickY
	RightStickX
	RightStickY
	LeftTrigger
	RightTrigger
	NumAxes
)

// Button is a gamepad button
type Button int

const (
	ButtonA Button = iota
	ButtonB
	ButtonX
	ButtonY
	ButtonMenu
	ButtonView
	DPadUp
	DPadDown
	DPadLeft
	DPadRight
	NumButtons
)

// GamepadState is a snapshot of a gamepad
type GamepadState struct {
	Connected bool
	Name      string
	Axes      [NumAxes]float64
	Buttons   [NumButtons]bool
}

var axisNames = [NumAxes]string{"left stick x", "left stick y", "right stick x", "right stick y", "LT", "RT"}
var buttonNames = [NumButtons]string{"A", "B", "X", "Y", "Menu", "View", "D-pad up", "D-pad down", "D-pad left", "D-pad right"}

// String shows the raw axis values and the buttons that are down
func (s GamepadState) String() string {
	text := ""
	for a, v := range s.Axes {
		text += fmt.Sprintf("%s %.2f, ", axisNames[a], v)
	}
	down := ""
	for b, pressed := range s.Buttons {
		if pressed {
			down += " " + buttonNames[b]
		}
	}
	if down == "" {
		down = " none"
	}
	return text + "buttons down:" + down
}

// State returns the gamepad state from the last update
func (c *Controller) State() GamepadState {
	return c.previous
}

// Command is what the controller wants done after a gamepad update
type Command struct {
	// The controller takes over from scripts (stop the running script)
	TakeOver bool
	// Set the twist, and the body's pitch and roll (degrees)
	Drive bool
	Twist robot.Twist
	Pitch float64
	Roll  float64
	// Move the body down (positive, towards the ground) or up (negative), in mm
	HeightChange float64
	// Return the body to the neutral pose (before HeightChange and Drive are applied)
	Level bool
	// Change to this gait (tripod, ripple or wave). Empty: no change
	Gait string
	// Messages for the user
	Messages []string
}

// Controller maps gamepad input to commands
type Controller struct {
	// Off: the gamepad is ignored, except for the button that turns it on again
	Enabled bool
	// Dead zones: smaller stick deflections and trigger values are 0. Sticks often rest slightly off centre (one
	// test pad's right stick rests at 0.17), so the sticks' dead zone is close to Microsoft's XInput recommendation
	// (0.24 to 0.27). The left stick's dead zone is radial (on the stick's deflection, so walking directions stay
	// smooth); the right stick's is per axis, since its axes do unrelated things (turn and pitch)
	StickDeadZone   float64
	TriggerDeadZone float64
	// Response curve: 0 is linear, 1 is cubic (fine control near the centre)
	Expo float64
	// Top speeds (mm/s) selected with the D-pad, and the selected one
	SpeedSteps []float64
	speed      int
	// Turn rate (degrees/s) at full deflection
	MaxYawRate float64
	// Pitch and roll (degrees) at full deflection
	MaxTilt float64
	// Body height change (mm) per D-pad press
	HeightStep float64

	previous GamepadState
	engaged  bool
	// B was pressed: walking stops until the sticks are released
	halted bool
}

// NewController returns a controller with the default settings
func NewController() *Controller {
	return &Controller{
		Enabled:    true,
		StickDeadZone:   0.25,
		TriggerDeadZone: 0.15,
		Expo:       0.6,
		SpeedSteps: []float64{25, 50, 75, 100, 150},
		speed:      2,
		MaxYawRate: 45,
		MaxTilt:    15,
		HeightStep: 10,
	}
}

// TopSpeed returns the speed (mm/s) at full deflection of the left stick
func (c *Controller) TopSpeed() float64 {
	return c.SpeedSteps[c.speed]
}

// Engaged returns true while the gamepad is driving the pod
func (c *Controller) Engaged() bool {
	return c.engaged
}

// Status describes the gamepad and what it is doing
func (c *Controller) Status() string {
	s := c.previous
	switch {
	case !s.Connected:
		return "Gamepad: not connected"
	case !c.Enabled:
		return fmt.Sprintf("Gamepad: %s, off (the View button turns it on)", s.Name)
	}
	state := "idle"
	if c.halted {
		state = "halted (release the sticks)"
	} else if c.engaged {
		state = "driving"
	}
	return fmt.Sprintf("Gamepad: %s, top speed %.0f mm/s, %s", s.Name, c.TopSpeed(), state)
}

// shape applies a dead zone and the response curve to an axis value
func (c *Controller) shape(v float64, deadZone float64) float64 {
	a := math.Abs(v)
	if a <= deadZone {
		return 0
	}
	n := math.Min((a-deadZone)/(1-deadZone), 1)
	n = (1-c.Expo)*n + c.Expo*n*n*n
	return math.Copysign(n, v)
}

// shapeStick applies the dead zone and the response curve to a stick's deflection, keeping its direction
func (c *Controller) shapeStick(x float64, y float64) (float64, float64) {
	m := math.Hypot(x, y)
	if m <= c.StickDeadZone {
		return 0, 0
	}
	n := c.shape(m, c.StickDeadZone)
	return x / m * n, y / m * n
}

// release stops the pod and levels its pitch and roll, and hands control back
func (c *Controller) release(cmd *Command) {
	cmd.Drive = true
	cmd.Twist = robot.Twist{}
	cmd.Pitch, cmd.Roll = 0, 0
	c.engaged = false
	c.halted = false
}

// Update takes the gamepad's current state, and returns what to do
func (c *Controller) Update(s GamepadState) Command {
	var cmd Command
	previous := c.previous
	c.previous = s

	if s.Connected != previous.Connected {
		if s.Connected {
			cmd.Messages = append(cmd.Messages, fmt.Sprintf("Gamepad connected: %s", s.Name))
		} else {
			cmd.Messages = append(cmd.Messages, "Gamepad disconnected")
		}
	}
	if !s.Connected {
		// Watchdog: never keep walking without a gamepad
		if c.engaged {
			c.release(&cmd)
			cmd.Messages = append(cmd.Messages, "Halting")
		}
		return cmd
	}

	pressed := func(b Button) bool { return s.Buttons[b] && !previous.Buttons[b] }

	if pressed(ButtonView) {
		c.Enabled = !c.Enabled
		if c.Enabled {
			cmd.Messages = append(cmd.Messages, "Gamepad control on")
		} else {
			cmd.Messages = append(cmd.Messages, "Gamepad control off")
			if c.engaged {
				c.release(&cmd)
			}
		}
		return cmd
	}
	if !c.Enabled {
		return cmd
	}

	// Buttons
	if pressed(DPadUp) && c.speed < len(c.SpeedSteps)-1 {
		c.speed++
		cmd.Messages = append(cmd.Messages, fmt.Sprintf("Top speed: %.0f mm/s", c.TopSpeed()))
	}
	if pressed(DPadDown) && c.speed > 0 {
		c.speed--
		cmd.Messages = append(cmd.Messages, fmt.Sprintf("Top speed: %.0f mm/s", c.TopSpeed()))
	}
	if pressed(DPadLeft) {
		cmd.HeightChange = c.HeightStep
	}
	if pressed(DPadRight) {
		cmd.HeightChange = -c.HeightStep
	}
	for button, gait := range map[Button]string{ButtonA: "tripod", ButtonX: "ripple", ButtonY: "wave"} {
		if pressed(button) {
			cmd.Gait = gait
		}
	}
	if pressed(ButtonB) {
		c.halted = true
		cmd.Messages = append(cmd.Messages, "Halt (release the sticks to walk again)")
	}
	if pressed(ButtonMenu) {
		cmd.Level = true
	}

	// Sticks and triggers
	lx, ly := c.shapeStick(s.Axes[LeftStickX], s.Axes[LeftStickY])
	rx, ry := c.shape(s.Axes[RightStickX], c.StickDeadZone), c.shape(s.Axes[RightStickY], c.StickDeadZone)
	lt, rt := c.shape(s.Axes[LeftTrigger], c.TriggerDeadZone), c.shape(s.Axes[RightTrigger], c.TriggerDeadZone)
	centred := lx == 0 && ly == 0 && rx == 0 && ry == 0 && lt == 0 && rt == 0
	if c.halted && centred {
		c.halted = false
	}

	active := !centred
	for _, down := range s.Buttons {
		active = active || down
	}
	if active && !c.engaged {
		c.engaged = true
		cmd.TakeOver = true
	}
	if !c.engaged {
		return cmd
	}
	if !active {
		c.release(&cmd)
		return cmd
	}

	cmd.Drive = true
	if !c.halted {
		// The stick's direction, at most the top speed (also diagonally)
		x, y := -lx, -ly
		if l := math.Hypot(x, y); l > 1 {
			x, y = x/l, y/l
		}
		cmd.Twist = robot.Twist{X: x * c.TopSpeed(), Y: y * c.TopSpeed(), Yaw: rx * c.MaxYawRate}
	}
	cmd.Pitch = ry * c.MaxTilt
	cmd.Roll = (lt - rt) * c.MaxTilt
	return cmd
}
