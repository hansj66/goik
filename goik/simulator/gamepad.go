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
	"fmt"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
)

// Gamepad control: the gamepad is read with Ebitengine's standard gamepad layout, and control.Controller turns it
// into gait engine commands. The gamepad is read every frame, also while the terminal has the focus.

var gamepadButtons = map[control.Button]ebiten.StandardGamepadButton{
	control.ButtonA:    ebiten.StandardGamepadButtonRightBottom,
	control.ButtonB:    ebiten.StandardGamepadButtonRightRight,
	control.ButtonX:    ebiten.StandardGamepadButtonRightLeft,
	control.ButtonY:    ebiten.StandardGamepadButtonRightTop,
	control.ButtonMenu: ebiten.StandardGamepadButtonCenterRight,
	control.ButtonView: ebiten.StandardGamepadButtonCenterLeft,
	control.DPadUp:     ebiten.StandardGamepadButtonLeftTop,
	control.DPadDown:   ebiten.StandardGamepadButtonLeftBottom,
	control.DPadLeft:   ebiten.StandardGamepadButtonLeftLeft,
	control.DPadRight:  ebiten.StandardGamepadButtonLeftRight,
}

// readGamepad returns the state of the first connected gamepad with a standard layout (Xbox and most others)
func readGamepad() control.GamepadState {
	for _, id := range ebiten.AppendGamepadIDs(nil) {
		if !ebiten.IsStandardGamepadLayoutAvailable(id) {
			continue
		}
		s := control.GamepadState{Connected: true, Name: ebiten.GamepadName(id)}
		s.Axes[control.LeftStickX] = ebiten.StandardGamepadAxisValue(id, ebiten.StandardGamepadAxisLeftStickHorizontal)
		s.Axes[control.LeftStickY] = ebiten.StandardGamepadAxisValue(id, ebiten.StandardGamepadAxisLeftStickVertical)
		s.Axes[control.RightStickX] = ebiten.StandardGamepadAxisValue(id, ebiten.StandardGamepadAxisRightStickHorizontal)
		s.Axes[control.RightStickY] = ebiten.StandardGamepadAxisValue(id, ebiten.StandardGamepadAxisRightStickVertical)
		s.Axes[control.LeftTrigger] = ebiten.StandardGamepadButtonValue(id, ebiten.StandardGamepadButtonFrontBottomLeft)
		s.Axes[control.RightTrigger] = ebiten.StandardGamepadButtonValue(id, ebiten.StandardGamepadButtonFrontBottomRight)
		for b, eb := range gamepadButtons {
			s.Buttons[b] = ebiten.IsStandardGamepadButtonPressed(id, eb)
		}
		return s
	}
	return control.GamepadState{}
}

// say prints a message in the shell without blocking (the game loop calls this)
func (s *Shell) say(message string) {
	select {
	case s.outputCh <- message:
	default:
	}
}

// applyGamepad passes the gamepad's state to the controller, and carries out the command it returns.
// Must be called with the shell locked.
func (s *Shell) applyGamepad(state control.GamepadState) {
	cmd := s.Gamepad.Update(state)
	for _, m := range cmd.Messages {
		s.say(m)
	}
	if cmd.TakeOver {
		s.stopScript()
	}
	if !cmd.Drive && !cmd.Level && cmd.Gait == "" && cmd.HeightChange == 0 {
		return
	}

	engine, err := s.gaitEngine()
	if err != nil {
		s.gamepadProblem(err)
		return
	}

	if cmd.Gait != "" {
		if err := engine.SetGaitByName(cmd.Gait); err != nil {
			s.say(fmt.Sprintf("Gamepad: %v", err))
		} else {
			s.say(fmt.Sprintf("Gait: %s", cmd.Gait))
		}
	}

	pose := engine.TargetBodyPose()
	if cmd.Level {
		pose = robot.BodyPose{}
		s.say("Level")
	}
	if cmd.HeightChange != 0 {
		pose.Z = math.Max(-robot.MAX_POSE_TRANSLATION, math.Min(robot.MAX_POSE_TRANSLATION, pose.Z+cmd.HeightChange))
		s.say(fmt.Sprintf("Body height: %+.0f mm", -pose.Z))
	}
	if cmd.Drive {
		engine.SetTwist(cmd.Twist)
		pose.Pitch, pose.Roll = cmd.Pitch, cmd.Roll
	}
	if err := engine.SetBodyPose(pose); err != nil {
		s.gamepadProblem(err)
		return
	}
	s.gamepadError = ""
}

// gamepadProblem reports an error once, not every frame
func (s *Shell) gamepadProblem(err error) {
	if message := fmt.Sprintf("Gamepad: %v", err); message != s.gamepadError {
		s.gamepadError = message
		s.say(message)
	}
}

func (s *Shell) executeGamepadCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)
	if len(args) > 2 {
		return fmt.Errorf("syntax error ('gamepad [on | off]'): %+v", args)
	}
	if len(args) == 2 {
		switch args[1] {
		case "on":
			s.Gamepad.Enabled = true
		case "off":
			s.Gamepad.Enabled = false
		default:
			return fmt.Errorf("syntax error ('gamepad [on | off]'): %+v", args)
		}
	}

	s.outputCh <- s.Gamepad.Status()
	if state := s.Gamepad.State(); state.Connected {
		s.outputCh <- "Raw input: " + state.String()
	}
	for _, line := range gamepadMapping {
		s.outputCh <- "\t" + line
	}
	return nil
}

var gamepadMapping = []string{
	"Left stick             - Walk: direction and speed",
	"Right stick left/right - Turn",
	"Right stick up/down    - Pitch (up tilts the nose down)",
	"LT / RT                - Roll left / right",
	"D-pad up/down          - Top speed",
	"D-pad left/right       - Body lower / higher",
	"A / X / Y              - Tripod / ripple / wave gait",
	"B                      - Halt (until the sticks are released)",
	"Menu                   - Level the body",
	"View                   - Gamepad control on/off",
}
