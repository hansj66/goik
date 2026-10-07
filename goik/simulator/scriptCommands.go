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
	"GOIK/script"
	"fmt"
	"path/filepath"
	"strconv"
	"strings"
)

// Commands for motion scripts and the servo mapping

const SCRIPTS_FOLDER = "scripts"
const SCRIPT_EXTENSION = ".goik"

// tickScript advances the running script. Called from the game loop (with the shell lock held)
// before the pod is updated
func (s *Shell) tickScript() {
	if s.Script == nil || s.Script.Done() {
		return
	}
	// The engine is dropped when the pod changes (reset, load, geometry changes, zero, ground)
	if s.Pod.Engine != s.scriptEngine {
		s.Script.Stop()
		s.outputCh <- fmt.Sprintf("Script %s stopped: the pod was changed", s.Script.Name)
		return
	}
	if err := s.Script.Tick(robot.ENGINE_DT); err != nil {
		s.outputCh <- fmt.Sprintf("Script stopped: %v", err)
		return
	}
	if s.Script.Done() {
		s.outputCh <- fmt.Sprintf("Script %s finished", s.Script.Name)
	}
}

// stopScript aborts the running script (if any) and halts the pod
func (s *Shell) stopScript() {
	if s.Script != nil && !s.Script.Done() {
		s.Script.Stop()
		s.outputCh <- fmt.Sprintf("Script %s aborted", s.Script.Name)
	}
}

func (s *Shell) executeRunCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if len(args) != 2 {
		return fmt.Errorf("syntax error ('run <script>'): %+v", args)
	}

	name := strings.TrimSuffix(args[1], SCRIPT_EXTENSION)
	commands, err := script.Load(filepath.Join(SCRIPTS_FOLDER, name+SCRIPT_EXTENSION))
	if err != nil {
		return err
	}

	engine, err := s.gaitEngine()
	if err != nil {
		return err
	}

	s.stopScript()
	s.startScript(name, commands, engine)
	s.outputCh <- fmt.Sprintf("Running %s (%d commands)", name, len(commands))
	return nil
}

// startScript starts running commands on the engine
func (s *Shell) startScript(name string, commands []script.Command, engine *robot.GaitEngine) {
	s.Script = script.NewRunner(name, commands, engine, func(msg string) { s.outputCh <- msg })
	s.scriptEngine = engine
}

func (s *Shell) executeAbortCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if s.Script == nil || s.Script.Done() {
		return fmt.Errorf("no script is running")
	}
	s.stopScript()
	return nil
}

func (s *Shell) executeScriptsCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	files, err := filepath.Glob(filepath.Join(SCRIPTS_FOLDER, "*"+SCRIPT_EXTENSION))
	if err != nil {
		return err
	}
	if len(files) == 0 {
		return fmt.Errorf("no scripts found in ./%s", SCRIPTS_FOLDER)
	}
	for _, f := range files {
		s.outputCh <- "\t" + strings.TrimSuffix(filepath.Base(f), SCRIPT_EXTENSION)
	}
	return nil
}

func (s *Shell) executeServosCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	m := s.Pod.BodyDefinition.ServoMapping()
	model := robot.ServoModels[m.Model]
	s.outputCh <- fmt.Sprintf("Servo model: %s (protocol %d, %2.0f degrees, %d positions), baud rate: %d", model.Name, model.Protocol, model.RangeDegrees, model.Resolution, m.BaudRate)
	for l := range m.Legs {
		line := fmt.Sprintf("Leg %d:", l)
		for _, name := range []string{"coxa", "femur", "tibia"} {
			j, _ := m.Legs[l].Joint(name)
			line += fmt.Sprintf("  %s id %d", name, j.Id)
			if j.Inverted {
				line += " inv"
			}
			if j.Offset != 0 {
				line += fmt.Sprintf(" off %2.1f", j.Offset)
			}
			if j.Min != 0 || j.Max != 0 {
				line += fmt.Sprintf(" [%2.0f, %2.0f]", j.Min, j.Max)
			}
			if j.CaseAngle != 0 {
				line += fmt.Sprintf(" case %2.0f", j.CaseAngle)
			}
			if j.AxisOffset != 0 {
				line += fmt.Sprintf(" axis %2.1f", j.AxisOffset)
			}
		}
		s.outputCh <- line
	}
	return nil
}

func (s *Shell) executeServoModelCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if len(args) != 2 {
		return fmt.Errorf("syntax error ('servo_model <%s>'): %+v", strings.Join(robot.ServoModelNames(), " | "), args)
	}
	model, ok := robot.FindServoModel(args[1])
	if !ok {
		return fmt.Errorf("unknown servo model '%s'. Supported models: %s", args[1], strings.Join(robot.ServoModelNames(), ", "))
	}
	s.Pod.BodyDefinition.ServoMapping().Model = model
	s.outputCh <- fmt.Sprintf("Servo model: %s (reset and load replace it with the pod's own mapping)", model)
	return nil
}

func (s *Shell) executeServoCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)
	usage := fmt.Errorf("syntax error ('servo <ALL | legNum> <coxa|femur|tibia> id <n> | invert <on|off> | offset <deg> | limits <min> <max> | case <deg> | axis_offset <mm>'): %+v", args)

	if len(args) < 5 {
		return usage
	}

	m := s.Pod.BodyDefinition.ServoMapping()
	var legs []int
	if strings.ToUpper(args[1]) == "ALL" {
		if args[3] == "id" {
			return fmt.Errorf("servo ids have to be unique. Set them one leg at a time")
		}
		for l := range m.Legs {
			legs = append(legs, l)
		}
	} else {
		l, err := strconv.Atoi(args[1])
		if err != nil || l < 0 || l >= len(m.Legs) {
			return fmt.Errorf("invalid leg index. (Pod has %d legs. Indexing is 0 based)", len(m.Legs))
		}
		legs = []int{l}
	}

	// Change a copy, so an invalid change (duplicate id etc) leaves the mapping untouched
	changed := *m
	changed.Legs = append([]robot.LegMapping(nil), m.Legs...)

	for _, l := range legs {
		j, err := changed.Legs[l].Joint(args[2])
		if err != nil {
			return err
		}
		switch args[3] {
		case "id":
			id, err := strconv.Atoi(args[4])
			if err != nil || len(args) != 5 {
				return usage
			}
			j.Id = id
		case "invert":
			if len(args) != 5 || (args[4] != "on" && args[4] != "off") {
				return usage
			}
			j.Inverted = args[4] == "on"
		case "offset":
			offset, err := strconv.ParseFloat(args[4], 64)
			if err != nil || len(args) != 5 {
				return usage
			}
			j.Offset = offset
		case "limits":
			if len(args) != 6 {
				return usage
			}
			min, err1 := strconv.ParseFloat(args[4], 64)
			max, err2 := strconv.ParseFloat(args[5], 64)
			if err1 != nil || err2 != nil {
				return usage
			}
			j.Min, j.Max = min, max
		case "case":
			angle, err := strconv.ParseFloat(args[4], 64)
			if err != nil || len(args) != 5 {
				return usage
			}
			j.CaseAngle = angle
		case "axis_offset":
			offset, err := strconv.ParseFloat(args[4], 64)
			if err != nil || len(args) != 5 {
				return usage
			}
			j.AxisOffset = offset
		default:
			return usage
		}
	}

	if err := changed.Validate(s.Pod.BodyDefinition.NumLegs); err != nil {
		return err
	}
	*m = changed
	return nil
}
