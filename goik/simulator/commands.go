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
	"fmt"
	"os"
	"strconv"
	"strings"
)

const POD_FOLDER = "pods"

func (s *Shell) executeEffectorsCmd(args []string) error {
	s.outputCh <- "Current end effector positions:"

	positions := s.Pod.GetEndEffectorPositions()
	for _, p := range positions {
		s.outputCh <- p.String()
	}

	return nil
}

func (s *Shell) executeSetCoxaLengthCmd(args []string) error {
	return s.setLegValue(args, "set_coxa_length <ALL | legNum> <length>", "coxa length", (*robot.Pod).SetCoxaLength)
}

func (s *Shell) executeSetFemurLengthCmd(args []string) error {
	return s.setLegValue(args, "set_femur_length <ALL | legNum> <length>", "femur length", (*robot.Pod).SetFemurLength)
}

func (s *Shell) executeSetTibiaLengthCmd(args []string) error {
	return s.setLegValue(args, "set_tibia_length <ALL | legNum> <length>", "tibia length", (*robot.Pod).SetTibiaLength)
}

func (s *Shell) executeSetCoxaAngleCmd(args []string) error {
	return s.setLegValue(args, "set_coxa_angle <ALL | legNum> <angle>", "coxa angle", (*robot.Pod).SetCoxaAngle)
}

func (s *Shell) executeSetFemurAngleCmd(args []string) error {
	return s.setLegValue(args, "set_femur_angle <ALL | legNum> <angle>", "femur angle", (*robot.Pod).SetFemurAngle)
}

func (s *Shell) executeSetTibiaAngleCmd(args []string) error {
	return s.setLegValue(args, "set_tibia_angle <ALL | legNum> <angle>", "tibia angle", (*robot.Pod).SetTibiaAngle)
}

// setLegValue sets a segment length or rest angle of one leg, or of ALL legs. When the pod has a design, a
// leg's mirror image changes with it, so ALL changes the legs on the +X side and the legs on the axis.
func (s *Shell) setLegValue(args []string, usage string, what string, set func(*robot.Pod, int, float64) error) error {
	s.outputCh <- fmt.Sprintf("%+v", args)
	if len(args) != 3 {
		return fmt.Errorf("syntax error ('%s'): %+v", usage, args)
	}
	value, err := strconv.ParseFloat(args[2], 64)
	if err != nil {
		return fmt.Errorf("syntax error ('%s'): %+v", usage, args)
	}

	design := s.Pod.BodyDefinition.Design
	changed := func(leg int) string {
		message := fmt.Sprintf("Changing %s of leg %d to %2.2f", what, leg, value)
		if design != nil {
			if mirror := design.Mirror(leg); mirror != -1 {
				message += fmt.Sprintf(" (and of leg %d, its mirror image)", mirror)
			}
		}
		return message
	}

	if strings.ToUpper(args[1]) == "ALL" {
		var legs []robot.DesignLeg
		if design != nil {
			legs, _ = design.Legs()
		}
		for leg := 0; leg < s.Pod.BodyDefinition.NumLegs; leg++ {
			if leg < len(legs) && legs[leg].Mirrored {
				continue
			}
			if err := set(s.Pod, leg, value); err != nil {
				s.outputCh <- fmt.Sprintf("Leg %d: %v", leg, err)
				continue
			}
			s.outputCh <- changed(leg)
		}
		return nil
	}

	legnum, err := strconv.Atoi(args[1])
	if err != nil {
		return fmt.Errorf("syntax error ('%s'): %+v", usage, args)
	}
	if legnum < 0 || legnum >= s.Pod.BodyDefinition.NumLegs {
		return fmt.Errorf("invalid leg index. (Pod has %d legs. Indexing is 0 based)", s.Pod.BodyDefinition.NumLegs)
	}
	if err := set(s.Pod, legnum, value); err != nil {
		return err
	}
	s.outputCh <- changed(legnum)
	return nil
}

func (s *Shell) Dispatch(command string) error {
	args := strings.Fields(command)
	if len(args) == 0 {
		return nil
	}

	execute, ok := s.dispatchMap[args[0]]
	if !ok {
		return fmt.Errorf("unknown command: '%s'", args[0])
	}

	s.mu.Lock()
	defer s.mu.Unlock()
	return execute(args)
}

func (s *Shell) executeResetCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if len(args) != 2 {
		return fmt.Errorf("syntax error ('reset <0-7>'): %+v", args)
	}

	s.stopScript()

	if args[1] == "0" {
		s.Pod = robot.NewPod(robot.NewExampleHexapod0())
	} else if args[1] == "1" {
		s.Pod = robot.NewPod(robot.NewExampleHexapod1())
	} else if args[1] == "2" {
		s.Pod = robot.NewPod(robot.NewExampleHexapod2())
	} else if args[1] == "3" {
		s.Pod = robot.NewPod(robot.NewExamplePentapod())
	} else if args[1] == "4" {
		s.Pod = robot.NewPod(robot.NewHeptapod())
	} else if args[1] == "5" {
		s.Pod = robot.NewPod(robot.NewSpider())
	} else if args[1] == "6" {
		s.Pod = robot.NewPod(robot.NewExampleHexapodAX12())
	} else if args[1] == "7" {
		s.Pod = robot.NewPod(robot.NewExampleHexapodSTS3215())
	} else {
		return fmt.Errorf("Unknown example preset")
	}

	s.Pod.Update()
	s.Pod.SetDebugChannel(s.outputCh)

	return nil
}

func (s *Shell) executeSpeedCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if len(args) != 2 {
		return fmt.Errorf("syntax error ('speed <1-10>'): %+v", args)
	}

	speed, err := strconv.ParseFloat(args[1], 64)
	if err != nil || speed < 1 || speed > 10 {
		return fmt.Errorf("syntax error ('speed <1-10>'): %+v", args)
	}

	DELAY_COUNTER = 10 - int(speed)

	return nil
}

func (s *Shell) executeGaitCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if len(args) != 2 {
		return fmt.Errorf("syntax error ('gait <tripod|ripple|wave>'): %+v", args)
	}

	// The gait engine blends into the new gait while walking
	if s.Pod.Engine != nil {
		return s.Pod.Engine.SetGaitByName(args[1])
	}

	gaitType, err := robot.ParseGaitType(args[1])
	if err != nil {
		return err
	}
	gait, err := robot.NewGait(s.Pod.BodyDefinition.NumLegs, gaitType)
	if err != nil {
		return err
	}
	// Make sure the gait engine can use it once the pod starts walking
	if _, err := robot.NewPhaseGait(gait, s.Pod.BodyDefinition.NumLegs); err != nil {
		return err
	}
	s.Pod.BodyDefinition.Gait = gait
	return nil
}

func (s *Shell) executeSaveCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	exists, err := folderExists(fmt.Sprintf("./%s", POD_FOLDER))
	if err != nil {
		return err
	}

	if !exists {
		err := os.Mkdir(fmt.Sprintf("%s", POD_FOLDER), 0755)
		if err != nil {
			return err
		}
	}

	if len(args) != 2 {
		return fmt.Errorf("syntax error ('save <filename>'): %+v", args)
	}

	return s.Pod.BodyDefinition.Save(fmt.Sprintf("./%s/%s", POD_FOLDER, args[1]))
}

func (s *Shell) executeLoadCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if len(args) != 2 {
		return fmt.Errorf("syntax error ('load <filename>'): %+v", args)
	}

	definition, err := s.Pod.BodyDefinition.Load(fmt.Sprintf("./%s/%s", POD_FOLDER, args[1]))
	if err != nil {
		return err
	}

	s.stopScript()
	s.Pod.LoadBodyDefinition(definition)

	return nil
}

func folderExists(path string) (bool, error) {
	_, err := os.Stat(path)
	if err == nil {
		return true, nil
	}
	if os.IsNotExist(err) {
		return false, nil
	}
	return false, err
}
