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
	"math"
	"strconv"
	"strings"
)

const stanceUsage = "stance [<height> [<reach> | vertical] | reach <leg> <reach | default> | off]"

// executeStanceCmd shows or changes the pod's stance (see docs/designing-a-pod.md)
func (s *Shell) executeStanceCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)
	if len(args) == 1 {
		return s.showStance()
	}

	b := s.Pod.BodyDefinition
	syntaxError := fmt.Errorf("syntax error ('%s'): %+v", stanceUsage, args)
	reach := func(arg string, keyword string) (float64, error) {
		if strings.ToLower(arg) == keyword {
			return 0, nil
		}
		r, err := strconv.ParseFloat(arg, 64)
		if err != nil {
			return 0, syntaxError
		}
		if r <= 0 {
			return 0, fmt.Errorf("the reach must be positive ('%s' for the default: the tibia vertical)", keyword)
		}
		return r, nil
	}

	var stance *robot.Stance
	switch strings.ToLower(args[1]) {
	case "off":
		if b.Stance == nil {
			return fmt.Errorf("the pod has no stance")
		}
		s.Pod.SetStance(nil)
		s.outputCh <- "The stance is removed. The femur and tibia rest angles stay as they are, and can be set by hand again"
		return nil

	case "reach":
		if b.Stance == nil {
			return fmt.Errorf("the pod has no stance. Set one first ('stance <height>')")
		}
		if len(args) != 4 {
			return syntaxError
		}
		leg, err := strconv.Atoi(args[2])
		if err != nil || leg < 0 || leg >= b.NumLegs {
			return fmt.Errorf("invalid leg index '%s' (the pod has %d legs)", args[2], b.NumLegs)
		}
		r, err := reach(args[3], "default")
		if err != nil {
			return err
		}
		stance = b.Stance.Clone()
		if len(stance.LegReach) == 0 {
			stance.LegReach = make([]float64, b.NumLegs)
		}
		stance.LegReach[leg] = r
		// A leg and its mirror image stand alike
		if b.Design != nil {
			if mirror := b.Design.Mirror(leg); mirror != -1 {
				stance.LegReach[mirror] = r
			}
		}
		overrides := false
		for _, r := range stance.LegReach {
			overrides = overrides || r > 0
		}
		if !overrides {
			stance.LegReach = nil
		}

	default:
		height, err := strconv.ParseFloat(args[1], 64)
		if err != nil || len(args) > 3 {
			return syntaxError
		}
		stance = &robot.Stance{}
		if b.Stance != nil {
			stance = b.Stance.Clone()
		}
		stance.Height = height
		if len(args) == 3 {
			if stance.Reach, err = reach(args[2], "vertical"); err != nil {
				return err
			}
		}
	}

	if err := s.Pod.SetStance(stance); err != nil {
		return err
	}
	s.stopScript()
	return s.showStance()
}

// showStance prints the pod's stance and what it gives
func (s *Shell) showStance() error {
	b := s.Pod.BodyDefinition
	st := b.Stance
	if st == nil {
		s.outputCh <- "The pod has no stance: the femur and tibia rest angles are set by hand. Set one with 'stance <height> [reach]'"
		return nil
	}

	describe := func(r float64) string {
		if r == 0 {
			return "tibia vertical"
		}
		return fmt.Sprintf("feet %.0f mm out from the femur joints", r)
	}
	s.outputCh <- fmt.Sprintf("Stance: body %.0f mm above the ground, %s", st.Height, describe(st.Reach))

	// The rest pose, whatever the pod is doing right now
	rest := robot.NewPod(b)
	for l, leg := range rest.Legs {
		femur, foot := leg.Joints[robot.FEMUR_ORIGIN_INDEX], leg.Joints[robot.EFFECTOR_ORIGIN_INDEX]
		override := ""
		if l < len(st.LegReach) && st.LegReach[l] > 0 {
			override = " (set for this leg)"
		}
		s.outputCh <- fmt.Sprintf("Leg %d: femur %.1f, tibia %.1f degrees, foot %.0f mm out from the femur joint%s",
			l, leg.ServoAngles.Femur, leg.ServoAngles.Tibia, math.Hypot(foot.X-femur.X, foot.Y-femur.Y), override)
	}

	s.outputCh <- fmt.Sprintf("The feet can step %.0f mm around the neutral stance (with the default step height of %.0f mm)",
		rest.ReachRadius(robot.Z_LIFT), robot.Z_LIFT)
	if lowest, highest, ok := robot.StanceHeightRange(b); ok {
		s.outputCh <- fmt.Sprintf("Possible heights with these reach settings: %.0f to %.0f mm", lowest, highest)
	}
	return nil
}
