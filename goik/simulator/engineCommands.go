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
	"strconv"
	"strings"
)

// Commands for the phase based gait engine (robot/gaitEngine.go)

// gaitEngine returns the pod's gait engine, creating it from the current pose and gait if necessary
func (s *Shell) gaitEngine() (*robot.GaitEngine, error) {
	if s.Pod.Engine != nil {
		return s.Pod.Engine, nil
	}

	engine, err := robot.NewGaitEngine(s.Pod)
	if err != nil {
		return nil, err
	}
	s.Pod.Engine = engine
	s.outputCh <- fmt.Sprintf("Gait engine started. Max stride: %2.1f mm, reach: %2.1f mm", engine.MaxStride, engine.Reach)
	return engine, nil
}

func (s *Shell) executeWalkCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	commands, err := script.Parse(strings.NewReader(strings.Join(args, " ")))
	if err != nil {
		return err
	}
	c := commands[0]

	engine, err := s.gaitEngine()
	if err != nil {
		return err
	}
	s.stopScript()

	// "walk ... for <s>" and "walk ... cycles <n>" run as a one line script, which halts at the end
	if c.Duration > 0 || c.Cycles > 0 {
		s.startScript("walk", commands, engine)
		return nil
	}
	engine.SetTwist(robot.Twist{X: c.Values[0], Y: c.Values[1], Yaw: c.Values[2]})
	return nil
}

// executePoseCmd handles pitch, roll, yaw, up, down, shift and level
func (s *Shell) executePoseCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	commands, err := script.Parse(strings.NewReader(strings.Join(args, " ")))
	if err != nil {
		return err
	}

	engine, err := s.gaitEngine()
	if err != nil {
		return err
	}
	if err := engine.SetBodyPose(script.PoseFor(&commands[0], engine.TargetBodyPose())); err != nil {
		return err
	}
	s.outputCh <- fmt.Sprintf("Body pose: %s", engine.TargetBodyPose().String())
	return nil
}

func (s *Shell) executeStepHeightCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if len(args) != 2 {
		return fmt.Errorf("syntax error ('step_height <mm>'): %+v", args)
	}
	mm, err := strconv.ParseFloat(args[1], 64)
	if err != nil {
		return fmt.Errorf("syntax error ('step_height <mm>'): %+v", args)
	}

	engine, err := s.gaitEngine()
	if err != nil {
		return err
	}
	if err := engine.SetStepHeight(mm); err != nil {
		return err
	}
	s.outputCh <- fmt.Sprintf("Max stride: %2.1f mm, reach: %2.1f mm", engine.MaxStride, engine.Reach)
	return nil
}

func (s *Shell) executeHaltCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if s.Pod.Engine == nil {
		return fmt.Errorf("the gait engine is not running")
	}
	s.stopScript()
	s.Pod.Engine.SetTwist(robot.Twist{})
	return nil
}

func (s *Shell) executeSwingTimeCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if len(args) != 2 {
		return fmt.Errorf("syntax error ('swing_time <seconds>'): %+v", args)
	}

	seconds, err := strconv.ParseFloat(args[1], 64)
	if err != nil || seconds < 0.1 || seconds > 5 {
		return fmt.Errorf("syntax error ('swing_time <0.1-5>'): %+v", args)
	}

	engine, err := s.gaitEngine()
	if err != nil {
		return err
	}
	return engine.SetSwingTime(seconds)
}

func (s *Shell) executeEngineCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	engine := s.Pod.Engine
	if engine == nil {
		return fmt.Errorf("the gait engine is not running. Start it with 'walk'")
	}

	s.outputCh <- fmt.Sprintf("Velocity: %s, stride: %2.1f mm, cycle time: %2.2f s, cycles: %2.1f, idle: %t, transitioning: %t",
		engine.GetTwist().String(), engine.Stride(), engine.CycleTime(), engine.Cycles(), engine.IsIdle(), engine.IsTransitioning())
	s.outputCh <- fmt.Sprintf("Body pose: %s (target %s, limited by reach: %t)",
		engine.BodyPose().String(), engine.TargetBodyPose().String(), engine.PoseLimited)
	s.outputCh <- fmt.Sprintf("Max stride: %2.1f mm, reach: %2.1f mm, IK errors: %d", engine.MaxStride, engine.Reach, engine.IKErrors)
	if engine.LastError != nil {
		s.outputCh <- fmt.Sprintf("Last error: %v", engine.LastError)
	}
	for i, l := range engine.Legs {
		s.outputCh <- fmt.Sprintf("Leg %d: phase %1.2f, swinging: %t, foot: %s", i, l.Phase, l.Swinging, l.Foot.String())
	}
	return nil
}
