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
	"strconv"
)

// Commands for the phase based gait engine (robot/gaitEngine.go)

// gaitEngine returns the pod's gait engine, creating it from the current pose and gait if necessary
func (s *Shell) gaitEngine() (*robot.GaitEngine, error) {
	if s.Pod.Engine != nil {
		return s.Pod.Engine, nil
	}
	if s.Pod.IsWalking || s.Pod.IsReverting {
		return nil, fmt.Errorf("the pod is running a stride. Use 'stop' and 'revert' first")
	}

	engine, err := robot.NewGaitEngine(s.Pod)
	if err != nil {
		return nil, err
	}
	s.Pod.Engine = engine
	s.outputCh <- fmt.Sprintf("Gait engine started. Max stride: %2.1f mm, reach: %2.1f mm", engine.MaxStride, engine.Reach)
	return engine, nil
}

// leaveGaitEngine switches back to the table based gait (stride_vector / stride_angle)
func (s *Shell) leaveGaitEngine() error {
	if s.Pod.Engine == nil {
		return nil
	}
	if !s.Pod.Engine.IsIdle() {
		return fmt.Errorf("the gait engine is running. Use 'halt' and wait for the pod to settle first")
	}
	s.stopScript()
	s.Pod.Engine = nil
	return nil
}

func (s *Shell) executeWalkCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if len(args) != 4 {
		return fmt.Errorf("syntax error ('walk <x mm/s> <y mm/s> <yaw deg/s>'): %+v", args)
	}

	var values [3]float64
	for i := range values {
		v, err := strconv.ParseFloat(args[i+1], 64)
		if err != nil {
			return fmt.Errorf("syntax error ('walk <x mm/s> <y mm/s> <yaw deg/s>'): %+v", args)
		}
		values[i] = v
	}

	engine, err := s.gaitEngine()
	if err != nil {
		return err
	}
	s.stopScript()
	engine.SetTwist(robot.Twist{X: values[0], Y: values[1], Yaw: values[2]})
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

	s.outputCh <- fmt.Sprintf("Velocity: %s, cycle time: %2.2f s, idle: %t, transitioning: %t",
		engine.GetTwist().String(), engine.CycleTime(), engine.IsIdle(), engine.IsTransitioning())
	s.outputCh <- fmt.Sprintf("Max stride: %2.1f mm, reach: %2.1f mm, IK errors: %d", engine.MaxStride, engine.Reach, engine.IKErrors)
	if engine.LastError != nil {
		s.outputCh <- fmt.Sprintf("Last error: %v", engine.LastError)
	}
	for i, l := range engine.Legs {
		s.outputCh <- fmt.Sprintf("Leg %d: phase %1.2f, swinging: %t, foot: %s", i, l.Phase, l.Swinging, l.Foot.String())
	}
	return nil
}
