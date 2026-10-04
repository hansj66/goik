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
	"log"
	"strings"
	"sync"

	"github.com/borud/chatui"
)

type dispatchFunc func(args []string) error

type Shell struct {
	Pod         *robot.Pod
	outputCh    chan string
	commandCh   chan string
	dispatchMap map[string]dispatchFunc
	// Script is the running motion script (nil if none)
	Script *script.Runner
	// The engine the script was started on
	scriptEngine *robot.GaitEngine
	// mu protects Pod and Script. Commands run on the shell goroutine, while
	// Ebiten updates and draws on its own goroutine
	mu sync.Mutex
}

func NewShell(pod *robot.Pod) *Shell {
	s := Shell{Pod: pod, outputCh: make(chan string, 10), commandCh: make(chan string)}

	s.Pod.SetDebugChannel(s.outputCh)

	s.dispatchMap = map[string]dispatchFunc{
		"help":             s.executeHelpCmd,
		"effectors":        s.executeEffectorsCmd,
		"set_coxa_length":  s.executeSetCoxaLengthCmd,
		"set_femur_length": s.executeSetFemurLengthCmd,
		"set_tibia_length": s.executeSetTibiaLengthCmd,
		"set_coxa_angle":   s.executeSetCoxaAngleCmd,
		"set_femur_angle":  s.executeSetFemurAngleCmd,
		"set_tibia_angle":  s.executeSetTibiaAngleCmd,
		"reset":            s.executeResetCmd,
		"speed":            s.executeSpeedCmd,
		"gait":             s.executeGaitCmd,
		"save":             s.executeSaveCmd,
		"load":             s.executeLoadCmd,
		"zero":             s.executeZeroCmd,
		"record":           s.executeRecordCmd,
		"export":           s.executeExportCmd,
		"debug":            s.executeDebugCmd,
		"ground":           s.executeGroundCmd,
		"walk":             s.executeWalkCmd,
		"halt":             s.executeHaltCmd,
		"swing_time":       s.executeSwingTimeCmd,
		"engine":           s.executeEngineCmd,
		"step_height":      s.executeStepHeightCmd,
		"pitch":            s.executePoseCmd,
		"roll":             s.executePoseCmd,
		"yaw":              s.executePoseCmd,
		"up":               s.executePoseCmd,
		"down":             s.executePoseCmd,
		"shift":            s.executePoseCmd,
		"level":            s.executePoseCmd,
		"run":              s.executeRunCmd,
		"abort":            s.executeAbortCmd,
		"scripts":          s.executeScriptsCmd,
		"servos":           s.executeServosCmd,
		"servo":            s.executeServoCmd,
		"servo_model":      s.executeServoModelCmd,
	}

	return &s
}

func (s *Shell) Run() {

	chatui := chatui.New(chatui.Config{
		OutputCh:     s.outputCh,
		CommandCh:    s.commandCh,
		DynamicColor: false,
		BlockCtrlC:   true,
		HistorySize:  10,
	})

	s.outputCh <- "Pod playground"

	go func() {
		for command := range s.commandCh {
			if strings.ToLower(command) == "/quit" {
				chatui.Stop()
			}
			err := s.Dispatch(command)
			if err != nil {
				s.outputCh <- err.Error()
			}
			chatui.SetStatus("last command was: " + command)
		}
	}()

	go func() {
		// this is done in a goroutine because it will block if the UI is not running.
		chatui.SetStatus("type /quit to exit")

		s.outputCh <- "-----------------------------------"
		s.outputCh <- "Hexapod Designer"
		s.outputCh <- "Copyright Hans Jørgen Grimstad 2024"
		s.outputCh <- "www.TimeExpander.com"
		s.outputCh <- "-----------------------------------"
		s.outputCh <- ""
		s.outputCh <- "Type 'help' for a list of commands"

	}()

	err := chatui.Run()
	if err != nil {
		log.Fatal(err)
	}
}
