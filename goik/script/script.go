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

// Package script parses and runs motion scripts: sequences of gait engine commands.
//
// A script is a text file with one command per line. Everything after '#' is a comment.
//
//	gait <tripod|ripple|wave>          blend into a new gait
//	walk <x> <y> <yaw> [for <seconds> | cycles <n>]
//	                                   set the body velocity (mm/s, mm/s, degrees/s), optionally for
//	                                   a while or a number of gait cycles
//	wait <seconds>                     keep the current motion for a while
//	halt                               stop and wait until the pod has settled in its neutral stance
//	swing_time <seconds>               duration of a leg swing
//	step_height <mm>                   height of the swing arc
//	pitch | roll | yaw <degrees>       rotate the body (see robot/bodyPose.go for the axes)
//	up | down <mm>                     raise or lower the body relative to the neutral stance
//	shift <x> <y>                      move the body (and its centre of gravity) in mm
//	level                              return the body to the neutral pose
//	repeat [count]                     run the block since the previous repeat (or the start of the
//	                                   script) count times in total. Without a count: forever
//
// Body pose commands set a target that the body moves towards while the script continues.
// halt waits for both the legs and the body to settle. The pod halts when the script ends.
//
// Scripts run in robot time: the runner is advanced by the same time step as the gait engine,
// so a script looks the same at any simulator speed.
package script

import (
	"GOIK/robot"
	"bufio"
	"fmt"
	"io"
	"math"
	"os"
	"strconv"
	"strings"
)

// HALT_TIMEOUT is the maximum time (in seconds) a halt command waits for the pod to settle
const HALT_TIMEOUT = 30.0

// Command is a parsed script line
type Command struct {
	// Line number in the script (0 for the implicit halt at the end)
	Line int
	// The command as written in the script (without comments)
	Text string
	Name string
	// Numeric arguments (walk: x, y, yaw. wait, swing_time, step_height: one value. repeat: count, 0 == forever)
	Values []float64
	// Duration in seconds (walk ... for <seconds>)
	Duration float64
	// Number of gait cycles (walk ... cycles <n>)
	Cycles float64
	// Gait name (gait command)
	Gait string
}

// Target is what a script controls. *robot.GaitEngine implements it.
type Target interface {
	SetTwist(t robot.Twist)
	SetGaitByName(name string) error
	SetSwingTime(seconds float64) error
	SetStepHeight(mm float64) error
	SetBodyPose(b robot.BodyPose) error
	TargetBodyPose() robot.BodyPose
	// Number of gait cycles completed
	Cycles() float64
	IsIdle() bool
}

// Load parses a script file
func Load(path string) ([]Command, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	return Parse(f)
}

// Parse parses a script. All lines are checked before anything runs.
func Parse(r io.Reader) ([]Command, error) {
	var commands []Command
	scanner := bufio.NewScanner(r)
	line := 0
	for scanner.Scan() {
		line++
		text := scanner.Text()
		if i := strings.Index(text, "#"); i >= 0 {
			text = text[:i]
		}
		text = strings.TrimSpace(text)
		if text == "" {
			continue
		}

		c, err := parseCommand(strings.Fields(text))
		if err != nil {
			return nil, fmt.Errorf("line %d: %w", line, err)
		}
		c.Line = line
		c.Text = text
		commands = append(commands, c)
	}
	if err := scanner.Err(); err != nil {
		return nil, err
	}
	return commands, nil
}

func parseCommand(fields []string) (Command, error) {
	c := Command{Name: strings.ToLower(fields[0])}
	args := fields[1:]

	switch c.Name {
	case "gait":
		if len(args) != 1 {
			return c, fmt.Errorf("syntax: gait <tripod|ripple|wave>")
		}
		if _, err := robot.ParseGaitType(args[0]); err != nil {
			return c, err
		}
		c.Gait = args[0]

	case "walk":
		suffix := ""
		if len(args) == 5 {
			suffix = strings.ToLower(args[3])
		}
		if len(args) != 3 && suffix != "for" && suffix != "cycles" {
			return c, fmt.Errorf("syntax: walk <x> <y> <yaw> [for <seconds> | cycles <n>]")
		}
		values, err := parseNumbers(args[:3])
		if err != nil {
			return c, err
		}
		c.Values = values
		if len(args) == 5 {
			d, err := parseNumbers(args[4:])
			if err != nil {
				return c, err
			}
			if d[0] < 0 && suffix == "for" {
				return c, fmt.Errorf("duration can not be negative")
			}
			if d[0] < 0 {
				return c, fmt.Errorf("number of cycles can not be negative")
			}
			if suffix == "for" {
				c.Duration = d[0]
			} else {
				c.Cycles = d[0]
			}
		}

	case "pitch", "roll", "yaw", "up", "down":
		if len(args) != 1 {
			return c, fmt.Errorf("syntax: %s <value>", c.Name)
		}
		values, err := parseNumbers(args)
		if err != nil {
			return c, err
		}
		c.Values = values

	case "shift":
		if len(args) != 2 {
			return c, fmt.Errorf("syntax: shift <x> <y>")
		}
		values, err := parseNumbers(args)
		if err != nil {
			return c, err
		}
		c.Values = values

	case "level":
		if len(args) != 0 {
			return c, fmt.Errorf("syntax: level")
		}

	case "wait", "swing_time", "step_height":
		if len(args) != 1 {
			return c, fmt.Errorf("syntax: %s <value>", c.Name)
		}
		values, err := parseNumbers(args)
		if err != nil {
			return c, err
		}
		if values[0] < 0 {
			return c, fmt.Errorf("%s can not be negative", c.Name)
		}
		c.Values = values

	case "halt":
		if len(args) != 0 {
			return c, fmt.Errorf("syntax: halt")
		}

	case "repeat":
		if len(args) > 1 {
			return c, fmt.Errorf("syntax: repeat [count]")
		}
		c.Values = []float64{0}
		if len(args) == 1 {
			count, err := strconv.Atoi(args[0])
			if err != nil || count < 1 {
				return c, fmt.Errorf("repeat count must be a whole number larger than 0")
			}
			c.Values[0] = float64(count)
		}

	default:
		return c, fmt.Errorf("unknown command '%s'", fields[0])
	}
	return c, nil
}

func parseNumbers(args []string) ([]float64, error) {
	values := make([]float64, len(args))
	for i, a := range args {
		v, err := strconv.ParseFloat(a, 64)
		if err != nil || math.IsNaN(v) || math.IsInf(v, 0) {
			return nil, fmt.Errorf("'%s' is not a number", a)
		}
		values[i] = v
	}
	return values, nil
}

// Runner executes a script against a target, advanced by Tick
type Runner struct {
	Name     string
	commands []Command
	target   Target
	log      func(string)

	pc            int
	current       *Command
	waitRemaining float64
	// Wait until the target has completed this many gait cycles (0 == not waiting)
	waitCycles  float64
	waitingIdle bool
	idleWaited  float64
	// Number of completed passes through the block before each repeat command
	repeats map[int]int
	done    bool
}

// NewRunner creates a runner for the commands. log receives a message for each executed command.
func NewRunner(name string, commands []Command, target Target, log func(string)) *Runner {
	// The pod always halts at the end of a script
	commands = append(append([]Command(nil), commands...), Command{Name: "halt", Text: "halt (end of script)"})
	return &Runner{
		Name:     name,
		commands: commands,
		target:   target,
		log:      log,
		repeats:  map[int]int{},
	}
}

// Done returns true when the script has finished (or failed, or was stopped)
func (r *Runner) Done() bool {
	return r.done
}

// Stop aborts the script and halts the pod
func (r *Runner) Stop() {
	if !r.done {
		r.target.SetTwist(robot.Twist{})
		r.done = true
	}
}

// Status describes what the script is doing
func (r *Runner) Status() string {
	if r.done {
		return fmt.Sprintf("%s: finished", r.Name)
	}
	if r.current == nil {
		return fmt.Sprintf("%s: starting", r.Name)
	}
	status := fmt.Sprintf("%s line %d: %s", r.Name, r.current.Line, r.current.Text)
	if r.waitRemaining > 0 {
		status += fmt.Sprintf(" (%2.1f s left)", r.waitRemaining)
	} else if r.waitCycles > 0 {
		status += fmt.Sprintf(" (%2.1f cycles left)", math.Max(0, r.waitCycles-r.target.Cycles()))
	} else if r.waitingIdle {
		status += " (settling)"
	}
	return status
}

// Tick advances the script dt seconds. On error the script stops and the pod halts.
func (r *Runner) Tick(dt float64) error {
	if r.done {
		return nil
	}
	err := r.tick(dt)
	if err != nil {
		r.Stop()
		if r.current != nil && r.current.Line > 0 {
			err = fmt.Errorf("%s line %d: %w", r.Name, r.current.Line, err)
		}
	}
	return err
}

func (r *Runner) tick(dt float64) error {
	if r.waitRemaining > 0 {
		r.waitRemaining -= dt
		// Tolerate rounding errors from summing many time steps
		if r.waitRemaining > 1e-9 {
			return nil
		}
		r.waitRemaining = 0
	}

	if r.waitCycles > 0 {
		// An idle pod (no velocity) never completes a cycle
		if r.target.Cycles() < r.waitCycles-1e-9 && !r.target.IsIdle() {
			return nil
		}
		r.waitCycles = 0
	}

	if r.waitingIdle {
		if !r.target.IsIdle() {
			r.idleWaited += dt
			if r.idleWaited > HALT_TIMEOUT {
				return fmt.Errorf("the pod did not settle within %2.0f seconds", HALT_TIMEOUT)
			}
			return nil
		}
		r.waitingIdle = false
	}

	// Run commands until one of them has to wait
	for executed := 0; r.pc < len(r.commands); executed++ {
		if executed > len(r.commands) {
			return fmt.Errorf("the script repeats forever without waiting")
		}
		r.current = &r.commands[r.pc]
		r.pc++
		if r.log != nil {
			r.log(fmt.Sprintf("[%s] %s", r.Name, r.current.Text))
		}
		if err := r.execute(r.current); err != nil {
			return err
		}
		if r.waitRemaining > 0 || r.waitCycles > 0 || r.waitingIdle {
			return nil
		}
	}

	r.done = true
	return nil
}

func (r *Runner) execute(c *Command) error {
	switch c.Name {
	case "gait":
		return r.target.SetGaitByName(c.Gait)
	case "walk":
		r.target.SetTwist(robot.Twist{X: c.Values[0], Y: c.Values[1], Yaw: c.Values[2]})
		r.waitRemaining = c.Duration
		if c.Cycles > 0 {
			r.waitCycles = r.target.Cycles() + c.Cycles
		}
	case "pitch", "roll", "yaw", "up", "down", "shift", "level":
		return r.target.SetBodyPose(PoseFor(c, r.target.TargetBodyPose()))
	case "wait":
		r.waitRemaining = c.Values[0]
	case "halt":
		r.target.SetTwist(robot.Twist{})
		r.waitingIdle = true
		r.idleWaited = 0
	case "swing_time":
		return r.target.SetSwingTime(c.Values[0])
	case "step_height":
		return r.target.SetStepHeight(c.Values[0])
	case "repeat":
		index := r.pc - 1
		r.repeats[index]++
		count := int(c.Values[0])
		if count == 0 || r.repeats[index] < count {
			r.pc = r.blockStart(index)
		} else {
			r.repeats[index] = 0
		}
	}
	return nil
}

// PoseFor returns the body pose after applying a pose command to the current target pose
func PoseFor(c *Command, pose robot.BodyPose) robot.BodyPose {
	switch c.Name {
	case "pitch":
		pose.Pitch = c.Values[0]
	case "roll":
		pose.Roll = c.Values[0]
	case "yaw":
		pose.Yaw = c.Values[0]
	case "up":
		// Z is positive towards the ground
		pose.Z = -c.Values[0]
	case "down":
		pose.Z = c.Values[0]
	case "shift":
		pose.X, pose.Y = c.Values[0], c.Values[1]
	case "level":
		pose = robot.BodyPose{}
	}
	return pose
}

// blockStart returns the index of the first command after the previous repeat (or 0)
func (r *Runner) blockStart(repeatIndex int) int {
	for i := repeatIndex - 1; i >= 0; i-- {
		if r.commands[i].Name == "repeat" {
			return i + 1
		}
	}
	return 0
}
