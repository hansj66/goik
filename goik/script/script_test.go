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

package script

import (
	"GOIK/robot"
	"strings"
	"testing"
)

func TestParseErrors(t *testing.T) {
	tests := []struct {
		script string
		error  string
	}{
		{"jump 1", "line 1: unknown command"},
		{"# comment\n\nwalk 1 2", "line 3: syntax: walk"},
		{"walk 1 2 3 during 4", "line 1: syntax: walk"},
		{"walk 1 x 3", "line 1: 'x' is not a number"},
		{"walk 1 2 3 for -1", "duration can not be negative"},
		{"gait gallop", "unknown gait 'gallop'"},
		{"repeat 0", "repeat count must be"},
		{"wait", "syntax: wait"},
		{"halt now", "syntax: halt"},
	}
	for _, tt := range tests {
		_, err := Parse(strings.NewReader(tt.script))
		if err == nil || !strings.Contains(err.Error(), tt.error) {
			t.Errorf("Parse(%q) error = %v, want it to contain %q", tt.script, err, tt.error)
		}
	}
}

func TestParse(t *testing.T) {
	commands, err := Parse(strings.NewReader("GAIT wave  # comment\nwalk 1 -2 3.5 for 2\n\nrepeat\n"))
	if err != nil {
		t.Fatal(err)
	}
	if len(commands) != 3 {
		t.Fatalf("got %d commands, want 3", len(commands))
	}
	if c := commands[0]; c.Name != "gait" || c.Gait != "wave" || c.Line != 1 || c.Text != "GAIT wave" {
		t.Errorf("unexpected gait command: %+v", c)
	}
	if c := commands[1]; c.Values[0] != 1 || c.Values[1] != -2 || c.Values[2] != 3.5 || c.Duration != 2 {
		t.Errorf("unexpected walk command: %+v", c)
	}
	if c := commands[2]; c.Line != 4 || c.Values[0] != 0 {
		t.Errorf("unexpected repeat command: %+v", c)
	}
}

// fakeTarget records the commands it receives. It becomes idle one tick after a zero velocity is set.
type fakeTarget struct {
	twists []robot.Twist
	gaits  []string
	moving bool
	pose   robot.BodyPose
	// Advances one gait cycle per 50 ticks while moving
	cycles float64
}

func (f *fakeTarget) SetTwist(t robot.Twist) {
	f.twists = append(f.twists, t)
	f.moving = !t.IsZero()
}
func (f *fakeTarget) SetGaitByName(name string) error    { f.gaits = append(f.gaits, name); return nil }
func (f *fakeTarget) SetSwingTime(seconds float64) error { return nil }
func (f *fakeTarget) SetStepHeight(mm float64) error     { return nil }
func (f *fakeTarget) IsIdle() bool                       { return !f.moving }
func (f *fakeTarget) SetBodyPose(b robot.BodyPose) error { f.pose = b; return nil }
func (f *fakeTarget) TargetBodyPose() robot.BodyPose     { return f.pose }
func (f *fakeTarget) Cycles() float64                    { return f.cycles }

func run(t *testing.T, script string, maxTicks int) (*fakeTarget, *Runner, int, error) {
	t.Helper()
	commands, err := Parse(strings.NewReader(script))
	if err != nil {
		t.Fatal(err)
	}
	target := &fakeTarget{}
	r := NewRunner("test", commands, target, nil)
	ticks := 0
	for ; !r.Done() && ticks < maxTicks; ticks++ {
		if err := r.Tick(robot.ENGINE_DT); err != nil {
			return target, r, ticks, err
		}
		if target.moving {
			target.cycles += 1.0 / 50
		}
	}
	return target, r, ticks, nil
}

func TestRunnerTiming(t *testing.T) {
	_, r, ticks, err := run(t, "walk 0 10 0 for 1\nwait 0.5", 1000)
	if err != nil {
		t.Fatal(err)
	}
	if !r.Done() {
		t.Fatal("script did not finish")
	}
	// walk runs on ticks 1-50 (1 s at 50 Hz), wait on ticks 51-75 (0.5 s), the final halt
	// starts on tick 76 and the runner sees the (fake) pod idle on tick 77
	if want := 77; ticks != want {
		t.Errorf("script took %d ticks, want %d", ticks, want)
	}
}

func TestRunnerRepeatBlocks(t *testing.T) {
	target, _, _, err := run(t, "gait tripod\nwalk 1 0 0 for 0.1\nrepeat 3\ngait wave\nwalk 2 0 0 for 0.1\nrepeat 2", 1000)
	if err != nil {
		t.Fatal(err)
	}
	var walks []float64
	for _, tw := range target.twists {
		if !tw.IsZero() {
			walks = append(walks, tw.X)
		}
	}
	want := []float64{1, 1, 1, 2, 2}
	if len(walks) != len(want) {
		t.Fatalf("walk velocities = %v, want %v", walks, want)
	}
	for i := range want {
		if walks[i] != want[i] {
			t.Fatalf("walk velocities = %v, want %v", walks, want)
		}
	}
	if len(target.gaits) != 5 {
		t.Errorf("gaits = %v, want 3 x tripod, 2 x wave", target.gaits)
	}
	if last := target.twists[len(target.twists)-1]; !last.IsZero() {
		t.Error("the pod should halt at the end of the script")
	}
}

func TestRunnerRepeatForever(t *testing.T) {
	_, r, ticks, err := run(t, "walk 0 10 0 for 1\nrepeat", 1000)
	if err != nil {
		t.Fatal(err)
	}
	if r.Done() || ticks != 1000 {
		t.Error("script with repeat (no count) should run forever")
	}
}

func TestRunnerDetectsLoopWithoutWaiting(t *testing.T) {
	_, r, _, err := run(t, "gait tripod\nrepeat", 10)
	if err == nil || !strings.Contains(err.Error(), "without waiting") {
		t.Errorf("error = %v, want a 'repeats forever without waiting' error", err)
	}
	if !r.Done() {
		t.Error("runner should stop after an error")
	}
}

func TestRunnerStop(t *testing.T) {
	commands, _ := Parse(strings.NewReader("walk 0 10 0 for 10"))
	target := &fakeTarget{}
	r := NewRunner("test", commands, target, nil)
	r.Tick(robot.ENGINE_DT)
	r.Stop()
	if !r.Done() || !target.IsIdle() {
		t.Error("stop should end the script and halt the pod")
	}
}

// The example script runs to the end on a real pod without IK errors
func TestDemoScriptOnGaitEngine(t *testing.T) {
	commands, err := Load("../scripts/demo.goik")
	if err != nil {
		t.Fatal(err)
	}

	p := robot.NewPod(robot.NewExampleHexapod1())
	p.SetDebugChannel(make(chan string, 1000))
	engine, err := robot.NewGaitEngine(p)
	if err != nil {
		t.Fatal(err)
	}
	p.Engine = engine

	r := NewRunner("demo", commands, engine, nil)
	ticks := 0
	for ; !r.Done() && ticks < 10000; ticks++ {
		if err := r.Tick(robot.ENGINE_DT); err != nil {
			t.Fatal(err)
		}
		p.Update()
	}

	if !r.Done() {
		t.Fatalf("script did not finish: %s", r.Status())
	}
	if !engine.IsIdle() {
		t.Error("pod should be idle after the script")
	}
	if engine.IKErrors > 0 {
		t.Errorf("%d IK errors, last: %v", engine.IKErrors, engine.LastError)
	}
}

func TestRunnerWalkCycles(t *testing.T) {
	_, r, ticks, err := run(t, "walk 0 10 0 cycles 2", 1000)
	if err != nil {
		t.Fatal(err)
	}
	if !r.Done() {
		t.Fatal("script did not finish")
	}
	// The fake completes a cycle per 50 ticks: 100 ticks of walking, then the final halt
	if ticks < 100 || ticks > 103 {
		t.Errorf("script took %d ticks, want about 101", ticks)
	}
}

func TestRunnerBodyPose(t *testing.T) {
	target, _, _, err := run(t, "pitch 10\nroll -5\nup 20\nshift 3 4\nyaw 15\nwait 0.1", 100)
	if err != nil {
		t.Fatal(err)
	}
	want := robot.BodyPose{Pitch: 10, Roll: -5, Yaw: 15, X: 3, Y: 4, Z: -20}
	if target.pose != want {
		t.Errorf("pose = %v, want %v", target.pose, want)
	}

	target, _, _, _ = run(t, "pitch 10\ndown 5\nlevel", 100)
	if !target.pose.IsZero() {
		t.Errorf("level should reset the pose, got %v", target.pose)
	}
}
