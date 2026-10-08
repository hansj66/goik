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

package robot

import (
	"math"
	"testing"
)

func TestNewPhaseGait(t *testing.T) {
	tests := []struct {
		name    string
		numLegs int
		gait    func() (*Gait, error)
		duty    float64
		offsets []float64
	}{
		{"hexapod tripod", 6, func() (*Gait, error) { return NewHexapodGait(TRIPOD) }, 0.5, []float64{0, 0.5, 0, 0.5, 0, 0.5}},
		{"hexapod ripple", 6, func() (*Gait, error) { return NewHexapodGait(RIPPLE) }, 2.0 / 3, []float64{0, 2.0 / 3, 1.0 / 3, 0, 2.0 / 3, 1.0 / 3}},
		{"hexapod wave", 6, func() (*Gait, error) { return NewHexapodGait(WAVE) }, 5.0 / 6, []float64{3.0 / 6, 4.0 / 6, 5.0 / 6, 0, 1.0 / 6, 2.0 / 6}},
		{"pentapod wave", 5, func() (*Gait, error) { return NewPentapodGait(WAVE) }, 0.8, []float64{0.8, 0.6, 0.4, 0.2, 0}},
		{"quadruped tripod", 4, func() (*Gait, error) { return NewGait(4, TRIPOD) }, 0.5, []float64{0, 0.5, 0, 0.5}},
		{"octopod tripod", 8, func() (*Gait, error) { return NewGait(8, TRIPOD) }, 0.5, []float64{0, 0.5, 0, 0.5, 0, 0.5, 0, 0.5}},
		{"octopod wave", 8, func() (*Gait, error) { return NewGait(8, WAVE) }, 7.0 / 8, []float64{7.0 / 8, 0, 1.0 / 8, 2.0 / 8, 3.0 / 8, 4.0 / 8, 5.0 / 8, 6.0 / 8}},
		{"nonapod ripple", 9, func() (*Gait, error) { return NewGait(9, RIPPLE) }, 2.0 / 3, []float64{0, 2.0 / 3, 1.0 / 3, 0, 2.0 / 3, 1.0 / 3, 0, 2.0 / 3, 1.0 / 3}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			g, err := tt.gait()
			if err != nil {
				t.Fatal(err)
			}
			pg, err := NewPhaseGait(g, tt.numLegs)
			if err != nil {
				t.Fatal(err)
			}
			if math.Abs(pg.DutyFactor-tt.duty) > 1e-9 {
				t.Errorf("duty factor = %f, want %f", pg.DutyFactor, tt.duty)
			}
			for i := range tt.offsets {
				if math.Abs(pg.Offsets[i]-tt.offsets[i]) > 1e-9 {
					t.Errorf("offsets = %v, want %v", pg.Offsets, tt.offsets)
					break
				}
			}
		})
	}
}

func TestNewPhaseGaitRejectsMissingLegs(t *testing.T) {
	g, _ := NewHexapodGait(WAVE)
	if _, err := NewPhaseGait(g, 8); err == nil {
		t.Error("expected an error for an 8 legged pod with a 6 legged gait")
	}
}

type step struct {
	duration float64
	twist    Twist
	gait     GaitType
}

type runStats struct {
	maxFootJump      float64
	maxFootJerk      float64
	unstableTicks    int
	minGroundedLegs  int
	ticksToSettle    int
	maxNeutralOffset float64
}

// runScenario chains a list of motions on a fresh pod and measures the motion quality
func runScenario(t *testing.T, body *BodyDefinition, steps []step) runStats {
	t.Helper()
	p := NewPod(body)
	p.SetDebugChannel(make(chan string, 1000))
	e, err := NewGaitEngine(p)
	if err != nil {
		t.Fatal(err)
	}
	p.Engine = e

	stats := runStats{minGroundedLegs: len(p.Legs)}
	previous := p.GetEndEffectorPositions()
	velocity := make([]Coordinate, len(previous))

	tick := func() {
		p.Update()
		current := p.GetEndEffectorPositions()
		for i := range current {
			v := NewCoordinate(current[i].X-previous[i].X, current[i].Y-previous[i].Y, current[i].Z-previous[i].Z)
			stats.maxFootJump = math.Max(stats.maxFootJump, distance(previous[i], current[i]))
			// Change of per tick displacement. A discontinuity shows up here even if the foot is slow
			stats.maxFootJerk = math.Max(stats.maxFootJerk, distance(v, velocity[i]))
			velocity[i] = v
		}
		previous = current

		grounded := groundedFeet(e)
		if len(grounded) < stats.minGroundedLegs {
			stats.minGroundedLegs = len(grounded)
		}
		if SupportMargin(grounded) < 0 {
			stats.unstableTicks++
		}
	}

	for _, s := range steps {
		g, err := NewGait(len(p.Legs), s.gait)
		if err != nil {
			t.Fatal(err)
		}
		pg, err := NewPhaseGait(g, len(p.Legs))
		if err != nil {
			t.Fatal(err)
		}
		if err := e.SetGait(pg); err != nil {
			t.Fatal(err)
		}
		e.SetTwist(s.twist)
		for i := 0; i < int(s.duration/ENGINE_DT); i++ {
			tick()
		}
	}

	// Halt and wait for the pod to settle in the neutral stance
	e.SetTwist(Twist{})
	for stats.ticksToSettle = 0; !e.IsIdle() && stats.ticksToSettle < 2000; stats.ticksToSettle++ {
		tick()
	}
	for i, l := range p.Legs {
		stats.maxNeutralOffset = math.Max(stats.maxNeutralOffset, distance(l.Joints[EFFECTOR_ORIGIN_INDEX], p.Legs[i].NeutralEffectorCoordinate))
	}

	if e.IKErrors > 0 {
		t.Errorf("%d IK errors, last: %v", e.IKErrors, e.LastError)
	}
	return stats
}

func groundedFeet(e *GaitEngine) []Coordinate {
	var feet []Coordinate
	for _, l := range e.Legs {
		if !l.Swinging {
			feet = append(feet, l.Foot)
		}
	}
	return feet
}

func TestGaitEngineChainedMotions(t *testing.T) {
	scenario := []step{
		{3, Twist{Y: 80}, TRIPOD},          // walk forward
		{3, Twist{Yaw: 20}, TRIPOD},        // turn on the spot
		{3, Twist{Y: 50, Yaw: 15}, TRIPOD}, // walk in an arc
		{6, Twist{Y: 40}, WAVE},            // slow down and change gait
		{6, Twist{X: 40}, RIPPLE},          // change direction and gait
		{4, Twist{X: -60, Y: 60}, TRIPOD},  // back to tripod, walking diagonally
	}

	bodies := map[string]func() *BodyDefinition{
		"hexapod0": NewExampleHexapod0,
		"hexapod1": NewExampleHexapod1,
		"hexapod2": NewExampleHexapod2,
		"ax12":     NewExampleHexapodAX12,
		"sts3215":  NewExampleHexapodSTS3215,
	}

	for name, body := range bodies {
		t.Run(name, func(t *testing.T) {
			stats := runScenario(t, body(), scenario)
			t.Logf("%+v", stats)

			// Fastest expected foot motion is a full swing: lift (sin^2) + smoothstep across the stride
			e, _ := NewGaitEngine(NewPod(body()))
			swingSpeed := math.Pi*e.StepHeight/e.SwingTime + 1.5*e.MaxStride/e.SwingTime
			if limit := 1.1 * swingSpeed * ENGINE_DT; stats.maxFootJump > limit {
				t.Errorf("foot moved %.1f mm in a single tick (limit %.1f)", stats.maxFootJump, limit)
			}
			// Largest expected change of foot velocity: peak lift and stride acceleration during a swing,
			// plus switching between stance speed and the (zero velocity) start/end of a swing.
			// The old table based gait jumped up to 60 mm in a single tick
			lift := 2 * math.Pi * math.Pi * e.StepHeight / (e.SwingTime * e.SwingTime)
			stride := 6 * e.MaxStride / (e.SwingTime * e.SwingTime)
			stanceSpeed := e.MaxStride / e.SwingTime // tripod: stance time == swing time
			if limit := 1.1 * ((lift+stride)*ENGINE_DT*ENGINE_DT + stanceSpeed*ENGINE_DT); stats.maxFootJerk > limit {
				t.Errorf("foot velocity changed by %.1f mm/tick in a single tick (limit %.1f)", stats.maxFootJerk, limit)
			}
			if stats.unstableTicks > 0 {
				t.Errorf("centre of gravity was outside the support polygon for %d ticks", stats.unstableTicks)
			}
			if stats.ticksToSettle >= 2000 {
				t.Errorf("pod did not settle after the halt command")
			}
			if stats.maxNeutralOffset > SETTLE_TOLERANCE {
				t.Errorf("feet settled %.2f mm from the neutral stance", stats.maxNeutralOffset)
			}
		})
	}
}

func TestGaitEngineRespectsMaxStride(t *testing.T) {
	p := NewPod(NewExampleHexapod1())
	e, _ := NewGaitEngine(p)
	e.MaxStride = 40

	limited := e.limitStride(Twist{Y: 1000})
	if stride := limited.Y * e.dutyFactor * e.CycleTime(); math.Abs(stride-40) > 1e-9 {
		t.Errorf("stride = %f, want 40", stride)
	}
}

func TestGaitEngineIdleWithoutVelocity(t *testing.T) {
	p := NewPod(NewExampleHexapod1())
	e, _ := NewGaitEngine(p)
	p.Engine = e
	before := p.GetEndEffectorPositions()

	for i := 0; i < 100; i++ {
		p.Update()
	}

	if !e.IsIdle() {
		t.Error("engine should be idle when no velocity has been commanded")
	}
	for i, c := range p.GetEndEffectorPositions() {
		if distance(c, before[i]) > 1e-9 {
			t.Errorf("leg %d moved while idle", i)
		}
	}
}

// Odometry and foot motion must agree: a grounded foot does not move in the world
func TestOdometryKeepsGroundedFeetFixed(t *testing.T) {
	p := NewPod(NewExampleHexapod1())
	e, _ := NewGaitEngine(p)
	p.Engine = e
	e.SetTwist(Twist{X: 20, Y: 60, Yaw: 15})

	type world struct{ x, y float64 }
	previous := make([]*world, len(e.Legs))
	checked := 0
	for i := 0; i < int(6/ENGINE_DT); i++ {
		p.Update()
		for l, leg := range e.Legs {
			if leg.Swinging {
				previous[l] = nil
				continue
			}
			x, y := e.GroundToWorld(leg.Foot.X, leg.Foot.Y)
			if previous[l] != nil {
				if d := math.Hypot(x-previous[l].x, y-previous[l].y); d > 1e-9 {
					t.Fatalf("tick %d: grounded foot %d moved %g mm in the world", i, l, d)
				}
				checked++
			}
			previous[l] = &world{x, y}
		}
	}
	if checked == 0 {
		t.Fatal("no grounded feet were checked")
	}

	// 6 s along an arc: the pod has travelled and turned
	x, y, heading := e.Odometry()
	if math.Hypot(x, y) < 100 || heading < 30 {
		t.Errorf("odometry = %f, %f, %f degrees; expected the pod to have travelled and turned", x, y, heading)
	}
	if gx, gy := e.WorldToGround(e.GroundToWorld(12, 34)); math.Abs(gx-12) > 1e-9 || math.Abs(gy-34) > 1e-9 {
		t.Errorf("WorldToGround(GroundToWorld(12, 34)) = %f, %f", gx, gy)
	}
}

func TestGeneratedGaitsRejectInvalidLegCounts(t *testing.T) {
	for _, tt := range []struct {
		numLegs int
		gait    GaitType
	}{{5, TRIPOD}, {9, TRIPOD}, {8, RIPPLE}, {2, WAVE}} {
		if _, err := NewGait(tt.numLegs, tt.gait); err == nil {
			t.Errorf("%d legs, gait %d: no error", tt.numLegs, tt.gait)
		}
	}
}

func TestGaitType(t *testing.T) {
	for _, want := range []GaitType{TRIPOD, RIPPLE, WAVE} {
		g, _ := NewGait(6, want)
		if got, err := g.Type(); err != nil || got != want {
			t.Errorf("%s: type %d (%v), want %d", g.Name, got, err, want)
		}
	}
}
