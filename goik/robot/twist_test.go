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
	"math/rand"
	"strings"
	"testing"
)

// twistedHexapod is example pod 6 with twisted joints on every leg
func twistedHexapod(twists JointTwists) *BodyDefinition {
	b := NewExampleHexapodAX12()
	for l := 0; l < b.NumLegs; l++ {
		b.Twists = append(b.Twists, twists)
	}
	return b
}

func distance3(a, b Coordinate) float64 {
	return math.Sqrt((a.X-b.X)*(a.X-b.X) + (a.Y-b.Y)*(a.Y-b.Y) + (a.Z-b.Z)*(a.Z-b.Z))
}

func TestLegModelMatchesForwardKinematics(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	for i := 0; i < 200; i++ {
		twists := JointTwists{Coxa: rng.Float64()*90 - 45, Femur: rng.Float64()*90 - 45, Tibia: rng.Float64()*90 - 45}
		p := NewPod(twistedHexapod(twists))
		leg := p.Legs[i%6]
		angles := ServoAngles{Coxa: rng.Float64()*120 - 60, Femur: rng.Float64()*120 - 60, Tibia: rng.Float64()*120 - 60}
		leg.RecalculateForwardKinematics(angles)

		joints, _ := leg.model.forward([3]float64{angles.Coxa * math.Pi / 180, angles.Femur * math.Pi / 180, angles.Tibia * math.Pi / 180})
		for j := range joints {
			if d := distance3(joints[j].coordinate(), leg.Joints[j]); d > 1e-9 {
				t.Fatalf("twists %s, angles %+v: joint %d is %.2e mm off", twists.String(), angles, j, d)
			}
		}
	}
}

func TestTwistedIKFindsTheFoot(t *testing.T) {
	rng := rand.New(rand.NewSource(2))
	for i := 0; i < 500; i++ {
		twists := JointTwists{Coxa: rng.Float64()*90 - 45, Femur: rng.Float64()*90 - 45, Tibia: rng.Float64()*90 - 45}
		p := NewPod(twistedHexapod(twists))
		leg := p.Legs[i%6]
		rest := leg.ServoAngles

		// A pose up to 25 degrees per joint away from the rest pose, like the gait engine asks for
		want := ServoAngles{Coxa: rest.Coxa + rng.Float64()*50 - 25, Femur: rest.Femur + rng.Float64()*50 - 25, Tibia: rest.Tibia + rng.Float64()*50 - 25}
		leg.RecalculateForwardKinematics(want)
		target := leg.Joints[EFFECTOR_ORIGIN_INDEX]
		leg.RecalculateForwardKinematics(rest)

		got, err := SolveEffectorIK(leg, target, nil)
		if err != nil {
			t.Fatalf("twists %s: %v", twists.String(), err)
		}
		leg.RecalculateForwardKinematics(got)
		if d := distance3(leg.Joints[EFFECTOR_ORIGIN_INDEX], target); d > 1e-3 {
			t.Fatalf("twists %s: the foot is %.4f mm off", twists.String(), d)
		}
	}
}

func TestTwistedIKRejectsUnreachableTargets(t *testing.T) {
	p := NewPod(twistedHexapod(JointTwists{Femur: 20, Tibia: -10}))
	leg := p.Legs[0]
	far := leg.Joints[COXA_ORIGIN_INDEX]
	far.X += 400
	if _, err := SolveEffectorIK(leg, far, nil); err == nil {
		t.Errorf("a target 400 mm away was reached")
	}
}

func TestUntwistedLegsAreUnchanged(t *testing.T) {
	// No twists: the twisted kinematic chain is the original one
	plain := NewPod(NewExampleHexapodAX12())
	zero := NewPod(twistedHexapod(JointTwists{}))
	for l := range plain.Legs {
		for j := range plain.Legs[l].Joints {
			if d := distance3(plain.Legs[l].Joints[j], zero.Legs[l].Joints[j]); d > 1e-12 {
				t.Errorf("leg %d joint %d: %.2e mm off", l, j, d)
			}
		}
	}
}

func TestMirroredTwistsMirrorTheLeg(t *testing.T) {
	d, _ := NewRectangularDesign(3, 180, 100, SegmentLengths{Coxa: 40, Femur: 60, Tibia: 100}, ServoAngles{Femur: -20, Tibia: 100})
	for i := range d.Mounts {
		d.Mounts[i].Twists = JointTwists{Coxa: 10, Femur: 25, Tibia: -15}
		d.Mounts[i].Rest.Coxa = 20
	}
	b, err := d.BodyDefinition(nil)
	if err != nil {
		t.Fatal(err)
	}
	p := NewPod(b)

	check := func(what string) {
		t.Helper()
		for l, leg := range p.Legs {
			m := d.Mirror(l)
			for j := range leg.Joints {
				a, o := leg.Joints[j], p.Legs[m].Joints[j]
				if math.Abs(a.X+o.X) > 1e-9 || math.Abs(a.Y-o.Y) > 1e-9 || math.Abs(a.Z-o.Z) > 1e-9 {
					t.Fatalf("%s: legs %d and %d, joint %d: %v and %v are not mirror images", what, l, m, j, a, o)
				}
			}
		}
	}
	check("rest pose")

	// Mirror images move alike with the opposite coxa angle and the same femur and tibia angles
	legs, err := d.Legs()
	if err != nil {
		t.Fatal(err)
	}
	for l, leg := range p.Legs {
		a := ServoAngles{Coxa: 15, Femur: -30, Tibia: 70}
		if legs[l].Mirrored {
			a.Coxa = -a.Coxa
		}
		leg.RecalculateForwardKinematics(a)
	}
	check("moved")
}

func TestTwistCommandsAndLimits(t *testing.T) {
	d, _ := NewRoundDesign(6, 80, SegmentLengths{Coxa: 60, Femur: 75, Tibia: 130}, ServoAngles{Femur: -15, Tibia: 97})
	b, _ := d.BodyDefinition(nil)
	p := NewPod(b)

	if err := p.SetFemurTwist(1, 20); err != nil {
		t.Fatal(err)
	}
	if tw := p.BodyDefinition.LegTwists(1); tw.Femur != 20 {
		t.Errorf("leg 1: twists %s, want femur 20", tw.String())
	}
	if tw := p.BodyDefinition.LegTwists(2); tw.Femur != -20 {
		t.Errorf("leg 2 (mirror image): twists %s, want femur -20", tw.String())
	}
	if err := p.SetTibiaTwist(0, MAX_TWIST+1); err == nil || !strings.Contains(err.Error(), "larger than") {
		t.Errorf("error = %v, want the twist limit", err)
	}

	// A leg on the symmetry axis can't be twisted
	r, _ := NewRoundDesign(4, 60, SegmentLengths{Coxa: 40, Femur: 60, Tibia: 100}, ServoAngles{Femur: -20, Tibia: 90})
	rb, _ := r.BodyDefinition(nil)
	if err := NewPod(rb).SetCoxaTwist(1, 10); err == nil {
		t.Errorf("a leg on the symmetry axis was twisted")
	}

	// Without a design, only the leg itself
	plain := NewPod(NewExampleHexapodAX12())
	if err := plain.SetCoxaTwist(3, -12); err != nil {
		t.Fatal(err)
	}
	if plain.BodyDefinition.LegTwists(3).Coxa != -12 || plain.BodyDefinition.LegTwists(0).Coxa != 0 {
		t.Errorf("twists %v", plain.BodyDefinition.Twists)
	}

	// The design keeps the twists through import and save
	again, err := NewDesignFromBodyDefinition(p.BodyDefinition)
	if err != nil {
		t.Fatal(err)
	}
	rebuilt, _ := again.BodyDefinition(nil)
	for l := 0; l < rebuilt.NumLegs; l++ {
		if rebuilt.LegTwists(l) != p.BodyDefinition.LegTwists(l) {
			t.Errorf("leg %d: imported twists %s, want %s", l, rebuilt.LegTwists(l).String(), p.BodyDefinition.LegTwists(l).String())
		}
	}
}

func TestTwistedStance(t *testing.T) {
	d, _ := NewRectangularDesign(3, 180, 100, SegmentLengths{Coxa: 50, Femur: 70, Tibia: 120}, ServoAngles{Femur: -15, Tibia: 95})
	for i := range d.Mounts {
		d.Mounts[i].Twists = JointTwists{Coxa: 5, Femur: 20, Tibia: -10}
	}
	b, _ := d.BodyDefinition(nil)
	p := NewPod(b)
	if err := p.SetStance(&Stance{Height: 100, Reach: 80}); err != nil {
		t.Fatal(err)
	}
	checkStance(t, p, 100, []float64{80})

	// Changing a twist keeps the stance
	if err := p.SetTibiaTwist(0, -20); err != nil {
		t.Fatal(err)
	}
	checkStance(t, p, 100, []float64{80})
}

func TestTwistedPodWalks(t *testing.T) {
	// Balanced twists: the front feet reach forward and the rear feet back. The same twist on all legs moves all
	// feet the same way, so the centre of gravity ends up near one end and halting can deadlock (BUGS.md #48)
	d, _ := NewRectangularDesign(3, 180, 100, SegmentLengths{Coxa: 50, Femur: 70, Tibia: 120}, ServoAngles{Femur: -15, Tibia: 95})
	d.Mounts[0].Twists = JointTwists{Coxa: 10, Femur: -25, Tibia: 15}
	d.Mounts[1].Twists = JointTwists{Coxa: 10}
	d.Mounts[2].Twists = JointTwists{Coxa: 10, Femur: 25, Tibia: -15}
	b, _ := d.BodyDefinition(nil)
	p := NewPod(b)
	if err := p.SetStance(&Stance{Height: 100, Reach: 80}); err != nil {
		t.Fatal(err)
	}
	stats := runScenario(t, p.BodyDefinition, []step{
		{3, Twist{Y: 60}, TRIPOD},
		{3, Twist{Yaw: 20}, RIPPLE},
		{4, Twist{X: 30, Y: 30}, WAVE},
	})
	t.Logf("%+v", stats)
	if stats.unstableTicks > 0 {
		t.Errorf("centre of gravity was outside the support polygon for %d ticks", stats.unstableTicks)
	}
	if stats.maxNeutralOffset > SETTLE_TOLERANCE {
		t.Errorf("feet settled %.2f mm from the neutral stance", stats.maxNeutralOffset)
	}
	if stats.ikErrors > 0 {
		t.Errorf("%d IK errors", stats.ikErrors)
	}
}

func TestCadExportOfTwistedJoints(t *testing.T) {
	if _, err := NewCadAssembly(twistedHexapod(JointTwists{Coxa: 10}), "twisted"); err == nil || !strings.Contains(err.Error(), "coxa twists") {
		t.Errorf("error = %v, want coxa twists refused", err)
	}

	// Femur and tibia twists are exported: the joint frames carry them
	a, err := NewCadAssembly(twistedHexapod(JointTwists{Femur: 15, Tibia: -10}), "twisted")
	if err != nil {
		t.Fatal(err)
	}
	p := NewPod(twistedHexapod(JointTwists{Femur: 15, Tibia: -10}))
	frames := p.Legs[0].ServoFrames()
	for j, joint := range a.Legs[0].Joints {
		// The CAD frame's Z axis is the joint axis, with Y and Z flipped (CAD is Z up)
		want := [3]float64{frames[j].At(0, 2), -frames[j].At(1, 2), -frames[j].At(2, 2)}
		got := [3]float64{joint.Frame[0][2], joint.Frame[1][2], joint.Frame[2][2]}
		for k := range want {
			if math.Abs(got[k]-want[k]) > 1e-9 {
				t.Fatalf("joint %s: axis %v, want %v", joint.Name, got, want)
			}
		}
	}
}

// BenchmarkIK compares the closed form solution (untwisted legs) with the numeric one (twisted legs), for the
// small foot movements the gait engine asks for every tick
func BenchmarkIK(b *testing.B) {
	for _, bench := range []struct {
		name   string
		twists JointTwists
	}{{"closed form", JointTwists{}}, {"numeric", JointTwists{Coxa: 10, Femur: 20, Tibia: -10}}} {
		b.Run(bench.name, func(b *testing.B) {
			p := NewPod(twistedHexapod(bench.twists))
			leg := p.Legs[0]
			n := leg.NeutralEffectorCoordinate
			targets := make([]Coordinate, 64)
			for i := range targets {
				a := 2 * math.Pi * float64(i) / float64(len(targets))
				targets[i] = NewCoordinate(n.X+30*math.Cos(a), n.Y+30*math.Sin(a), n.Z-10*math.Sin(a))
			}
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				angles, err := SolveEffectorIK(leg, targets[i%len(targets)], nil)
				if err != nil {
					b.Fatal(err)
				}
				leg.ServoAngles = angles
			}
		})
	}
}

func TestExampleInsect(t *testing.T) {
	b := NewExampleInsect()
	if !b.HasTwists() || b.Design == nil || b.Stance == nil {
		t.Fatalf("twists %v, design %v, stance %v: want all three", b.HasTwists(), b.Design != nil, b.Stance != nil)
	}
	p := NewPod(b)
	checkStance(t, p, 55, []float64{95})

	// The front feet reach forward and the rear feet back, symmetrically
	front, rear := p.Legs[1].NeutralEffectorCoordinate, p.Legs[5].NeutralEffectorCoordinate
	if front.Y < 140 || math.Abs(front.Y+rear.Y) > 1e-6 {
		t.Errorf("front foot at y %.1f, rear foot at y %.1f: want the front foot well forward and the rear foot mirrored back", front.Y, rear.Y)
	}

	stats := runScenario(t, b, []step{
		{3, Twist{Y: 80}, TRIPOD},
		{3, Twist{X: -40, Yaw: 25}, RIPPLE},
		{4, Twist{X: 40, Y: -40}, WAVE},
	})
	if stats.unstableTicks > 0 || stats.ticksToSettle >= 2000 {
		t.Errorf("%+v", stats)
	}
}
