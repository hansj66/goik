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
	"path/filepath"
	"strings"
	"testing"
)

// checkStance checks that every foot is at the stance's height, below a knee that is above it, and (if reach
// is not 0) the given horizontal distance from the femur joint. With reach 0 the tibia must be vertical.
func checkStance(t *testing.T, p *Pod, height float64, reach []float64) {
	t.Helper()
	for l, leg := range p.Legs {
		femur, knee, foot := leg.Joints[FEMUR_ORIGIN_INDEX], leg.Joints[TIBIA_ORIGIN_INDEX], leg.Joints[EFFECTOR_ORIGIN_INDEX]
		if math.Abs(foot.Z-height) > 1e-6 {
			t.Errorf("leg %d: foot at Z = %.4f, want %.4f", l, foot.Z, height)
		}
		if knee.Z >= foot.Z {
			t.Errorf("leg %d: knee (Z = %.2f) is not above the foot (Z = %.2f)", l, knee.Z, foot.Z)
		}
		r := reach[l%len(reach)]
		if r == 0 {
			if d := math.Hypot(knee.X-foot.X, knee.Y-foot.Y); d > 1e-6 {
				t.Errorf("leg %d: tibia not vertical (the foot is %.4f mm from below the knee)", l, d)
			}
		} else if d := math.Hypot(foot.X-femur.X, foot.Y-femur.Y); math.Abs(d-r) > 1e-6 {
			t.Errorf("leg %d: foot %.4f mm out from the femur joint, want %.4f", l, d, r)
		}
	}
}

func TestStanceWithVerticalTibias(t *testing.T) {
	p := NewPod(NewExampleHexapodAX12())
	if err := p.SetStance(&Stance{Height: 110}); err != nil {
		t.Fatal(err)
	}
	checkStance(t, p, 110, []float64{0})

	// The coxa rest angles are untouched
	for l, a := range p.BodyDefinition.RestAngles {
		if a.Coxa != 0 {
			t.Errorf("leg %d: coxa rest angle %.2f, want 0", l, a.Coxa)
		}
	}
}

func TestStanceWithReach(t *testing.T) {
	p := NewPod(NewExampleHexapodAX12())
	if err := p.SetStance(&Stance{Height: 90, Reach: 120}); err != nil {
		t.Fatal(err)
	}
	checkStance(t, p, 90, []float64{120})

	// Per leg: legs 0 and 3 stand further out
	if err := p.SetStance(&Stance{Height: 90, Reach: 120, LegReach: []float64{140, 0, 0, 140, 0, 0}}); err != nil {
		t.Fatal(err)
	}
	checkStance(t, p, 90, []float64{140, 120, 120, 140, 120, 120})
}

func TestStanceFollowsLegChanges(t *testing.T) {
	p := NewPod(NewExampleHexapodAX12())
	p.SetStance(&Stance{Height: 110})
	p.SetCoxaAngle(1, 20)
	if err := p.SetTibiaLength(1, 140); err != nil {
		t.Fatal(err)
	}
	checkStance(t, p, 110, []float64{0})
	if a := p.BodyDefinition.RestAngles[1].Coxa; a != 20 {
		t.Errorf("leg 1: coxa rest angle %.2f, want 20", a)
	}

	if err := p.SetFemurAngle(2, 10); err == nil || !strings.Contains(err.Error(), "stance") {
		t.Errorf("error = %v, want one about the stance", err)
	}

	// Too short a tibia for the height: rejected, and the pod is unchanged
	if err := p.SetTibiaLength(2, 20); err == nil {
		t.Errorf("a 20 mm tibia was accepted at a height of 110 mm")
	}
	if l := p.BodyDefinition.Segments[2].Tibia; l != 130 {
		t.Errorf("leg 2: tibia %.2f mm after a failed change, want 130", l)
	}

	// Without a stance the angles can be set by hand again
	p.SetStance(nil)
	if err := p.SetFemurAngle(2, 10); err != nil {
		t.Error(err)
	}
}

func TestStanceValidation(t *testing.T) {
	tests := []struct {
		name   string
		stance Stance
		want   string
	}{
		{"too low", Stance{Height: 5}, "at least 10 mm above the ground"},
		{"tibia can't be vertical", Stance{Height: 230}, "can't stand vertical"},
		{"out of reach", Stance{Height: 150, Reach: 200}, "only reaches"},
		{"too close", Stance{Height: 40, Reach: 20}, "can't fold closer"},
		{"no room to step", Stance{Height: 80}, "can only step 5 mm"},
		{"tibia out of range", Stance{Height: 60}, "tibia angle (159.0 degrees) is outside the AX-12A's range"},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			p := NewPod(NewExampleHexapodAX12())
			before := p.BodyDefinition.RestAngles[0]
			err := p.SetStance(&tt.stance)
			if err == nil || !strings.Contains(err.Error(), tt.want) {
				t.Fatalf("error = %v, want one containing '%s'", err, tt.want)
			}
			if p.BodyDefinition.Stance != nil || p.BodyDefinition.RestAngles[0] != before {
				t.Errorf("the pod changed")
			}
		})
	}
}

func TestStanceHeightRange(t *testing.T) {
	b := NewExampleHexapodAX12()
	b.Stance = &Stance{}
	lowest, highest, ok := StanceHeightRange(b)
	if !ok {
		t.Fatal("no height works")
	}
	t.Logf("AX-12A pod, tibia vertical: %.0f to %.0f mm", lowest, highest)
	// The tibia (130 mm) can stand vertical from 55 to 205 mm with a 75 mm femur
	if lowest < 55 || highest > 205 || lowest > 110 || highest < 110 {
		t.Errorf("possible heights %.0f to %.0f mm, want a range inside 55 to 205 mm that includes 110", lowest, highest)
	}

	p := NewPod(NewExampleHexapodAX12())
	err := p.SetStance(&Stance{Height: 230})
	if err == nil || !strings.Contains(err.Error(), "Possible heights with this reach") {
		t.Errorf("error = %v, want the possible heights", err)
	}
}

func TestStanceWithDesign(t *testing.T) {
	d, _ := NewRoundDesign(6, 80, SegmentLengths{Coxa: 60, Femur: 75, Tibia: 130}, ServoAngles{Femur: -15, Tibia: 97})
	b, _ := d.BodyDefinition(nil)
	p := NewPod(b)
	if err := p.SetStance(&Stance{Height: 100}); err != nil {
		t.Fatal(err)
	}

	// Mirror images change together, and the stance keeps the height
	if err := p.SetTibiaLength(1, 140); err != nil {
		t.Fatal(err)
	}
	checkStance(t, p, 100, []float64{0})
	if r := p.BodyDefinition.RestAngles; r[1] != r[2] {
		t.Errorf("legs 1 and 2: rest angles %+v and %+v, want the same", r[1], r[2])
	}
	if err := p.SetFemurAngle(1, 0); err == nil {
		t.Errorf("a femur angle was accepted with a stance")
	}

	// A new design keeps the stance
	rect, _ := NewRectangularDesign(4, 210, 110, SegmentLengths{Coxa: 50, Femur: 70, Tibia: 120}, ServoAngles{})
	if err := p.ApplyDesign(rect); err != nil {
		t.Fatal(err)
	}
	if p.BodyDefinition.NumLegs != 8 || p.BodyDefinition.Stance == nil {
		t.Fatalf("%d legs, stance %v: want 8 legs with the stance", p.BodyDefinition.NumLegs, p.BodyDefinition.Stance)
	}
	checkStance(t, p, 100, []float64{0})

	// The design holds the computed rest angles, so it rebuilds the same pod
	again, err := p.BodyDefinition.Design.BodyDefinition(nil)
	if err != nil {
		t.Fatal(err)
	}
	sameGeometry(t, again, p.BodyDefinition, 1e-9)
}

func TestStanceIsSavedAndLoaded(t *testing.T) {
	p := NewPod(NewExampleHexapodSTS3215())
	if err := p.SetStance(&Stance{Height: 95, Reach: 90}); err != nil {
		t.Fatal(err)
	}
	file := filepath.Join(t.TempDir(), "sts")
	if err := p.BodyDefinition.Save(file); err != nil {
		t.Fatal(err)
	}
	loaded, err := p.BodyDefinition.Load(file)
	if err != nil {
		t.Fatal(err)
	}
	if loaded.Stance == nil || loaded.Stance.Height != 95 || loaded.Stance.Reach != 90 {
		t.Fatalf("stance = %+v, want height 95 and reach 90", loaded.Stance)
	}
	checkStance(t, NewPod(loaded), 95, []float64{90})
}

func TestStancedPodWalks(t *testing.T) {
	p := NewPod(NewExampleHexapodAX12())
	if err := p.SetStance(&Stance{Height: 100}); err != nil {
		t.Fatal(err)
	}
	stats := runScenario(t, p.BodyDefinition, []step{
		{3, Twist{Y: 60}, TRIPOD},
		{3, Twist{Yaw: 20}, RIPPLE},
		{4, Twist{X: 30}, WAVE},
	})
	if stats.unstableTicks > 0 {
		t.Errorf("centre of gravity was outside the support polygon for %d ticks", stats.unstableTicks)
	}
	if stats.maxNeutralOffset > SETTLE_TOLERANCE {
		t.Errorf("feet settled %.2f mm from the neutral stance", stats.maxNeutralOffset)
	}
}
