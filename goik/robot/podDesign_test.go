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
	"reflect"
	"strings"
	"testing"
)

// sameGeometry compares the geometry of two body definitions
func sameGeometry(t *testing.T, got *BodyDefinition, want *BodyDefinition, tolerance float64) {
	t.Helper()
	if got.NumLegs != want.NumLegs {
		t.Fatalf("NumLegs = %d, want %d", got.NumLegs, want.NumLegs)
	}
	near := func(a, b float64) bool { return math.Abs(a-b) <= tolerance }
	for i := 0; i < want.NumLegs; i++ {
		g, w := got.CoxaCoordinates[i], want.CoxaCoordinates[i]
		if !near(g.X, w.X) || !near(g.Y, w.Y) || !near(g.Z, w.Z) {
			t.Errorf("leg %d: coxa at %v, want %v", i, g, w)
		}
		if !near(normalizeAngle(got.CoxaAngles[i]), normalizeAngle(want.CoxaAngles[i])) {
			t.Errorf("leg %d: coxa angle %.2f, want %.2f", i, got.CoxaAngles[i], want.CoxaAngles[i])
		}
		gs, ws := got.Segments[i], want.Segments[i]
		if !near(gs.Coxa, ws.Coxa) || !near(gs.Femur, ws.Femur) || !near(gs.Tibia, ws.Tibia) {
			t.Errorf("leg %d: segments %+v, want %+v", i, gs, ws)
		}
		gr, wr := got.RestAngles[i], want.RestAngles[i]
		if !near(gr.Coxa, wr.Coxa) || !near(gr.Femur, wr.Femur) || !near(gr.Tibia, wr.Tibia) {
			t.Errorf("leg %d: rest angles %+v, want %+v", i, gr, wr)
		}
	}
}

func TestRoundDesignBuildsExamplePod6(t *testing.T) {
	d, err := NewRoundDesign(6, 80, SegmentLengths{Coxa: 60, Femur: 75, Tibia: 130}, ServoAngles{Femur: -15, Tibia: 97})
	if err != nil {
		t.Fatal(err)
	}
	b, err := d.BodyDefinition(nil)
	if err != nil {
		t.Fatal(err)
	}

	want := NewExampleHexapodAX12()
	for _, field := range []struct {
		name      string
		got, want interface{}
	}{
		{"CoxaAngles", b.CoxaAngles, want.CoxaAngles},
		{"CoxaCoordinates", b.CoxaCoordinates, want.CoxaCoordinates},
		{"Segments", b.Segments, want.Segments},
		{"RestAngles", b.RestAngles, want.RestAngles},
	} {
		if !reflect.DeepEqual(field.got, field.want) {
			t.Errorf("%s = %v, want %v", field.name, field.got, field.want)
		}
	}
	if b.Gait.Name != "Tripod gait" {
		t.Errorf("gait = %s, want tripod", b.Gait.Name)
	}
	if len(d.Mounts) != 3 {
		t.Errorf("%d mounts, want 3 (one side of the pod)", len(d.Mounts))
	}
	want6 := []Point2{{0, 69.28}, {40, 69.28}, {80, 0}, {40, -69.28}, {0, -69.28}}
	if !reflect.DeepEqual(d.Outline, want6) {
		t.Errorf("outline = %v, want %v", d.Outline, want6)
	}
}

func TestDesignImportRoundTrip(t *testing.T) {
	bodies := map[string]func() *BodyDefinition{
		"hexapod0": NewExampleHexapod0,
		"hexapod1": NewExampleHexapod1,
		"hexapod2": NewExampleHexapod2,
		"ax12":     NewExampleHexapodAX12,
		"sts3215":  NewExampleHexapodSTS3215,
	}
	for name, body := range bodies {
		t.Run(name, func(t *testing.T) {
			b := body()
			d, err := NewDesignFromBodyDefinition(b)
			if err != nil {
				t.Fatal(err)
			}
			rebuilt, err := d.BodyDefinition(b)
			if err != nil {
				t.Fatal(err)
			}
			sameGeometry(t, rebuilt, b, 1e-9)
			if rebuilt.Servos != b.Servos {
				t.Errorf("the servo mapping was not kept")
			}
			if len(d.Outline) == 0 {
				t.Errorf("no outline")
			}
		})
	}
}

func TestDesignImportRejectsAsymmetricPods(t *testing.T) {
	// The pentapod and heptapod are symmetric front to back, not left to right
	for name, body := range map[string]func() *BodyDefinition{
		"pentapod": NewExamplePentapod,
		"heptapod": NewHeptapod,
		"spider":   NewSpider,
	} {
		if _, err := NewDesignFromBodyDefinition(body()); err == nil || !strings.Contains(err.Error(), "mirror image") {
			t.Errorf("%s: error = %v, want a missing mirror image", name, err)
		}
	}
}

func TestDesignMirrorsLegs(t *testing.T) {
	d, err := NewRectangularDesign(4, 180, 100, SegmentLengths{Coxa: 40, Femur: 60, Tibia: 100}, ServoAngles{Femur: -20, Tibia: 90})
	if err != nil {
		t.Fatal(err)
	}
	d.Mounts[0].Rest.Coxa = 30 // front leg on the +X side swept forward
	legs, err := d.Legs()
	if err != nil {
		t.Fatal(err)
	}
	if len(legs) != 8 {
		t.Fatalf("%d legs, want 8", len(legs))
	}

	// Numbered counter clockwise from +X
	for i := 1; i < len(legs); i++ {
		if positionAngle(legs[i].Position) < positionAngle(legs[i-1].Position) {
			t.Errorf("leg %d at %v comes before leg %d at %v", i, legs[i].Position, i-1, legs[i-1].Position)
		}
	}

	for i, l := range legs {
		m := d.Mirror(i)
		if m == -1 {
			t.Fatalf("leg %d has no mirror image", i)
		}
		o := legs[m]
		if o.Position.X != -l.Position.X || o.Position.Y != l.Position.Y {
			t.Errorf("legs %d and %d: %v and %v are not mirror images", i, m, l.Position, o.Position)
		}
		if normalizeAngle(o.Angle) != normalizeAngle(180-l.Angle) {
			t.Errorf("legs %d and %d: angles %.2f and %.2f are not mirror images", i, m, l.Angle, o.Angle)
		}
		if o.Rest.Coxa != -l.Rest.Coxa || o.Rest.Femur != l.Rest.Femur || o.Rest.Tibia != l.Rest.Tibia {
			t.Errorf("legs %d and %d: rest angles %+v and %+v are not mirror images", i, m, l.Rest, o.Rest)
		}
	}
}

func TestOddRoundDesignHasALegOnTheAxis(t *testing.T) {
	d, err := NewRoundDesign(5, 50, SegmentLengths{Coxa: 40, Femur: 60, Tibia: 120}, ServoAngles{Femur: -50, Tibia: 100})
	if err != nil {
		t.Fatal(err)
	}
	b, err := d.BodyDefinition(nil)
	if err != nil {
		t.Fatal(err)
	}
	if b.NumLegs != 5 || b.Gait.Name != "Wave gait" {
		t.Fatalf("%d legs with %s, want 5 legs with wave gait", b.NumLegs, b.Gait.Name)
	}
	if c := b.CoxaCoordinates[1]; c.X != 0 || c.Y != 50 || b.CoxaAngles[1] != 90 {
		t.Errorf("leg 1 at %v pointing at %.2f, want a front leg at (0, 50) pointing forward", c, b.CoxaAngles[1])
	}
}

func TestPodSettersChangeTheMirrorImage(t *testing.T) {
	d, _ := NewRoundDesign(6, 80, SegmentLengths{Coxa: 60, Femur: 75, Tibia: 130}, ServoAngles{Femur: -15, Tibia: 97})
	b, _ := d.BodyDefinition(nil)
	p := NewPod(b)

	// Leg 1 (front, +X) and leg 2 (front, -X) are mirror images
	if err := p.SetCoxaAngle(1, 20); err != nil {
		t.Fatal(err)
	}
	if err := p.SetTibiaLength(2, 140); err != nil {
		t.Fatal(err)
	}
	r := p.BodyDefinition.RestAngles
	s := p.BodyDefinition.Segments
	if r[1].Coxa != 20 || r[2].Coxa != -20 {
		t.Errorf("coxa rest angles %.2f and %.2f, want 20 and -20", r[1].Coxa, r[2].Coxa)
	}
	if s[1].Tibia != 140 || s[2].Tibia != 140 || s[0].Tibia != 130 {
		t.Errorf("tibia lengths %.2f, %.2f and %.2f, want 140, 140 and 130", s[1].Tibia, s[2].Tibia, s[0].Tibia)
	}
	if p.BodyDefinition.Design == nil {
		t.Errorf("the pod lost its design")
	}

	// A pod without a design changes one leg only
	p = NewPod(NewExampleHexapodAX12())
	p.SetCoxaAngle(1, 20)
	if r := p.BodyDefinition.RestAngles; r[1].Coxa != 20 || r[2].Coxa != 0 {
		t.Errorf("coxa rest angles %.2f and %.2f, want 20 and 0", r[1].Coxa, r[2].Coxa)
	}
}

func TestLegOnTheAxisKeepsCoxaAngleZero(t *testing.T) {
	d, _ := NewRoundDesign(4, 60, SegmentLengths{Coxa: 40, Femur: 60, Tibia: 100}, ServoAngles{Femur: -20, Tibia: 90})
	b, _ := d.BodyDefinition(nil)
	p := NewPod(b)
	// Leg 1 points forward from (0, 60)
	if err := p.SetCoxaAngle(1, 10); err == nil {
		t.Errorf("a leg on the symmetry axis accepted a coxa rest angle")
	}
}

func TestDesignValidation(t *testing.T) {
	leg := SegmentLengths{Coxa: 40, Femur: 60, Tibia: 100}
	mounts := []LegMount{{X: 50, Y: 50, Segments: leg}, {X: 50, Y: -50, Segments: leg}}
	tests := []struct {
		name   string
		design PodDesign
		want   string
	}{
		{"-X side", PodDesign{Mounts: []LegMount{{X: -50, Y: 0, Segments: leg}, {X: 50, Y: 50, Segments: leg}}}, "-X side"},
		{"sideways on the axis", PodDesign{Mounts: append([]LegMount{{X: 0, Y: 80, Angle: 0, Segments: leg}}, mounts...)}, "forward (90) or backward"},
		{"coxa angle on the axis", PodDesign{Mounts: append([]LegMount{{X: 0, Y: 80, Angle: 90, Segments: leg, Rest: ServoAngles{Coxa: 10}}}, mounts...)}, "coxa rest angle must be 0"},
		{"too few legs", PodDesign{Mounts: []LegMount{{X: 50, Y: 0, Segments: leg}}}, "at least 3 legs"},
		{"too close", PodDesign{Mounts: append([]LegMount{{X: 0.4, Y: 80, Angle: 90, Segments: leg}}, mounts...)}, "less than 1 mm apart"},
		{"outline off the axis", PodDesign{Outline: []Point2{{10, 50}, {50, 0}, {0, -50}}, Mounts: mounts}, "start and end on the symmetry axis"},
		{"outline crossing itself", PodDesign{Outline: []Point2{{0, 50}, {50, -20}, {50, 20}, {0, -50}}, Mounts: mounts}, "crosses itself"},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if _, err := tt.design.Legs(); err == nil || !strings.Contains(err.Error(), tt.want) {
				t.Errorf("error = %v, want one containing '%s'", err, tt.want)
			}
		})
	}
}

func TestDesignedPodCanBeSavedAndLoaded(t *testing.T) {
	d, _ := NewRectangularDesign(4, 180, 100, SegmentLengths{Coxa: 40, Femur: 60, Tibia: 100}, ServoAngles{Femur: -20, Tibia: 90})
	b, _ := d.BodyDefinition(nil)
	file := filepath.Join(t.TempDir(), "octopod")
	if err := b.Save(file); err != nil {
		t.Fatal(err)
	}
	loaded, err := b.Load(file)
	if err != nil {
		t.Fatal(err)
	}
	if loaded.Design == nil {
		t.Fatal("the design was not saved")
	}
	sameGeometry(t, loaded, b, 1e-9)
	if !reflect.DeepEqual(loaded.Design, b.Design) {
		t.Errorf("design = %+v, want %+v", loaded.Design, b.Design)
	}
}

func TestDesignedOctopodWalks(t *testing.T) {
	d, err := NewRectangularDesign(4, 210, 110, SegmentLengths{Coxa: 50, Femur: 70, Tibia: 120}, ServoAngles{Femur: -15, Tibia: 95})
	if err != nil {
		t.Fatal(err)
	}
	b, err := d.BodyDefinition(nil)
	if err != nil {
		t.Fatal(err)
	}
	stats := runScenario(t, b, []step{
		{3, Twist{Y: 60}, TRIPOD},
		{3, Twist{Yaw: 20}, TRIPOD},
		{6, Twist{X: 30}, WAVE},
	})
	t.Logf("%+v", stats)
	if stats.unstableTicks > 0 {
		t.Errorf("centre of gravity was outside the support polygon for %d ticks", stats.unstableTicks)
	}
	if stats.ticksToSettle >= 2000 {
		t.Errorf("pod did not settle after the halt command")
	}
	if stats.maxNeutralOffset > SETTLE_TOLERANCE {
		t.Errorf("feet settled %.2f mm from the neutral stance", stats.maxNeutralOffset)
	}
}
