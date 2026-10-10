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
	"testing"
)

func walkingCentipede(t *testing.T) (*Pod, *GaitEngine) {
	t.Helper()
	p := NewPod(NewExampleCentipede())
	p.SetDebugChannel(make(chan string, 1000))
	e, err := NewGaitEngine(p)
	if err != nil {
		t.Fatal(err)
	}
	p.Engine = e
	return p, e
}

func TestSegmentChainFollowsTheHead(t *testing.T) {
	body := SegmentedBody{Count: 6, Spacing: 30, Length: 26, Width: 22, MaxJointAngle: 30}
	ch := newSegmentChain(body, SegmentPose{})

	// Straight at first: the segments are lined up behind the head
	for k, p := range ch.poses {
		if math.Abs(p.X) > 1e-9 || math.Abs(p.Y+float64(k)*30) > 1e-9 || math.Abs(p.Heading) > 1e-9 {
			t.Fatalf("segment %d at %+v, want straight behind the head", k, p)
		}
	}

	// The head walks a circle of radius 200: every segment ends up on it, Spacing mm (along the path) apart
	const radius = 200.0
	var head SegmentPose
	for a := 0.0; a <= 2.0; a += 0.002 {
		head = SegmentPose{X: radius*math.Cos(a) - radius, Y: radius * math.Sin(a), Heading: a}
		ch.moveHead(head)
	}
	for k, p := range ch.poses[1:] {
		if d := math.Hypot(p.X+radius, p.Y); math.Abs(d-radius) > 0.5 {
			t.Errorf("segment %d is %.2f mm from the circle's centre, want %.0f", k+1, d, radius)
		}
	}
	for k, a := range ch.jointAngles() {
		// Neighbours 30 mm apart on a 200 mm circle: 30 / 200 radians
		if math.Abs(a-30.0/radius*180/math.Pi) > 0.3 {
			t.Errorf("joint %d at %.2f degrees, want %.2f", k, a, 30.0/radius*180/math.Pi)
		}
	}
}

func TestMetachronalGait(t *testing.T) {
	b := NewExampleCentipede()
	pg, err := NewPhaseGait(b.Gait, b.NumLegs)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(pg.DutyFactor-METACHRONAL_DUTY_FACTOR) > 1e-9 {
		t.Errorf("duty factor %.2f, want %.2f", pg.DutyFactor, METACHRONAL_DUTY_FACTOR)
	}
	for l := 0; l < b.NumLegs; l += 2 {
		// The pair of a segment is half a cycle apart
		if d := math.Abs(wrapPhaseError(pg.Offsets[l] - pg.Offsets[l+1])); math.Abs(d-0.5) > 1e-9 {
			t.Errorf("legs %d and %d are %.3f apart, want 0.5", l, l+1, d)
		}
		// The wave runs from the head to the tail: each segment lifts 1/8 cycle after the one in front of it
		if l >= 2 {
			if d := wrapPhaseError(pg.Offsets[l-2] - pg.Offsets[l]); math.Abs(d-1.0/METACHRONAL_WAVELENGTH) > 1e-9 {
				t.Errorf("segments %d and %d are %.3f apart, want %.3f", l/2-1, l/2, d, 1.0/METACHRONAL_WAVELENGTH)
			}
		}
	}

	// One piece pods can walk with it too
	for _, body := range []*BodyDefinition{NewExampleHexapodAX12(), NewExampleInsect()} {
		g, err := NewGaitFor(body, METACHRONAL)
		if err != nil {
			t.Fatal(err)
		}
		if _, err := NewPhaseGait(g, body.NumLegs); err != nil {
			t.Error(err)
		}
	}
	// A segmented body walks with the metachronal gait only
	if _, err := NewGaitFor(b, TRIPOD); err == nil {
		t.Errorf("a segmented body accepted tripod gait")
	}
}

func TestFixedCoxaIK(t *testing.T) {
	p := NewPod(NewExampleCentipede())
	leg := p.Legs[0]
	n := leg.NeutralEffectorCoordinate

	// Forwards and up, in the leg's plane: reached exactly, without turning the coxa
	target := NewCoordinate(n.X, n.Y+20, n.Z)
	a, err := SolveEffectorIK(leg, target, nil)
	if err != nil {
		t.Fatal(err)
	}
	leg.RecalculateForwardKinematics(a)
	if d := distance(leg.Joints[EFFECTOR_ORIGIN_INDEX], target); d > 0.01 || a.Coxa != 0 {
		t.Errorf("foot %.3f mm off, coxa %.2f: want on target with the coxa fixed", d, a.Coxa)
	}

	// Slightly sideways: as close as the leg's plane allows. Far sideways: out of reach
	if _, err := SolveEffectorIK(leg, NewCoordinate(n.X+5, n.Y, n.Z), nil); err != nil {
		t.Errorf("5 mm out of the leg's plane: %v", err)
	}
	if _, err := SolveEffectorIK(leg, NewCoordinate(n.X+40, n.Y, n.Z), nil); err == nil {
		t.Errorf("40 mm out of the leg's plane was reached")
	}
}

func TestCentipedeWalksWithItsFeetOnTheGround(t *testing.T) {
	p, e := walkingCentipede(t)
	e.SetTwist(Twist{Y: 40})

	// A grounded foot stays where it is in the world
	world := func(leg int) Coordinate {
		pose := e.chain.poses[p.BodyDefinition.LegSegment(leg)]
		return pose.ToParent(e.Legs[leg].Foot)
	}
	const leg = 20
	var planted *Coordinate
	worst := 0.0
	for i := 0; i < 500; i++ {
		p.Update()
		if e.Legs[leg].Swinging {
			planted = nil
			continue
		}
		w := world(leg)
		if planted != nil {
			worst = math.Max(worst, math.Hypot(w.X-planted.X, w.Y-planted.Y))
		} else {
			planted = &w
		}
	}
	if worst > 1e-6 {
		t.Errorf("a grounded foot moved %.6f mm in the world", worst)
	}
	if _, y, _ := e.Odometry(); y < 350 {
		t.Errorf("walked %.0f mm in 10 s at 40 mm/s", y)
	}
	if e.IKErrors > 0 {
		t.Errorf("%d IK errors: %v", e.IKErrors, e.LastError)
	}

	// Halting settles the feet
	e.SetTwist(Twist{})
	for i := 0; i < 2000 && !e.IsIdle(); i++ {
		p.Update()
	}
	if !e.IsIdle() {
		t.Errorf("did not settle after halting")
	}
}

func TestCentipedeTurnsWithinItsJoints(t *testing.T) {
	p, e := walkingCentipede(t)
	limit := p.BodyDefinition.Body.MaxJointAngle

	// Walking in a tight curve
	e.SetTwist(Twist{Y: 40, Yaw: 30})
	worst := 0.0
	for i := 0; i < 1000; i++ {
		p.Update()
		for _, a := range e.JointAngles() {
			worst = math.Max(worst, math.Abs(a))
		}
	}
	if worst > limit+1 {
		t.Errorf("a joint bent %.1f degrees, the limit is %.0f", worst, limit)
	}
	if e.IKErrors > 0 {
		t.Errorf("%d IK errors: %v", e.IKErrors, e.LastError)
	}

	// Turning on the spot: only the first joint bends, up to its limit
	p, e = walkingCentipede(t)
	e.SetTwist(Twist{Yaw: 20})
	for i := 0; i < 500; i++ {
		p.Update()
	}
	angles := e.JointAngles()
	if math.Abs(math.Abs(angles[0])-limit) > 1 || math.Abs(angles[1]) > 1e-6 {
		t.Errorf("joints %.1f and %.1f after turning on the spot, want %.0f and 0", angles[0], angles[1], limit)
	}
}

func TestSegmentedBodyLimits(t *testing.T) {
	p, e := walkingCentipede(t)

	// No sideways walking, no body pose, no stance, no CAD export
	e.SetTwist(Twist{X: 50, Y: 20})
	if e.target.X != 0 {
		t.Errorf("sideways velocity %.0f accepted", e.target.X)
	}
	if err := e.SetBodyPose(BodyPose{Pitch: 10}); err == nil {
		t.Errorf("a body pose was accepted")
	}
	if err := p.SetStance(&Stance{Height: 40}); err == nil {
		t.Errorf("a stance was accepted")
	}
	if _, err := NewCadAssembly(p.BodyDefinition, "centipede"); err == nil {
		t.Errorf("the CAD export accepted a segmented body")
	}
}

func TestCentipedeIsSavedAndLoaded(t *testing.T) {
	b := NewExampleCentipede()
	file := filepath.Join(t.TempDir(), "centipede")
	if err := b.Save(file); err != nil {
		t.Fatal(err)
	}
	loaded, err := b.Load(file)
	if err != nil {
		t.Fatal(err)
	}
	if loaded.Body == nil || *loaded.Body != *b.Body || len(loaded.LegSegments) != b.NumLegs || !loaded.HasFixedCoxa(5) {
		t.Fatalf("body %+v, %d leg segments: the segmented body was not saved", loaded.Body, len(loaded.LegSegments))
	}
	sameGeometry(t, loaded, b, 1e-9)
}

func TestSegmentChainBacksUp(t *testing.T) {
	body := SegmentedBody{Count: 6, Spacing: 30, Length: 26, Width: 22, MaxJointAngle: 30}
	ch := newSegmentChain(body, SegmentPose{})

	// Forward in a curve, then straight back the same way: the body retraces the path
	var poses []SegmentPose
	for a := 0.0; a < 1.0; a += 0.01 {
		ch.moveHead(SegmentPose{X: 100*math.Cos(a) - 100, Y: 100 * math.Sin(a), Heading: a})
		poses = append(poses, ch.head)
	}
	for i := len(poses) - 1; i >= 0; i-- {
		ch.moveHead(poses[i])
	}
	ch.moveHead(SegmentPose{})
	for k, p := range ch.poses {
		if math.Abs(p.X) > 0.5 || math.Abs(p.Y+float64(k)*30) > 0.5 {
			t.Errorf("segment %d at %+v after backing up, want straight behind the head again", k, p)
		}
	}
	// Further back than the body has walked: along the straight line it started on
	for y := 0.0; y >= -60; y -= 1 {
		ch.moveHead(SegmentPose{Y: y})
	}
	if tail := ch.poses[5]; math.Abs(tail.X) > 0.5 || math.Abs(tail.Y-(-60-150)) > 0.5 {
		t.Errorf("tail at %+v, want (0, -210)", tail)
	}
}
