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

func TestBodyPoseTransforms(t *testing.T) {
	pose := BodyPose{Pitch: 10, Roll: -5, Yaw: 20, X: 3, Y: -4, Z: -15}
	p := NewCoordinate(100, 50, 80)
	if back := pose.ToBody(pose.ToGround(p)); distance(back, p) > 1e-9 {
		t.Errorf("ToBody(ToGround(p)) = %v, want %v", back, p)
	}

	// The ground is at +Z, so "raised" means a smaller Z
	front := BodyPose{Pitch: 10}.ToGround(NewCoordinate(0, 100, 0))
	if front.Z >= 0 {
		t.Errorf("positive pitch should raise the front (+Y) of the body, got z = %f", front.Z)
	}
	side := BodyPose{Roll: 10}.ToGround(NewCoordinate(100, 0, 0))
	if side.Z <= 0 {
		t.Errorf("positive roll should lower the +X side of the body, got z = %f", side.Z)
	}
	turned := BodyPose{Yaw: 90}.ToGround(NewCoordinate(100, 0, 0))
	if math.Abs(turned.X) > 1e-9 || math.Abs(turned.Y-100) > 1e-9 {
		t.Errorf("positive yaw should turn +X towards +Y, got %v", turned)
	}
	up := BodyPose{Z: -20}.ToGround(Coordinate{})
	if up.Z != -20 {
		t.Errorf("negative Z should raise the body, got %v", up)
	}
}

// settle ticks the pod until the engine is idle
func settle(t *testing.T, p *Pod) {
	t.Helper()
	for i := 0; i < 2000; i++ {
		p.Update()
		if p.Engine.IsIdle() {
			return
		}
	}
	t.Fatal("the engine did not settle")
}

func TestBodyPoseWhileStanding(t *testing.T) {
	p := NewPod(NewExampleHexapod1())
	e, _ := NewGaitEngine(p)
	p.Engine = e
	feet := make([]Coordinate, len(e.Legs))
	for i, l := range e.Legs {
		feet[i] = l.Foot
	}

	if err := e.SetBodyPose(BodyPose{Pitch: 10, Z: -15}); err != nil {
		t.Fatal(err)
	}
	p.Update()
	if e.IsIdle() {
		t.Fatal("the engine should not be idle while the body pose changes")
	}
	settle(t, p)

	if e.BodyPose() != e.TargetBodyPose() || e.PoseLimited {
		t.Errorf("pose = %v, want %v", e.BodyPose(), e.TargetBodyPose())
	}
	for i, l := range e.Legs {
		if distance(l.Foot, feet[i]) > 1e-9 || l.Swinging {
			t.Errorf("leg %d moved its foot while the body was posing", i)
		}
	}
	// Leg 1 is mounted at the front (+Y) of the body, leg 4 at the back
	joints := p.GroundJoints()
	if front, back := joints[1][COXA_ORIGIN_INDEX].Z, joints[4][COXA_ORIGIN_INDEX].Z; front >= back {
		t.Errorf("front coxa (z = %f) should be higher than the back coxa (z = %f)", front, back)
	}
	// The feet seen from the ground frame stay on the ground
	for i := range joints {
		if d := distance(joints[i][EFFECTOR_ORIGIN_INDEX], feet[i]); d > 1e-6 {
			t.Errorf("leg %d: foot is %f mm from where it was standing", i, d)
		}
	}
	if e.IKErrors > 0 {
		t.Errorf("%d IK errors", e.IKErrors)
	}
}

func TestBodyPoseLimitedToReach(t *testing.T) {
	p := NewPod(NewExampleHexapod2())
	e, _ := NewGaitEngine(p)
	p.Engine = e

	// Far too low for the small hexapod
	if err := e.SetBodyPose(BodyPose{Z: 100}); err != nil {
		t.Fatal(err)
	}
	settle(t, p)

	if !e.PoseLimited {
		t.Error("the pose should have been limited")
	}
	if z := e.BodyPose().Z; z <= 0 || z >= 100 {
		t.Errorf("the body should have moved down as far as possible, z = %f", z)
	}
	if e.IKErrors > 0 {
		t.Errorf("%d IK errors, last: %v", e.IKErrors, e.LastError)
	}
}

func TestWalkingWithBodyPose(t *testing.T) {
	p := NewPod(NewExampleHexapod1())
	e, _ := NewGaitEngine(p)
	p.Engine = e
	if err := e.SetBodyPose(BodyPose{Pitch: 5, Z: -10}); err != nil {
		t.Fatal(err)
	}
	e.SetTwist(Twist{Y: 60, Yaw: 10})

	start := e.Cycles()
	for i := 0; i < int(5/ENGINE_DT); i++ {
		p.Update()
		feet := groundedFeet(e)
		cog := e.centreOfGravity()
		for j := range feet {
			feet[j] = NewCoordinate(feet[j].X-cog.X, feet[j].Y-cog.Y, 0)
		}
		if SupportMargin(feet) < 0 {
			t.Fatalf("tick %d: the centre of gravity is outside the support polygon", i)
		}
	}

	// Tripod: 0.4 s swing + 0.4 s stance
	if cycles := e.Cycles() - start; math.Abs(cycles-5/e.CycleTime()) > 0.01 {
		t.Errorf("completed %f cycles in 5 s, want %f", cycles, 5/e.CycleTime())
	}
	if e.IKErrors > 0 || e.PoseLimited {
		t.Errorf("%d IK errors (last: %v), pose limited: %t", e.IKErrors, e.LastError, e.PoseLimited)
	}
}

func TestSetBodyPoseRejectsExtremes(t *testing.T) {
	e, _ := NewGaitEngine(NewPod(NewExampleHexapod1()))
	if err := e.SetBodyPose(BodyPose{Roll: 60}); err == nil {
		t.Error("expected an error for a 60 degree roll")
	}
	if err := e.SetBodyPose(BodyPose{Z: -150}); err == nil {
		t.Error("expected an error for a 150 mm height change")
	}
}
