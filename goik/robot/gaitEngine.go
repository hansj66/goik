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

/*
	Notes regarding the gait engine

	The table based gait in pod.go precomputes one stride and plays it back. Changing the stride,
	the direction or the gait means rebuilding the tables, which makes the feet jump.

	The gait engine instead keeps the *current foot positions* as its state and only lets commands
	change rates:
		- A body velocity (Twist) moves the stance feet backwards relative to the body.
		- Each leg has a phase in [0, 1). The leg is in stance while phase < duty factor and swings
		  for the rest of the cycle. Swing feet are steered towards a landing point that centres the
		  next stance on the leg's neutral position.
		- A new velocity is approached with an acceleration limit.
		- A new gait is approached by pulling each leg's phase towards its new offset (a small
		  phase locked loop) and by blending the duty factor.
		- The swing time is fixed (servo speed is what limits a swing), so the cycle time follows
		  from the duty factor. Gaits with a high duty factor (wave) are therefore slower.
		- A leg that is due to lift off waits until lifting it keeps the centre of gravity
		  (assumed to be at the body origin) inside the support polygon of the remaining feet.
		  This never triggers in a steady gait, but keeps the pod stable while gaits are blended.
		- While a leg waits, its foot is still dragged along by the body. If a grounded foot gets close
		  to the edge of the leg's reach, the body slows down (and stops if necessary) until it can step.
	Since the feet are only ever moved by small increments, there are no jumps when one motion is
	chained to the next. With a zero velocity, the swing targets are the neutral positions, so the pod
	walks itself back to the neutral stance and then goes idle.
*/

import (
	"fmt"
	"math"
	"sort"
)

// ENGINE_DT is the time (in seconds) the gait engine advances on each pod update
const ENGINE_DT = 0.02

// Feet closer than this (in mm) to their target are considered to be in place
const SETTLE_TOLERANCE = 0.5

// Fraction of a leg's reach used for the default maximum stride (leaves room for gait transitions)
const STRIDE_REACH_MARGIN = 0.7

// Fraction of a leg's reach where the body starts slowing down to keep a waiting foot reachable
const SLOWDOWN_REACH_FRACTION = 0.8

// How fast (fraction of full speed per second) the body speeds up again after slowing down for a foot
const SPEED_RECOVERY_RATE = 1.0

// Twist is a body velocity in the base reference frame.
// X and Y are in mm/s. Yaw is in degrees/s (positive yaw turns the body from +X towards +Y).
type Twist struct {
	X   float64
	Y   float64
	Yaw float64
}

func (t Twist) IsZero() bool {
	return t.X == 0 && t.Y == 0 && t.Yaw == 0
}

func (t Twist) String() string {
	return fmt.Sprintf("[%2.1f mm/s, %2.1f mm/s, %2.1f deg/s]", t.X, t.Y, t.Yaw)
}

// PhaseGait describes a gait by when each leg lifts off in the gait cycle,
// instead of as a table of steps.
type PhaseGait struct {
	Name string
	// Fraction of the gait cycle a leg spends in the stance phase
	DutyFactor float64
	// Offsets[i] is the phase of leg i when the gait cycle clock is at 0.
	// A leg is in stance for phase < DutyFactor and in swing for the rest of the cycle.
	Offsets []float64
}

// NewPhaseGait converts a gait pattern table into phase offsets.
// Every leg must swing exactly once per cycle and all legs must swing for the same number of columns.
func NewPhaseGait(g *Gait, numLegs int) (*PhaseGait, error) {
	if g == nil {
		return nil, fmt.Errorf("no gait defined")
	}

	n := g.NumIndicesInPattern
	pg := PhaseGait{Name: g.Name, Offsets: make([]float64, numLegs)}
	swingColumns := -1

	for leg := 0; leg < numLegs; leg++ {
		if leg >= len(*g.Pattern) || len((*g.Pattern)[leg]) < n {
			return nil, fmt.Errorf("%s has no pattern for leg %d", g.Name, leg)
		}
		row := (*g.Pattern)[leg]

		count := 0
		first := -1
		for c := 0; c < n; c++ {
			if row[c] != 1 {
				continue
			}
			count++
			// Start of a block of swing columns (blocks may wrap around the end of the pattern)
			if row[(c+n-1)%n] != 1 {
				if first != -1 {
					return nil, fmt.Errorf("leg %d swings more than once per cycle in %s", leg, g.Name)
				}
				first = c
			}
		}
		if count == 0 || count == n {
			return nil, fmt.Errorf("leg %d must have both a swing and a stance phase in %s", leg, g.Name)
		}
		if swingColumns != -1 && count != swingColumns {
			return nil, fmt.Errorf("all legs must swing for the same duration in %s", g.Name)
		}
		swingColumns = count

		// The leg lifts off (phase == duty factor) when the clock reaches column first/n
		duty := 1 - float64(count)/float64(n)
		pg.Offsets[leg] = wrapPhase(duty - float64(first)/float64(n))
	}
	pg.DutyFactor = 1 - float64(swingColumns)/float64(n)

	return &pg, nil
}

// EngineLeg is the gait engine's state for a single leg
type EngineLeg struct {
	// Position in the gait cycle [0, 1)
	Phase float64
	// True while the leg is in the swing phase
	Swinging bool
	// Commanded end effector position in the base reference frame
	Foot Coordinate

	liftPending   bool
	liftoff       Coordinate
	swingProgress float64
	swingDuration float64
}

// GaitEngine generates continuous leg motion from a body velocity and a gait,
// and allows both to be changed at any time.
type GaitEngine struct {
	pod  *Pod
	Legs []EngineLeg

	// Duration of the swing phase in seconds. The gait cycle time follows from this and the duty factor
	SwingTime float64
	// Maximum height of the swing arc in mm
	StepHeight float64
	// Maximum distance (mm) a foot may travel during one stance phase.
	// Faster velocities are scaled down to respect this.
	MaxStride float64
	// Acceleration limits used when changing velocity (mm/s^2 and degrees/s^2)
	LinearAcceleration float64
	YawAcceleration    float64
	// How hard leg phases are pulled towards the offsets of a new gait (per cycle)
	PhaseGain float64
	// Maximum relative change of a leg's phase rate while re-phasing (0.25 == +/- 25%)
	MaxPhaseCorrection float64
	// Maximum change of the duty factor per cycle during a gait transition
	DutyFactorRate float64
	// Minimum distance (mm) from the centre of gravity to the edge of the support polygon when lifting a leg
	StabilityMargin float64
	// Radius (mm) around the neutral foot positions every leg can reach
	Reach float64

	gait       *PhaseGait
	dutyFactor float64
	clock      float64
	target     Twist
	current    Twist
	idle       bool
	speedScale float64

	// Number of IK failures since the engine was created. The leg keeps its last valid pose on failure
	IKErrors  int
	LastError error
}

// NewGaitEngine creates a gait engine for the pod, starting from the pod's current pose and gait
func NewGaitEngine(p *Pod) (*GaitEngine, error) {
	gait, err := NewPhaseGait(p.BodyDefinition.Gait, p.BodyDefinition.NumLegs)
	if err != nil {
		return nil, err
	}

	e := &GaitEngine{
		pod:                p,
		Legs:               make([]EngineLeg, p.BodyDefinition.NumLegs),
		SwingTime:          0.4,
		StepHeight:         Z_LIFT,
		LinearAcceleration: 100,
		YawAcceleration:    60,
		PhaseGain:          1.0,
		MaxPhaseCorrection: 0.25,
		DutyFactorRate:     0.25,
		StabilityMargin:    10,
		gait:               gait,
		dutyFactor:         gait.DutyFactor,
		idle:               true,
		speedScale:         1,
	}

	for i, l := range p.Legs {
		e.Legs[i].Foot = l.Joints[EFFECTOR_ORIGIN_INDEX]
		e.Legs[i].Phase = gait.Offsets[i]
		e.Legs[i].liftPending = gait.Offsets[i] >= gait.DutyFactor
	}

	e.Reach = p.ReachRadius(e.StepHeight)
	e.MaxStride = 2 * STRIDE_REACH_MARGIN * e.Reach
	if e.MaxStride <= 0 {
		return nil, fmt.Errorf("the feet can not move away from the neutral stance without IK failures")
	}

	return e, nil
}

// CycleTime returns the duration (in seconds) of a full gait cycle with the current duty factor
func (e *GaitEngine) CycleTime() float64 {
	return e.SwingTime / (1 - e.dutyFactor)
}

// SetTwist sets the body velocity the engine will accelerate towards
func (e *GaitEngine) SetTwist(t Twist) {
	e.target = t
}

// GetTwist returns the current (possibly still accelerating) body velocity
func (e *GaitEngine) GetTwist() Twist {
	return e.current
}

// SetGait starts a smooth transition to a new gait
func (e *GaitEngine) SetGait(g *PhaseGait) error {
	if len(g.Offsets) != len(e.Legs) {
		return fmt.Errorf("%s is defined for %d legs, the pod has %d", g.Name, len(g.Offsets), len(e.Legs))
	}
	e.gait = g
	return nil
}

// SetGaitByName starts a smooth transition to a gait by name (tripod, ripple or wave)
// and makes it the pod's current gait
func (e *GaitEngine) SetGaitByName(name string) error {
	gaitType, err := ParseGaitType(name)
	if err != nil {
		return err
	}
	gait, err := NewGait(len(e.Legs), gaitType)
	if err != nil {
		return err
	}
	phaseGait, err := NewPhaseGait(gait, len(e.Legs))
	if err != nil {
		return err
	}
	if err := e.SetGait(phaseGait); err != nil {
		return err
	}
	e.pod.BodyDefinition.Gait = gait
	return nil
}

// SetSwingTime sets the duration (in seconds) of a leg swing
func (e *GaitEngine) SetSwingTime(seconds float64) error {
	if seconds < 0.1 || seconds > 5 {
		return fmt.Errorf("swing time must be between 0.1 and 5 seconds")
	}
	e.SwingTime = seconds
	return nil
}

// SetStepHeight sets the height (in mm) of the swing arc. The maximum stride is recalculated,
// since the legs reach less far when the foot is lifted higher.
func (e *GaitEngine) SetStepHeight(mm float64) error {
	if mm < 0 {
		return fmt.Errorf("step height can not be negative")
	}
	reach := e.pod.ReachRadius(mm)
	if reach <= 0 {
		return fmt.Errorf("a step height of %2.1f mm is out of reach for the legs", mm)
	}
	e.StepHeight = mm
	e.Reach = reach
	e.MaxStride = 2 * STRIDE_REACH_MARGIN * reach
	return nil
}

// IsIdle returns true when the pod is standing still in its neutral stance
func (e *GaitEngine) IsIdle() bool {
	return e.idle
}

// IsTransitioning returns true while the velocity or gait has not yet reached its target
func (e *GaitEngine) IsTransitioning() bool {
	if e.current != e.target || e.dutyFactor != e.gait.DutyFactor {
		return true
	}
	for i := range e.Legs {
		if math.Abs(wrapPhaseError(e.clock+e.gait.Offsets[i]-e.Legs[i].Phase)) > 0.01 {
			return true
		}
	}
	return false
}

// PatternIndex returns the column of an n column gait pattern table matching the current clock
func (e *GaitEngine) PatternIndex(n int) int {
	return int(e.clock*float64(n)) % n
}

// Tick advances the engine dt seconds and updates the pod's servo angles and joints
func (e *GaitEngine) Tick(dt float64) {
	e.rampTwist(dt)
	// Landing targets follow the commanded velocity. The body itself may be slowed down further
	// to keep the feet reachable, but that should not move the target of a foot that is about to land
	commanded := e.limitStride(e.current)
	twist := e.limitReach(commanded, dt)

	if commanded.IsZero() && e.isSettled() {
		e.idle = true
		return
	}
	e.idle = false

	cycles := dt / e.CycleTime()
	e.clock = wrapPhase(e.clock + cycles)
	e.dutyFactor = approach(e.dutyFactor, e.gait.DutyFactor, e.DutyFactorRate*cycles)
	stanceTime := e.dutyFactor * e.CycleTime()

	targets := make([]Coordinate, len(e.Legs))
	for i := range e.Legs {
		l := &e.Legs[i]
		e.advancePhase(i, cycles)
		targets[i] = e.landingTarget(i, commanded, stanceTime)

		if l.Swinging {
			e.updateSwing(l, targets[i], dt)
		} else {
			l.Foot = moveWithGround(l.Foot, twist, dt)
		}
	}

	// Lift off after all landings in this tick, so the support polygon is up to date
	for i := range e.Legs {
		l := &e.Legs[i]
		if l.liftPending && !l.Swinging && e.canLift(i) {
			e.liftOff(l, commanded, targets[i])
		}
		e.solve(i)
	}
}

// advancePhase moves the leg's phase forward, running slightly faster or slower
// than the clock until it matches the leg's offset in the current gait
func (e *GaitEngine) advancePhase(legIndex int, cycles float64) {
	l := &e.Legs[legIndex]
	phaseError := wrapPhaseError(e.clock + e.gait.Offsets[legIndex] - l.Phase)
	correction := math.Max(-e.MaxPhaseCorrection, math.Min(e.MaxPhaseCorrection, e.PhaseGain*phaseError))

	previous := l.Phase
	l.Phase += cycles * (1 + correction)
	if l.Phase >= 1 {
		l.Phase -= 1
	}
	if previous < e.dutyFactor && l.Phase >= e.dutyFactor {
		l.liftPending = true
	}
}

// canLift returns true if the pod stays statically stable without the support of the leg
func (e *GaitEngine) canLift(legIndex int) bool {
	var feet []Coordinate
	for i, l := range e.Legs {
		if i != legIndex && !l.Swinging {
			feet = append(feet, l.Foot)
		}
	}
	return SupportMargin(feet) >= e.StabilityMargin
}

// landingTarget returns the point a swinging foot should land on, so that the following
// stance phase is centred on the leg's neutral position
func (e *GaitEngine) landingTarget(legIndex int, twist Twist, stanceTime float64) Coordinate {
	neutral := e.pod.Legs[legIndex].NeutralEffectorCoordinate
	half := stanceTime / 2

	yaw := twist.Yaw * math.Pi / 180 * half
	x := neutral.X*math.Cos(yaw) - neutral.Y*math.Sin(yaw) + twist.X*half
	y := neutral.X*math.Sin(yaw) + neutral.Y*math.Cos(yaw) + twist.Y*half

	return NewCoordinate(x, y, neutral.Z)
}

// liftOff starts a swing phase, unless the foot is already where it would land
func (e *GaitEngine) liftOff(l *EngineLeg, twist Twist, target Coordinate) {
	l.liftPending = false

	if twist.IsZero() && distance(l.Foot, target) < SETTLE_TOLERANCE {
		return
	}

	l.Swinging = true
	l.liftoff = l.Foot
	l.swingProgress = 0
	// A leg that had to wait for support during a gait transition still gets the full swing time.
	// Its next stance phase is shortened instead.
	l.swingDuration = e.SwingTime
}

// updateSwing moves a swinging foot along an arc from its liftoff point towards the landing target
func (e *GaitEngine) updateSwing(l *EngineLeg, target Coordinate, dt float64) {
	l.swingProgress += dt / l.swingDuration
	if l.swingProgress >= 1 {
		l.Swinging = false
		l.Foot = target
		return
	}

	s := l.swingProgress
	w := s * s * (3 - 2*s) // smoothstep: no velocity jump at liftoff and touchdown
	l.Foot = NewCoordinate(
		l.liftoff.X+(target.X-l.liftoff.X)*w,
		l.liftoff.Y+(target.Y-l.liftoff.Y)*w,
		// Z is positive towards the ground, so lifting the leg means decreasing Z.
		// sin^2 starts and ends the lift with zero vertical velocity
		l.liftoff.Z+(target.Z-l.liftoff.Z)*w-e.StepHeight*math.Pow(math.Sin(math.Pi*s), 2))
}

// solve runs IK for the leg's commanded foot position. On failure the leg keeps its last valid pose.
func (e *GaitEngine) solve(legIndex int) {
	leg := e.pod.Legs[legIndex]
	angles, err := SolveEffectorIK(leg, e.Legs[legIndex].Foot, e.pod.debugChannel)
	if err != nil {
		e.IKErrors++
		e.LastError = fmt.Errorf("leg %d: %w", legIndex, err)
		return
	}
	leg.RecalculateForwardKinematics(angles)
}

// rampTwist moves the current velocity towards the target velocity within the acceleration limits
func (e *GaitEngine) rampTwist(dt float64) {
	dx := e.target.X - e.current.X
	dy := e.target.Y - e.current.Y
	maxStep := e.LinearAcceleration * dt
	if d := math.Hypot(dx, dy); d > maxStep {
		dx *= maxStep / d
		dy *= maxStep / d
	}
	e.current.X += dx
	e.current.Y += dy
	e.current.Yaw = approach(e.current.Yaw, e.target.Yaw, e.YawAcceleration*dt)

	// Avoid creeping forever towards the target because of floating point rounding
	if math.Abs(e.current.X-e.target.X) < 1e-9 && math.Abs(e.current.Y-e.target.Y) < 1e-9 {
		e.current.X, e.current.Y = e.target.X, e.target.Y
	}
}

// limitStride scales the velocity down if any foot would travel further than MaxStride during a stance phase
func (e *GaitEngine) limitStride(t Twist) Twist {
	stanceTime := e.dutyFactor * e.CycleTime()
	yaw := t.Yaw * math.Pi / 180

	longest := 0.0
	for _, l := range e.pod.Legs {
		n := l.NeutralEffectorCoordinate
		// Foot speed relative to the body is v + yaw x r
		stride := math.Hypot(t.X-yaw*n.Y, t.Y+yaw*n.X) * stanceTime
		longest = math.Max(longest, stride)
	}

	if longest <= e.MaxStride {
		return t
	}
	scale := e.MaxStride / longest
	return Twist{X: t.X * scale, Y: t.Y * scale, Yaw: t.Yaw * scale}
}

// limitReach slows the body down when a grounded foot moving away from its neutral position
// gets close to the edge of the leg's reach. Slowing down is immediate, speeding up again is gradual.
func (e *GaitEngine) limitReach(t Twist, dt float64) Twist {
	soft := SLOWDOWN_REACH_FRACTION * e.Reach
	yaw := t.Yaw * math.Pi / 180

	scale := 1.0
	for i, l := range e.Legs {
		if l.Swinging {
			continue
		}
		n := e.pod.Legs[i].NeutralEffectorCoordinate
		dx, dy := l.Foot.X-n.X, l.Foot.Y-n.Y
		// Grounded feet move opposite to the body: -(v + yaw x r)
		vx, vy := -(t.X - yaw*l.Foot.Y), -(t.Y + yaw*l.Foot.X)
		offset := math.Hypot(dx, dy)
		if offset <= soft || dx*vx+dy*vy <= 0 {
			continue
		}
		scale = math.Min(scale, math.Max(0, (e.Reach-offset)/(e.Reach-soft)))
	}

	e.speedScale = math.Min(scale, e.speedScale+SPEED_RECOVERY_RATE*dt)
	return Twist{X: t.X * e.speedScale, Y: t.Y * e.speedScale, Yaw: t.Yaw * e.speedScale}
}

// isSettled returns true if all feet are grounded in their neutral positions
func (e *GaitEngine) isSettled() bool {
	for i, l := range e.Legs {
		if l.Swinging || distance(l.Foot, e.pod.Legs[i].NeutralEffectorCoordinate) > SETTLE_TOLERANCE {
			return false
		}
	}
	return true
}

// SupportMargin returns the distance from the body origin to the nearest edge of the support
// polygon spanned by the grounded feet. The result is negative if the origin is outside the polygon.
func SupportMargin(feet []Coordinate) float64 {
	hull := convexHull(feet)
	if len(hull) < 3 {
		return math.Inf(-1)
	}
	margin := math.Inf(1)
	for i := range hull {
		a := hull[i]
		b := hull[(i+1)%len(hull)]
		// Signed distance from the origin to the edge a->b (positive inside a counter clockwise hull)
		d := (a.X*b.Y - a.Y*b.X) / math.Hypot(b.X-a.X, b.Y-a.Y)
		margin = math.Min(margin, d)
	}
	return margin
}

// convexHull returns the XY convex hull of the points in counter clockwise order (Andrew's monotone chain)
func convexHull(points []Coordinate) []Coordinate {
	if len(points) < 3 {
		return points
	}
	p := append([]Coordinate(nil), points...)
	sort.Slice(p, func(i, j int) bool {
		return p[i].X < p[j].X || (p[i].X == p[j].X && p[i].Y < p[j].Y)
	})
	cross := func(o, a, b Coordinate) float64 {
		return (a.X-o.X)*(b.Y-o.Y) - (a.Y-o.Y)*(b.X-o.X)
	}

	hull := make([]Coordinate, 0, 2*len(p))
	for _, pt := range p {
		for len(hull) >= 2 && cross(hull[len(hull)-2], hull[len(hull)-1], pt) <= 0 {
			hull = hull[:len(hull)-1]
		}
		hull = append(hull, pt)
	}
	lower := len(hull) + 1
	for i := len(p) - 2; i >= 0; i-- {
		for len(hull) >= lower && cross(hull[len(hull)-2], hull[len(hull)-1], p[i]) <= 0 {
			hull = hull[:len(hull)-1]
		}
		hull = append(hull, p[i])
	}
	return hull[:len(hull)-1]
}

// moveWithGround moves a grounded foot opposite to the body's motion
func moveWithGround(foot Coordinate, t Twist, dt float64) Coordinate {
	yaw := -t.Yaw * math.Pi / 180 * dt
	return NewCoordinate(
		foot.X*math.Cos(yaw)-foot.Y*math.Sin(yaw)-t.X*dt,
		foot.X*math.Sin(yaw)+foot.Y*math.Cos(yaw)-t.Y*dt,
		foot.Z)
}

func distance(a Coordinate, b Coordinate) float64 {
	return math.Sqrt((a.X-b.X)*(a.X-b.X) + (a.Y-b.Y)*(a.Y-b.Y) + (a.Z-b.Z)*(a.Z-b.Z))
}

// approach moves value towards target by at most maxStep
func approach(value float64, target float64, maxStep float64) float64 {
	if math.Abs(target-value) <= maxStep {
		return target
	}
	if target > value {
		return value + maxStep
	}
	return value - maxStep
}

// wrapPhase wraps a phase into [0, 1)
func wrapPhase(phase float64) float64 {
	phase = math.Mod(phase, 1)
	if phase < 0 {
		phase += 1
	}
	return phase
}

// wrapPhaseError wraps a phase difference into [-0.5, 0.5)
func wrapPhaseError(diff float64) float64 {
	return wrapPhase(diff+0.5) - 0.5
}
