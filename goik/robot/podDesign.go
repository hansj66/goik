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
	"fmt"
	"math"
	"sort"
)

/*
	A pod design describes a pod that is symmetric about the Y axis (forward is +Y): the left and right sides
	are mirror images. Only the +X half is designed, and the -X half is mirrored from it:

		position (x, y)   ->  (-x, y)
		mount angle a     ->  180 - a
		coxa rest angle c ->  -c        (femur and tibia are unchanged)
		joint twists      ->  opposite  (see twist.go)

	A leg mounted on the symmetry axis (x = 0) is a single leg pointing forward or backward.

	The design generates the pod's body definition (see BodyDefinition), with the legs numbered counter
	clockwise around the body, starting at +X. The gait patterns rely on this order.
*/

// Tolerance (mm and degrees) for being on the symmetry axis, and for the coxa angle of a leg on it
const DESIGN_TOLERANCE = 1e-3

// Tolerance (mm and degrees) when looking for mirror images in an existing pod definition
const DESIGN_IMPORT_TOLERANCE = 0.1

// Legs closer than this (mm) to each other are rejected
const DESIGN_MIN_LEG_DISTANCE = 1.0

// Point2 is a point in the body's XY plane (mm)
type Point2 struct {
	X float64 `json:"X"`
	Y float64 `json:"Y"`
}

// LegMount is a leg on the +X half of the body (mirrored to the -X half), or a single leg on the symmetry axis
type LegMount struct {
	// Position of the coxa joint (mm). X >= 0, and X == 0 is a leg on the symmetry axis
	X float64 `json:"X"`
	Y float64 `json:"Y"`
	// Mount angle (degrees): where the coxa points at a coxa angle of 0. 0 is +X (sideways), 90 is forward.
	// A leg on the symmetry axis points forward (90) or backward (270)
	Angle    float64        `json:"Angle"`
	Segments SegmentLengths `json:"Segments"`
	// Rest angles of the leg on the +X side. The mirrored leg's coxa angle is negated
	Rest ServoAngles `json:"Rest"`
	// Twists of the joint axes of the leg on the +X side (see twist.go). The mirrored leg's twists are opposite
	Twists JointTwists `json:"Twists"`
}

// OnAxis returns true if the leg is on the symmetry axis (a single leg, not a mirrored pair)
func (m LegMount) OnAxis() bool {
	return math.Abs(m.X) < DESIGN_TOLERANCE
}

// PodDesign describes a pod that is symmetric about the Y axis (see the notes above)
type PodDesign struct {
	// The +X half of the body outline: from a point on the symmetry axis, through points with X > 0, to another
	// point on the axis. The -X half is mirrored. May be empty
	Outline []Point2 `json:"Outline"`
	// The legs on the +X half and on the symmetry axis
	Mounts []LegMount `json:"Mounts"`
}

// DesignLeg is a leg generated from a design
type DesignLeg struct {
	// Index of the leg's mount in the design, and whether this is the mirror image (-X side) of the mount
	Mount    int
	Mirrored bool
	Position Coordinate
	Angle    float64
	Segments SegmentLengths
	Rest     ServoAngles
	Twists   JointTwists
}

// Clone returns a deep copy of the design
func (d *PodDesign) Clone() *PodDesign {
	c := &PodDesign{}
	c.Outline = append(c.Outline, d.Outline...)
	c.Mounts = append(c.Mounts, d.Mounts...)
	return c
}

// Validate checks the outline and the leg mounts
func (d *PodDesign) Validate() error {
	if err := validateOutline(d.Outline); err != nil {
		return err
	}
	for _, m := range d.Mounts {
		at := fmt.Sprintf("(%.2f, %.2f)", m.X, m.Y)
		if m.X < -DESIGN_TOLERANCE {
			return fmt.Errorf("the leg at %s is on the -X side. Design the +X side, it is mirrored", at)
		}
		if math.Hypot(m.X, m.Y) < DESIGN_TOLERANCE {
			return fmt.Errorf("a leg can't be mounted at the centre of the body")
		}
		if m.Segments.Coxa < 0 || m.Segments.Femur <= 0 || m.Segments.Tibia <= 0 {
			return fmt.Errorf("the leg at %s has invalid segment lengths (coxa %.2f, femur %.2f, tibia %.2f)", at,
				m.Segments.Coxa, m.Segments.Femur, m.Segments.Tibia)
		}
		if m.OnAxis() {
			a := normalizeAngle(m.Angle)
			if math.Abs(a-90) > DESIGN_TOLERANCE && math.Abs(a-270) > DESIGN_TOLERANCE {
				return fmt.Errorf("the leg at %s is on the symmetry axis, so it must point forward (90) or backward (270), not %.2f", at, m.Angle)
			}
			if math.Abs(m.Rest.Coxa) > DESIGN_TOLERANCE {
				return fmt.Errorf("the leg at %s is on the symmetry axis, so its coxa rest angle must be 0, not %.2f", at, m.Rest.Coxa)
			}
			if !m.Twists.IsZero() {
				return fmt.Errorf("the leg at %s is on the symmetry axis, so its joints can't be twisted (it is its own mirror image)", at)
			}
		}
		if err := m.Twists.Validate(); err != nil {
			return fmt.Errorf("the leg at %s: %w", at, err)
		}
	}
	return nil
}

// validateOutline checks that the half outline starts and ends on the symmetry axis, stays on the +X side
// in between, and doesn't cross itself. Then the mirrored outline is a simple polygon.
func validateOutline(outline []Point2) error {
	if len(outline) == 0 {
		return nil
	}
	n := len(outline)
	if n < 3 {
		return fmt.Errorf("the outline needs at least 3 points (it has %d)", n)
	}
	if math.Abs(outline[0].X) > DESIGN_TOLERANCE || math.Abs(outline[n-1].X) > DESIGN_TOLERANCE {
		return fmt.Errorf("the outline must start and end on the symmetry axis (X = 0)")
	}
	for _, p := range outline[1 : n-1] {
		if p.X <= DESIGN_TOLERANCE {
			return fmt.Errorf("the outline's point (%.2f, %.2f) is not on the +X side. Only the first and last points are on the axis", p.X, p.Y)
		}
	}
	for i := 0; i < n-1; i++ {
		for j := i + 2; j < n-1; j++ {
			if segmentsIntersect(outline[i], outline[i+1], outline[j], outline[j+1]) {
				return fmt.Errorf("the outline crosses itself (between (%.2f, %.2f) and (%.2f, %.2f))",
					outline[i].X, outline[i].Y, outline[j].X, outline[j].Y)
			}
		}
	}
	return nil
}

// segmentsIntersect returns true if the line segments a-b and c-d intersect or touch
func segmentsIntersect(a, b, c, d Point2) bool {
	cross := func(o, p, q Point2) float64 { return (p.X-o.X)*(q.Y-o.Y) - (p.Y-o.Y)*(q.X-o.X) }
	onSegment := func(p, q, r Point2) bool {
		return math.Min(p.X, q.X) <= r.X && r.X <= math.Max(p.X, q.X) && math.Min(p.Y, q.Y) <= r.Y && r.Y <= math.Max(p.Y, q.Y)
	}
	d1, d2 := cross(c, d, a), cross(c, d, b)
	d3, d4 := cross(a, b, c), cross(a, b, d)
	if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) {
		return true
	}
	return (d1 == 0 && onSegment(c, d, a)) || (d2 == 0 && onSegment(c, d, b)) ||
		(d3 == 0 && onSegment(a, b, c)) || (d4 == 0 && onSegment(a, b, d))
}

// FullOutline returns the whole body outline (the half outline and its mirror image) as a closed polygon,
// or nil if the design has no outline
func (d *PodDesign) FullOutline() []Point2 {
	if len(d.Outline) == 0 {
		return nil
	}
	full := append([]Point2{}, d.Outline...)
	for i := len(d.Outline) - 2; i > 0; i-- {
		full = append(full, Point2{X: -d.Outline[i].X, Y: d.Outline[i].Y})
	}
	return full
}

// Legs validates the design and returns its legs, numbered counter clockwise around the body from +X
func (d *PodDesign) Legs() ([]DesignLeg, error) {
	if err := d.Validate(); err != nil {
		return nil, err
	}

	var legs []DesignLeg
	for i, m := range d.Mounts {
		y := m.Y + 0 // no negative zero
		if m.OnAxis() {
			legs = append(legs, DesignLeg{Mount: i, Position: Coordinate{X: 0, Y: y}, Angle: normalizeAngle(m.Angle), Segments: m.Segments, Rest: ServoAngles{Femur: m.Rest.Femur, Tibia: m.Rest.Tibia}, Twists: m.Twists})
			continue
		}
		legs = append(legs, DesignLeg{Mount: i, Position: Coordinate{X: m.X, Y: y}, Angle: normalizeAngle(m.Angle), Segments: m.Segments, Rest: m.Rest, Twists: m.Twists})
		mirrored := m.Rest
		mirrored.Coxa = 0 - m.Rest.Coxa
		legs = append(legs, DesignLeg{Mount: i, Mirrored: true, Position: Coordinate{X: -m.X, Y: y}, Angle: normalizeAngle(180 - m.Angle), Segments: m.Segments, Rest: mirrored, Twists: m.Twists.Mirrored()})
	}

	if len(legs) < 3 {
		return nil, fmt.Errorf("a pod needs at least 3 legs (the design has %d)", len(legs))
	}

	sort.SliceStable(legs, func(i, j int) bool {
		ai, aj := positionAngle(legs[i].Position), positionAngle(legs[j].Position)
		if ai != aj {
			return ai < aj
		}
		return math.Hypot(legs[i].Position.X, legs[i].Position.Y) < math.Hypot(legs[j].Position.X, legs[j].Position.Y)
	})

	for i := range legs {
		for j := i + 1; j < len(legs); j++ {
			a, b := legs[i].Position, legs[j].Position
			if math.Hypot(a.X-b.X, a.Y-b.Y) < DESIGN_MIN_LEG_DISTANCE {
				return nil, fmt.Errorf("legs %d (%.2f, %.2f) and %d (%.2f, %.2f) are less than %.0f mm apart", i, a.X, a.Y, j, b.X, b.Y, DESIGN_MIN_LEG_DISTANCE)
			}
		}
	}
	return legs, nil
}

// Mirror returns the leg that is the mirror image of the given leg, or -1 if it has none (it is on the
// symmetry axis) or the design is invalid
func (d *PodDesign) Mirror(leg int) int {
	legs, err := d.Legs()
	if err != nil || leg < 0 || leg >= len(legs) {
		return -1
	}
	for i, l := range legs {
		if i != leg && l.Mount == legs[leg].Mount {
			return i
		}
	}
	return -1
}

// BodyDefinition builds a body definition from the design. The gait type and the servo mapping are taken
// from previous (which may be nil) when they fit the new number of legs. Otherwise the pod gets tripod gait
// (wave gait for an odd number of legs) and the default servo mapping for the previous servo model.
func (d *PodDesign) BodyDefinition(previous *BodyDefinition) (*BodyDefinition, error) {
	legs, err := d.Legs()
	if err != nil {
		return nil, err
	}

	n := len(legs)
	b := &BodyDefinition{NumLegs: n, Design: d}
	for _, l := range legs {
		b.CoxaAngles = append(b.CoxaAngles, l.Angle)
		b.CoxaCoordinates = append(b.CoxaCoordinates, l.Position)
		b.Segments = append(b.Segments, l.Segments)
		b.RestAngles = append(b.RestAngles, l.Rest)
		b.Twists = append(b.Twists, l.Twists)
	}
	if !b.HasTwists() {
		b.Twists = nil
	}

	gaitType := TRIPOD
	if n%2 != 0 {
		gaitType = WAVE
	}
	if previous != nil {
		if t, err := previous.Gait.Type(); err == nil {
			if _, err := NewGaitFor(b, t); err == nil {
				gaitType = t
			}
		}
	}
	if b.Gait, err = NewGaitFor(b, gaitType); err != nil {
		return nil, err
	}

	if previous != nil && previous.Servos != nil {
		if previous.NumLegs == n {
			b.Servos = previous.Servos
		} else {
			b.Servos = NewDefaultServoMapping(n)
			b.Servos.Model = previous.Servos.Model
		}
	}
	return b, nil
}

// SetLeg sets the segment lengths, rest angles and joint twists of a leg (as seen on that leg). The leg's mirror
// image changes with it. A leg on the symmetry axis must have a coxa rest angle of 0 and no twists.
func (d *PodDesign) SetLeg(leg int, segments SegmentLengths, rest ServoAngles, twists JointTwists) error {
	legs, err := d.Legs()
	if err != nil {
		return err
	}
	if leg < 0 || leg >= len(legs) {
		return fmt.Errorf("invalid leg index %d (the design has %d legs)", leg, len(legs))
	}

	m := &d.Mounts[legs[leg].Mount]
	if legs[leg].Mirrored {
		rest.Coxa = 0 - rest.Coxa
		twists = twists.Mirrored()
	}
	if m.OnAxis() {
		if math.Abs(rest.Coxa) > DESIGN_TOLERANCE {
			return fmt.Errorf("leg %d is on the symmetry axis, so its coxa rest angle must be 0", leg)
		}
		if !twists.IsZero() {
			return fmt.Errorf("leg %d is on the symmetry axis, so its joints can't be twisted (it is its own mirror image)", leg)
		}
		rest.Coxa = 0
	}
	if err := twists.Validate(); err != nil {
		return fmt.Errorf("leg %d: %w", leg, err)
	}
	m.Segments = segments
	m.Rest = rest
	m.Twists = twists
	return nil
}

// MoveLeg moves a leg (and its mirror image) to a new position and mount angle, given as seen on that leg
func (d *PodDesign) MoveLeg(leg int, x float64, y float64, angle float64) error {
	legs, err := d.Legs()
	if err != nil {
		return err
	}
	if leg < 0 || leg >= len(legs) {
		return fmt.Errorf("invalid leg index %d (the design has %d legs)", leg, len(legs))
	}

	m := &d.Mounts[legs[leg].Mount]
	if legs[leg].Mirrored {
		x, angle = -x, 180-angle
	}
	m.X, m.Y, m.Angle = x, y, normalizeAngle(angle)
	if m.OnAxis() {
		m.X = 0
	}
	return nil
}

// Radius returns the distance (mm) from the centre of the body to the leg mount furthest from it
func (d *PodDesign) Radius() float64 {
	r := 0.0
	for _, m := range d.Mounts {
		r = math.Max(r, math.Hypot(m.X, m.Y))
	}
	return r
}

// SetRadius scales the design (see Scale) so the leg mount furthest from the centre is radius mm from it
func (d *PodDesign) SetRadius(radius float64) error {
	if radius <= 0 {
		return fmt.Errorf("the radius must be positive")
	}
	current := d.Radius()
	if current < DESIGN_TOLERANCE {
		return fmt.Errorf("the design has no legs away from the centre")
	}
	return d.Scale(radius / current)
}

// Scale makes the body larger (factor > 1) or smaller: the leg mounts and the outline move away from or towards the
// centre of the body. The legs themselves don't change
func (d *PodDesign) Scale(factor float64) error {
	if factor <= 0 {
		return fmt.Errorf("the scale factor must be positive")
	}
	for i := range d.Mounts {
		d.Mounts[i].X = round2(d.Mounts[i].X * factor)
		d.Mounts[i].Y = round2(d.Mounts[i].Y * factor)
	}
	for i := range d.Outline {
		d.Outline[i].X = round2(d.Outline[i].X * factor)
		d.Outline[i].Y = round2(d.Outline[i].Y * factor)
	}
	return nil
}

// RemoveLeg removes a leg and its mirror image
func (d *PodDesign) RemoveLeg(leg int) error {
	legs, err := d.Legs()
	if err != nil {
		return err
	}
	if leg < 0 || leg >= len(legs) {
		return fmt.Errorf("invalid leg index %d (the design has %d legs)", leg, len(legs))
	}
	i := legs[leg].Mount
	d.Mounts = append(d.Mounts[:i], d.Mounts[i+1:]...)
	return nil
}

// NewDesignFromBodyDefinition creates a design from a pod definition that is symmetric about the Y axis
func NewDesignFromBodyDefinition(b *BodyDefinition) (*PodDesign, error) {
	const tol = DESIGN_IMPORT_TOLERANCE
	near := func(a, b float64) bool { return math.Abs(a-b) < tol }
	nearAngle := func(a, b float64) bool {
		diff := math.Abs(normalizeAngle(a) - normalizeAngle(b))
		return diff < tol || diff > 360-tol
	}

	d := &PodDesign{}
	used := make([]bool, b.NumLegs)
	for i := 0; i < b.NumLegs; i++ {
		if used[i] {
			continue
		}
		c, a, s, r, tw := b.CoxaCoordinates[i], b.CoxaAngles[i], b.Segments[i], b.RestAngles[i], b.LegTwists(i)
		if !near(c.Z, 0) {
			return nil, fmt.Errorf("leg %d is mounted at Z = %.2f. A design has all legs mounted at Z = 0", i, c.Z)
		}

		if near(c.X, 0) {
			if !nearAngle(a, 90) && !nearAngle(a, 270) {
				return nil, fmt.Errorf("leg %d is on the symmetry axis (X = 0), but points sideways (%.2f degrees)", i, a)
			}
			if !near(r.Coxa, 0) {
				return nil, fmt.Errorf("leg %d is on the symmetry axis (X = 0), but has a coxa rest angle of %.2f", i, r.Coxa)
			}
			if !tw.IsZero() {
				return nil, fmt.Errorf("leg %d is on the symmetry axis (X = 0), but has twisted joints", i)
			}
			used[i] = true
			d.Mounts = append(d.Mounts, LegMount{X: 0, Y: c.Y, Angle: normalizeAngle(a), Segments: s, Rest: ServoAngles{Femur: r.Femur, Tibia: r.Tibia}})
			continue
		}

		mirror := -1
		for j := 0; j < b.NumLegs; j++ {
			cj, sj, rj, twj := b.CoxaCoordinates[j], b.Segments[j], b.RestAngles[j], b.LegTwists(j)
			if j != i && !used[j] && near(cj.X, -c.X) && near(cj.Y, c.Y) && near(cj.Z, c.Z) && nearAngle(b.CoxaAngles[j], 180-a) &&
				near(sj.Coxa, s.Coxa) && near(sj.Femur, s.Femur) && near(sj.Tibia, s.Tibia) &&
				near(rj.Coxa, -r.Coxa) && near(rj.Femur, r.Femur) && near(rj.Tibia, r.Tibia) &&
				near(twj.Coxa, -tw.Coxa) && near(twj.Femur, -tw.Femur) && near(twj.Tibia, -tw.Tibia) {
				mirror = j
				break
			}
		}
		if mirror == -1 {
			return nil, fmt.Errorf("leg %d has no mirror image (a leg at (%.2f, %.2f) pointing at %.2f degrees, with the same segment lengths, a coxa rest angle of %.2f and opposite twists). Only pods that are symmetric about the Y axis can be designed",
				i, 0-c.X, c.Y, normalizeAngle(180-a), 0-r.Coxa)
		}
		used[i], used[mirror] = true, true
		if c.X > 0 {
			d.Mounts = append(d.Mounts, LegMount{X: c.X, Y: c.Y, Angle: normalizeAngle(a), Segments: s, Rest: r, Twists: tw})
		} else {
			mr := b.RestAngles[mirror]
			mc := b.CoxaCoordinates[mirror]
			d.Mounts = append(d.Mounts, LegMount{X: mc.X, Y: mc.Y, Angle: normalizeAngle(b.CoxaAngles[mirror]), Segments: s, Rest: mr, Twists: b.LegTwists(mirror)})
		}
	}

	d.Outline = OutlineThroughMounts(d.Mounts)
	if _, err := d.Legs(); err != nil {
		return nil, err
	}
	return d, nil
}

// OutlineThroughMounts returns a half outline through the legs' mount points, from the front to the back,
// or nil if they don't make a valid outline
func OutlineThroughMounts(mounts []LegMount) []Point2 {
	var points []Point2
	for _, m := range mounts {
		points = append(points, Point2{X: m.X, Y: m.Y})
	}
	if len(points) == 0 {
		return nil
	}
	// Around the +X side, from the front (+Y) to the back
	sort.SliceStable(points, func(i, j int) bool {
		return math.Atan2(points[i].Y, points[i].X) > math.Atan2(points[j].Y, points[j].X)
	})
	if first := points[0]; math.Abs(first.X) > DESIGN_TOLERANCE {
		points = append([]Point2{{X: 0, Y: first.Y}}, points...)
	}
	if last := points[len(points)-1]; math.Abs(last.X) > DESIGN_TOLERANCE {
		points = append(points, Point2{X: 0, Y: last.Y})
	}
	if validateOutline(points) != nil {
		return nil
	}
	return points
}

// NewRoundDesign creates a design with legs evenly spaced on a circle, pointing outwards. With an even
// number of legs, leg 0 points sideways (+X). With an odd number, a leg points forward.
func NewRoundDesign(numLegs int, radius float64, segments SegmentLengths, rest ServoAngles) (*PodDesign, error) {
	if numLegs < 3 {
		return nil, fmt.Errorf("a pod needs at least 3 legs")
	}
	if radius <= 0 {
		return nil, fmt.Errorf("the radius must be positive")
	}
	rest.Coxa = 0

	start := 0.0
	if numLegs%2 != 0 {
		start = 90
	}
	d := &PodDesign{}
	for k := 0; k < numLegs; k++ {
		a := normalizeAngle(start + 360*float64(k)/float64(numLegs))
		x := round2(radius * math.Cos(a*math.Pi/180))
		y := round2(radius * math.Sin(a*math.Pi/180))
		if x < 0 {
			continue // mirrored
		}
		d.Mounts = append(d.Mounts, LegMount{X: x + 0, Y: y + 0, Angle: a, Segments: segments, Rest: rest})
	}
	d.Outline = OutlineThroughMounts(d.Mounts)
	if _, err := d.Legs(); err != nil {
		return nil, err
	}
	return d, nil
}

// NewRectangularDesign creates a design with a rectangular body (width along X, length along Y), and the
// legs evenly spaced along its sides from the front corner to the back corner, pointing sideways
func NewRectangularDesign(legsPerSide int, length float64, width float64, segments SegmentLengths, rest ServoAngles) (*PodDesign, error) {
	if legsPerSide < 2 {
		return nil, fmt.Errorf("a rectangular design needs at least 2 legs per side")
	}
	if length <= 0 || width <= 0 {
		return nil, fmt.Errorf("the length and width must be positive")
	}
	rest.Coxa = 0

	x := round2(width / 2)
	d := &PodDesign{Outline: []Point2{{X: 0, Y: round2(length / 2)}, {X: x, Y: round2(length / 2)}, {X: x, Y: round2(-length / 2)}, {X: 0, Y: round2(-length / 2)}}}
	for i := 0; i < legsPerSide; i++ {
		y := round2(length/2 - float64(i)*length/float64(legsPerSide-1))
		d.Mounts = append(d.Mounts, LegMount{X: x, Y: y + 0, Angle: 0, Segments: segments, Rest: rest})
	}
	if _, err := d.Legs(); err != nil {
		return nil, err
	}
	return d, nil
}

// normalizeAngle returns the angle in [0, 360) degrees
func normalizeAngle(a float64) float64 {
	a = math.Mod(a, 360)
	if a < 0 {
		a += 360
	}
	if a >= 360 {
		a -= 360
	}
	return a + 0 // no negative zero
}

// positionAngle returns the direction of a position from the centre of the body, in [0, 360) degrees from +X
func positionAngle(c Coordinate) float64 {
	return normalizeAngle(math.Atan2(c.Y, c.X) * 180 / math.Pi)
}

// round2 rounds to 0.01 (mm)
func round2(v float64) float64 {
	return math.Round(v*100) / 100
}
