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

package simulator

import (
	"GOIK/robot"
	"fmt"
	"sort"
	"strconv"
	"strings"
)

const designUsage = "design [import | round <legs> <radius> | rectangle <legs per side> <length> <width> | outline <auto | x y x y ...> | " +
	"add <x> <y> <angle> | move <leg> <x> <y> [angle] | angle <leg> <angle> | remove <leg> | scale <factor> | radius <mm> | off]"

// executeDesignCmd shows or changes the pod's design (see docs/designing-a-pod.md)
func (s *Shell) executeDesignCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)
	if len(args) == 1 {
		return s.showDesign()
	}

	numbers := func(from int) ([]float64, error) {
		var values []float64
		for _, a := range args[from:] {
			v, err := strconv.ParseFloat(a, 64)
			if err != nil {
				return nil, fmt.Errorf("syntax error ('%s'): '%s' is not a number", designUsage, a)
			}
			values = append(values, v)
		}
		return values, nil
	}
	syntaxError := fmt.Errorf("syntax error ('%s'): %+v", designUsage, args)
	if s.Pod.BodyDefinition.IsSegmented() {
		return fmt.Errorf("a segmented body can't be designed with 'design' (yet)")
	}

	// The legs of a new design get the current pod's leg 0, pointing straight out
	segments, rest := s.Pod.BodyDefinition.Segments[0], s.Pod.BodyDefinition.RestAngles[0]
	rest.Coxa = 0

	switch strings.ToLower(args[1]) {
	case "import":
		if len(args) != 2 {
			return syntaxError
		}
		d, err := robot.NewDesignFromBodyDefinition(s.Pod.BodyDefinition)
		if err != nil {
			return err
		}
		return s.applyDesign(d)

	case "round":
		v, err := numbers(2)
		if err != nil || len(v) != 2 || v[0] != float64(int(v[0])) {
			return syntaxError
		}
		d, err := robot.NewRoundDesign(int(v[0]), v[1], segments, rest)
		if err != nil {
			return err
		}
		return s.applyDesign(d)

	case "rectangle":
		v, err := numbers(2)
		if err != nil || len(v) != 3 || v[0] != float64(int(v[0])) {
			return syntaxError
		}
		d, err := robot.NewRectangularDesign(int(v[0]), v[1], v[2], segments, rest)
		if err != nil {
			return err
		}
		return s.applyDesign(d)

	case "off":
		if s.Pod.BodyDefinition.Design == nil {
			return fmt.Errorf("the pod has no design")
		}
		s.Pod.BodyDefinition.Design = nil
		s.outputCh <- "The pod no longer has a design. Its legs can now be changed one at a time"
		return nil
	}

	// The rest change the current design
	if s.Pod.BodyDefinition.Design == nil {
		return fmt.Errorf("the pod has no design. Start one with 'design import', 'design round' or 'design rectangle'")
	}
	d := s.Pod.BodyDefinition.Design.Clone()
	legs, err := d.Legs()
	if err != nil {
		return err
	}
	leg := func() (int, error) {
		if len(args) < 3 {
			return 0, syntaxError
		}
		l, err := strconv.Atoi(args[2])
		if err != nil || l < 0 || l >= len(legs) {
			return 0, fmt.Errorf("invalid leg index '%s' (the pod has %d legs)", args[2], len(legs))
		}
		return l, nil
	}

	switch strings.ToLower(args[1]) {
	case "outline":
		if len(args) == 3 && strings.ToLower(args[2]) == "auto" {
			d.Outline = robot.OutlineThroughMounts(d.Mounts)
			if d.Outline == nil {
				return fmt.Errorf("the legs' mount points don't make an outline")
			}
			break
		}
		v, err := numbers(2)
		if err != nil || len(v) < 6 || len(v)%2 != 0 {
			return fmt.Errorf("syntax error ('design outline <x y x y ...>', at least 3 points from the +X half, starting and ending at X = 0): %+v", args)
		}
		d.Outline = nil
		for i := 0; i < len(v); i += 2 {
			d.Outline = append(d.Outline, robot.Point2{X: v[i], Y: v[i+1]})
		}

	case "add":
		v, err := numbers(2)
		if err != nil || len(v) != 3 {
			return syntaxError
		}
		// The new leg is a copy of the first one in the design
		m := d.Mounts[0]
		m.X, m.Y, m.Angle = v[0], v[1], v[2]
		m.Rest.Coxa = 0
		d.Mounts = append(d.Mounts, m)

	case "move":
		l, err := leg()
		if err != nil {
			return err
		}
		v, err := numbers(3)
		if err != nil || (len(v) != 2 && len(v) != 3) {
			return syntaxError
		}
		angle := legs[l].Angle
		if len(v) == 3 {
			angle = v[2]
		}
		if err := d.MoveLeg(l, v[0], v[1], angle); err != nil {
			return err
		}

	case "angle":
		l, err := leg()
		if err != nil {
			return err
		}
		v, err := numbers(3)
		if err != nil || len(v) != 1 {
			return syntaxError
		}
		p := legs[l].Position
		if err := d.MoveLeg(l, p.X, p.Y, v[0]); err != nil {
			return err
		}

	case "remove":
		l, err := leg()
		if err != nil {
			return err
		}
		if err := d.RemoveLeg(l); err != nil {
			return err
		}

	case "scale":
		v, err := numbers(2)
		if err != nil || len(v) != 1 {
			return syntaxError
		}
		if err := d.Scale(v[0]); err != nil {
			return err
		}

	case "radius":
		v, err := numbers(2)
		if err != nil || len(v) != 1 {
			return syntaxError
		}
		if err := d.SetRadius(v[0]); err != nil {
			return err
		}

	default:
		return syntaxError
	}
	return s.applyDesign(d)
}

// applyDesign rebuilds the pod from a design and shows it
func (s *Shell) applyDesign(d *robot.PodDesign) error {
	before := s.Pod.BodyDefinition
	if err := s.Pod.ApplyDesign(d); err != nil {
		return err
	}
	s.stopScript()

	after := s.Pod.BodyDefinition
	if before.Gait != nil && after.Gait.Name != before.Gait.Name {
		s.outputCh <- fmt.Sprintf("The pod now has %d legs and uses %s", after.NumLegs, strings.ToLower(after.Gait.Name))
	}
	if before.Servos != nil && after.Servos != before.Servos {
		s.outputCh <- fmt.Sprintf("The number of legs changed, so the servo mapping is reset to the defaults (%s)", after.Servos.Model)
	}
	return s.showDesign()
}

// showDesign prints the pod's design
func (s *Shell) showDesign() error {
	d := s.Pod.BodyDefinition.Design
	if d == nil {
		s.outputCh <- "The pod has no design. Start one with 'design import' (from a pod that is symmetric about the Y axis), 'design round' or 'design rectangle'"
		return nil
	}
	legs, err := d.Legs()
	if err != nil {
		return err
	}

	s.outputCh <- fmt.Sprintf("Pod design: %d legs, symmetric about the Y axis (+Y is forward). Legs on the -X side mirror those on the +X side (shown)", len(legs))
	s.outputCh <- fmt.Sprintf("Body radius: %.2f mm (the leg mount furthest from the centre)", d.Radius())
	if len(d.Outline) == 0 {
		s.outputCh <- "Outline: none"
	} else {
		var points []string
		for _, p := range d.Outline {
			points = append(points, fmt.Sprintf("(%.2f, %.2f)", p.X, p.Y))
		}
		s.outputCh <- "Outline (+X half): " + strings.Join(points, " ")
	}

	// One line per mount, in leg order
	byMount := make([][]int, len(d.Mounts))
	for i, l := range legs {
		byMount[l.Mount] = append(byMount[l.Mount], i)
	}
	sort.Slice(byMount, func(i, j int) bool { return byMount[i][0] < byMount[j][0] })
	for _, numbers := range byMount {
		l := legs[numbers[0]]
		if l.Mirrored {
			l = legs[numbers[1]]
			numbers[0], numbers[1] = numbers[1], numbers[0]
		}
		which := fmt.Sprintf("Leg %d (on the axis)", numbers[0])
		if len(numbers) == 2 {
			which = fmt.Sprintf("Legs %d (+X) and %d (-X)", numbers[0], numbers[1])
		}
		line := fmt.Sprintf("%s: (%.2f, %.2f), mount angle %.2f, coxa %.2f, femur %.2f, tibia %.2f mm, rest angles %.2f / %.2f / %.2f",
			which, l.Position.X, l.Position.Y, l.Angle, l.Segments.Coxa, l.Segments.Femur, l.Segments.Tibia, l.Rest.Coxa, l.Rest.Femur, l.Rest.Tibia)
		if !l.Twists.IsZero() {
			line += fmt.Sprintf(", twists %.1f / %.1f / %.1f", l.Twists.Coxa, l.Twists.Femur, l.Twists.Tibia)
		}
		s.outputCh <- line
	}
	return nil
}
