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
	Notes regarding gaits

	1) Pentapods, hexapods and heptapods have hand written patterns, since they are easier to read than code.
	   Other numbers of legs get generated patterns (see newGeneratedGait).
	2) Currently supported gaits:
		- Hexapods: 									tripod, ripple and wave gait.
		- Pentapods and heptapods:						wave gait.
		- Other numbers of legs (3 or more):			wave gait, tripod gait (every other leg) for an even
		                                                number of legs, and ripple gait for a multiple of 3.
		  The patterns assume that the legs are numbered in order around the body (as in the example pods and
		  pod designs), so that neighbouring legs don't swing together.
	3) Yes, a symmetrical centipede with metachronal gait would be nice. Unfortunately I've run out of dynamixels
*/

import (
	"fmt"
	"math"
	"strings"
)

type GaitType int

const (
	TRIPOD      GaitType = 0
	WAVE        GaitType = 1
	RIPPLE      GaitType = 2
	METACHRONAL GaitType = 3
)

type GaitPattern [][]int

type Gait struct {
	Pattern             *GaitPattern
	Name                string
	NumIndicesInPattern int
}

func NewHeptapodGait(GaitType GaitType) (*Gait, error) {
	if GaitType != WAVE {
		return nil, fmt.Errorf("Nope. Not doing that. Give it time and you'll figure out why (...)")
	}
	p := make(GaitPattern, 49)

	copy(p, [][]int{
		{1, 0, 0, 0, 0, 0, 0}, // 1 == swing phase, 0 == stance phase
		{0, 1, 0, 0, 0, 0, 0},
		{0, 0, 1, 0, 0, 0, 0},
		{0, 0, 0, 1, 0, 0, 0},
		{0, 0, 0, 0, 1, 0, 0},
		{0, 0, 0, 0, 0, 1, 0},
		{0, 0, 0, 0, 0, 0, 1}})
	return &Gait{
		Pattern:             &p,
		Name:                "Wave gait",
		NumIndicesInPattern: 7,
	}, nil

}

func NewPentapodGait(GaitType GaitType) (*Gait, error) {
	if GaitType != WAVE {
		return nil, fmt.Errorf("Nope. Not doing that. Give it time and you'll figure out why (...)")
	}

	p := make(GaitPattern, 25)

	copy(p, [][]int{
		{1, 0, 0, 0, 0}, // 1 == swing phase, 0 == stance phase
		{0, 1, 0, 0, 0},
		{0, 0, 1, 0, 0},
		{0, 0, 0, 1, 0},
		{0, 0, 0, 0, 1}})
	return &Gait{
		Pattern:             &p,
		Name:                "Wave gait",
		NumIndicesInPattern: 5,
	}, nil
}

func NewHexapodGait(GaitType GaitType) (*Gait, error) {
	p := make(GaitPattern, 36)

	if GaitType == WAVE {
		copy(p, [][]int{{0, 0, 1, 0, 0, 0}, // 1 == swing phase, 0 == stance phase
			{0, 1, 0, 0, 0, 0},
			{1, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 1},
			{0, 0, 0, 0, 1, 0},
			{0, 0, 0, 1, 0, 0}})

		return &Gait{
			Pattern:             &p,
			Name:                "Wave gait",
			NumIndicesInPattern: 6,
		}, nil
	} else if GaitType == RIPPLE {
		copy(p, [][]int{{0, 0, 1, 0, 0, 1}, // 1 == swing phase, 0 == stance phase
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
			{1, 0, 0},
			{0, 1, 0}})

		return &Gait{
			Pattern:             &p,
			Name:                "Ripple gait",
			NumIndicesInPattern: 3,
		}, nil
	}

	// default to TRIPOD
	copy(p, [][]int{
		{0, 1}, // 1 == swing phase, 0 == stance phase
		{1, 0},
		{0, 1},
		{1, 0},
		{0, 1},
		{1, 0},
		{0, 1},
		{1, 0}})
	return &Gait{
		Pattern:             &p,
		Name:                "Tripod gait",
		NumIndicesInPattern: 2,
	}, nil
}

// ParseGaitType converts a gait name (tripod, ripple or wave) to a GaitType
func ParseGaitType(name string) (GaitType, error) {
	switch name {
	case "tripod":
		return TRIPOD, nil
	case "ripple":
		return RIPPLE, nil
	case "wave":
		return WAVE, nil
	case "metachronal":
		return METACHRONAL, nil
	}
	return TRIPOD, fmt.Errorf("unknown gait '%s' (tripod, ripple, wave or metachronal)", name)
}

func NewGait(NumLegs int, GaitType GaitType) (*Gait, error) {

	switch NumLegs {
	case 6:
		return NewHexapodGait(GaitType)
	case 5:
		return NewPentapodGait(GaitType)
	case 7:
		return NewHeptapodGait(GaitType)
	}

	if GaitType == METACHRONAL {
		return nil, fmt.Errorf("the metachronal gait depends on where the legs are on the body (see NewGaitFor)")
	}
	return newGeneratedGait(NumLegs, GaitType)
}

// NewGaitFor creates a gait for a pod. The metachronal gait depends on where the legs are; a segmented body only
// walks with the metachronal gait (the other gaits assume the legs are numbered in order around the body)
func NewGaitFor(b *BodyDefinition, gaitType GaitType) (*Gait, error) {
	if gaitType == METACHRONAL {
		return newMetachronalGait(b)
	}
	if b.IsSegmented() {
		return nil, fmt.Errorf("a segmented body walks with the metachronal gait")
	}
	return NewGait(b.NumLegs, gaitType)
}

// Number of columns of the metachronal gait's pattern for a segmented body: the wave reaches the same phase again
// this many segments further back
const METACHRONAL_WAVELENGTH = 8

// Fraction of the gait cycle a leg spends on the ground in the metachronal gait
const METACHRONAL_DUTY_FACTOR = 0.75

// newMetachronalGait creates a metachronal gait: a wave of steps that runs along the body from the head to the tail
// (as in centipedes), with the legs on the left and right half a cycle apart. A leg's place in the wave is its
// segment (segmented bodies) or its rank along its side of the body, front first (one piece bodies)
func newMetachronalGait(b *BodyDefinition) (*Gait, error) {
	if b.NumLegs < 3 {
		return nil, fmt.Errorf("a pod needs at least 3 legs to walk (this one has %d)", b.NumLegs)
	}
	rank := make([]int, b.NumLegs)
	side := make([]int, b.NumLegs)
	for l := range rank {
		if b.CoxaCoordinates[l].X < 0 {
			side[l] = 1
		}
	}
	var columns int
	if b.IsSegmented() {
		for l := range rank {
			rank[l] = b.LegSegment(l)
		}
		columns = METACHRONAL_WAVELENGTH
	} else {
		// Front (+Y) first on each side
		perSide := 0
		for l := range rank {
			for o := range rank {
				if side[o] == side[l] && (b.CoxaCoordinates[o].Y > b.CoxaCoordinates[l].Y || (b.CoxaCoordinates[o].Y == b.CoxaCoordinates[l].Y && o < l)) {
					rank[l]++
				}
			}
			if rank[l]+1 > perSide {
				perSide = rank[l] + 1
			}
		}
		// The wave spans the body, so each step of rank moves the phase on by 1 / perSide
		columns = 2 * perSide
		for l := range rank {
			rank[l] *= 2
		}
	}

	swing := int(math.Round(float64(columns) * (1 - METACHRONAL_DUTY_FACTOR)))
	if swing < 1 {
		swing = 1
	}
	p := make(GaitPattern, b.NumLegs)
	for l := range p {
		p[l] = make([]int, columns) // 1 == swing phase, 0 == stance phase
		start := rank[l] + side[l]*columns/2
		for c := 0; c < swing; c++ {
			p[l][(start+c)%columns] = 1
		}
	}
	return &Gait{
		Pattern:             &p,
		Name:                "Metachronal gait",
		NumIndicesInPattern: columns,
	}, nil
}

// newGeneratedGait creates a gait pattern for 3 or more legs, numbered in order around the body
//   - Tripod: every other leg swings, so the legs swing in two alternating groups. Needs an even number of legs
//   - Ripple: three groups (leg n swings with legs n+3, n+6 ...), like the hexapod ripple gait. Needs a multiple of 3
//   - Wave: one leg at a time, going around the body
func newGeneratedGait(numLegs int, gaitType GaitType) (*Gait, error) {
	if numLegs < 3 {
		return nil, fmt.Errorf("a pod needs at least 3 legs to walk (this one has %d)", numLegs)
	}

	var name string
	var columns int
	var swingColumn func(leg int) int
	switch gaitType {
	case TRIPOD:
		if numLegs%2 != 0 {
			return nil, fmt.Errorf("tripod gait needs an even number of legs (this pod has %d)", numLegs)
		}
		name, columns = "Tripod gait", 2
		swingColumn = func(leg int) int { return (leg + 1) % 2 }
	case RIPPLE:
		if numLegs%3 != 0 {
			return nil, fmt.Errorf("ripple gait needs a multiple of 3 legs (this pod has %d)", numLegs)
		}
		name, columns = "Ripple gait", 3
		swingColumn = func(leg int) int { return (leg + 2) % 3 }
	default:
		name, columns = "Wave gait", numLegs
		swingColumn = func(leg int) int { return (numLegs - leg) % numLegs }
	}

	p := make(GaitPattern, numLegs)
	for leg := range p {
		p[leg] = make([]int, columns) // 1 == swing phase, 0 == stance phase
		p[leg][swingColumn(leg)] = 1
	}
	return &Gait{
		Pattern:             &p,
		Name:                name,
		NumIndicesInPattern: columns,
	}, nil
}

// Type returns the type of a gait, from its name
func (g *Gait) Type() (GaitType, error) {
	if g == nil {
		return TRIPOD, fmt.Errorf("no gait defined")
	}
	return ParseGaitType(strings.ToLower(strings.TrimSuffix(g.Name, " gait")))
}
