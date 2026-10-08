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
	"strings"
)

type GaitType int

const (
	TRIPOD GaitType = 0
	WAVE   GaitType = 1
	RIPPLE GaitType = 2
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
	}
	return TRIPOD, fmt.Errorf("unknown gait '%s' (tripod, ripple or wave)", name)
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

	return newGeneratedGait(NumLegs, GaitType)
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
