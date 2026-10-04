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

	1) Yes, these patterns could be created algorithmically, but it would make the code harder to read.
	   It also makes my head hurt.
	2) Currently supported gaits:
		- Hexapods: 									tripod, ripple and wave gait.
		- <Odd number>pods (pentapods/heptapods):	    wave gait.
		- <Even number>pods (hexapod): 	                wave gait, ripple gait.
		  (gait type is still an argument for constructing these patterns, since someone just might come
		   up with some clever new gait)
	3) Yes, a symmetrical centipede with metachronal gait would be nice. Unfortunately I've run out of dynamixels
*/

import "fmt"

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

	return nil, fmt.Errorf("missing gait definition for pod with %d legs. Please update gaits.go.", NumLegs)
}
