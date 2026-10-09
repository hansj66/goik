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

// Example pod with 6 legs. Uneven separation between legs
func NewExampleHexapod0() *BodyDefinition {
	gait, _ := NewHexapodGait(TRIPOD)
	b := &BodyDefinition{
		NumLegs:    6,
		CoxaAngles: []float64{0, 0, 180, 180, 180, 0},
		Gait:       gait,
	}
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{40, 0, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{40, 80, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-40, 80, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-40, 0, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-40, -80, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{40, -80, 0})

	for i := 0; i < b.NumLegs; i++ {
		b.RestAngles = append(b.RestAngles, ServoAngles{Coxa: 0, Femur: -50, Tibia: 100})
		b.Segments = append(b.Segments, SegmentLengths{Coxa: 30, Femur: 70, Tibia: 120})
	}
	return b
}

// Example pod with 6 legs. This can use tripod, ripple and wave gait
func NewExampleHexapod1() *BodyDefinition {
	gait, _ := NewHexapodGait(TRIPOD)
	b := &BodyDefinition{
		NumLegs:    6,
		CoxaAngles: []float64{0, 60, 120, 180, 240, 300},
		Gait:       gait,
	}
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{40, 0, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{20, 34.64, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-20, 34.64, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-40, 0, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-20, -34.64, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{20, -34.64, 0})

	for i := 0; i < b.NumLegs; i++ {
		b.RestAngles = append(b.RestAngles, ServoAngles{Coxa: 0, Femur: -50, Tibia: 100})
		b.Segments = append(b.Segments, SegmentLengths{Coxa: 30, Femur: 70, Tibia: 120})
	}

	return b
}

// Example pod with 6 legs, a relatively small body and short tibias
// This can use tripod, ripple and wave gait
func NewExampleHexapod2() *BodyDefinition {
	gait, _ := NewHexapodGait(TRIPOD)
	b := &BodyDefinition{
		NumLegs:    6,
		CoxaAngles: []float64{0, 60, 120, 180, 240, 300},
		Gait:       gait,
	}
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{40, 0, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{20, 34.64, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-20, 34.64, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-40, 0, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-20, -34.64, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{20, -34.64, 0})

	for i := 0; i < b.NumLegs; i++ {
		b.RestAngles = append(b.RestAngles, ServoAngles{Coxa: 0, Femur: 45, Tibia: 45})
		b.Segments = append(b.Segments, SegmentLengths{Coxa: 53.85, Femur: 48, Tibia: 61.7})
	}

	return b
}

// Example pod with 5 legs. The only valid gate is wave gait
func NewExamplePentapod() *BodyDefinition {
	gait, _ := NewPentapodGait(WAVE)
	b := &BodyDefinition{
		NumLegs:    5,
		CoxaAngles: []float64{0, 72, 144, 216, 288, 360},
		Gait:       gait,
	}

	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{40.00, 0.00, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{12.36, 38.04, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-32.36, 23.51, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-32.36, -23.51, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{12.36, -38.04, 0})

	for i := 0; i < b.NumLegs; i++ {
		b.RestAngles = append(b.RestAngles, ServoAngles{Coxa: 0, Femur: -50, Tibia: 100})
		b.Segments = append(b.Segments, SegmentLengths{Coxa: 40, Femur: 60, Tibia: 150})
	}

	return b
}

// Example pod with 7 legs. The only valid gate is wave gait
func NewHeptapod() *BodyDefinition {
	gait, _ := NewHeptapodGait(WAVE)
	b := &BodyDefinition{
		NumLegs:    7,
		CoxaAngles: []float64{51.43, 102.86, 154.29, 205.72, 257.15, 308.58, 360},

		Gait: gait,
	}

	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{24.94, 31.27, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-8.90, 39.00, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-36.04, 17.36, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-36.04, -17.36, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-8.90, -39.00, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{24.94, -31.27, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{40.00, 0.00, 0})

	for i := 0; i < b.NumLegs; i++ {
		b.RestAngles = append(b.RestAngles, ServoAngles{Coxa: 0, Femur: -50, Tibia: 100})
		b.Segments = append(b.Segments, SegmentLengths{Coxa: 40, Femur: 60, Tibia: 150})
	}

	return b
}

// Example (spider like) pod with 8 legs and varying segment lengths
func NewSpider() *BodyDefinition {
	gait, _ := NewHexapodGait(TRIPOD)
	b := &BodyDefinition{
		NumLegs:    8,
		CoxaAngles: []float64{20, 70, 110, 150, 210, 250, 290, 330},
		Gait:       gait,
	}

	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{10, 10, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{15, 40, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-15, 40, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-10, 10, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-10, -10, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-10, -30, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{10, -30, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{10, -10, 0})

	b.Segments = append(b.Segments, SegmentLengths{Coxa: 20, Femur: 40, Tibia: 60})
	b.Segments = append(b.Segments, SegmentLengths{Coxa: 40, Femur: 70, Tibia: 120})
	b.Segments = append(b.Segments, SegmentLengths{Coxa: 40, Femur: 70, Tibia: 120})
	b.Segments = append(b.Segments, SegmentLengths{Coxa: 20, Femur: 40, Tibia: 60})
	b.Segments = append(b.Segments, SegmentLengths{Coxa: 20, Femur: 40, Tibia: 60})
	b.Segments = append(b.Segments, SegmentLengths{Coxa: 40, Femur: 70, Tibia: 120})
	b.Segments = append(b.Segments, SegmentLengths{Coxa: 40, Femur: 70, Tibia: 120})
	b.Segments = append(b.Segments, SegmentLengths{Coxa: 20, Femur: 40, Tibia: 60})

	for i := 0; i < b.NumLegs; i++ {
		b.RestAngles = append(b.RestAngles, ServoAngles{Coxa: 0, Femur: -50, Tibia: 100})
	}

	return b
}

// Example hexapod designed around real AX-12A servos (32 x 50 x 32 mm cases, see cad/servos.json).
// The coxa mounts are far enough from the centre for the six coxa servo cases to fit. The coxa is
// long enough (60 mm) for the femur servo and its mounting to stay clear of the coxa servo's corners
// at any coxa angle, and the femur clears the tibia servo. The rest pose puts the feet 90 mm outside
// the femur joints and 110 mm below the body. The CAD export (export_cad) of this pod, including
// the printable brackets, has no collisions.
func NewExampleHexapodAX12() *BodyDefinition {
	gait, _ := NewHexapodGait(TRIPOD)
	b := &BodyDefinition{
		NumLegs:    6,
		CoxaAngles: []float64{0, 60, 120, 180, 240, 300},
		Gait:       gait,
	}
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{80, 0, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{40, 69.28, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-40, 69.28, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-80, 0, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-40, -69.28, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{40, -69.28, 0})

	for i := 0; i < b.NumLegs; i++ {
		b.RestAngles = append(b.RestAngles, ServoAngles{Coxa: 0, Femur: -15, Tibia: 97})
		b.Segments = append(b.Segments, SegmentLengths{Coxa: 60, Femur: 75, Tibia: 130})
	}
	b.Servos = NewDefaultServoMapping(b.NumLegs)

	return b
}

// Example hexapod designed around Feetech STS3215 servos (45.22 x 24.72 mm cases, see cad/servos.json).
// The coxa mounts are far enough from the centre for the six coxa servo cases to fit, the coxa (52 mm)
// is long enough for the femur servo's mounting to clear the coxa servo's corners at any coxa angle,
// and the femur clears the tibia servo. The rest pose puts the feet 85 mm outside the femur joints and
// 105 mm below the body. The CAD export of this pod, including the printable brackets, has no collisions.
func NewExampleHexapodSTS3215() *BodyDefinition {
	gait, _ := NewHexapodGait(TRIPOD)
	b := &BodyDefinition{
		NumLegs:    6,
		CoxaAngles: []float64{0, 60, 120, 180, 240, 300},
		Gait:       gait,
	}
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{66, 0, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{33, 57.16, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-33, 57.16, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-66, 0, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{-33, -57.16, 0})
	b.CoxaCoordinates = append(b.CoxaCoordinates, Coordinate{33, -57.16, 0})

	for i := 0; i < b.NumLegs; i++ {
		b.RestAngles = append(b.RestAngles, ServoAngles{Coxa: 0, Femur: -11, Tibia: 94})
		b.Segments = append(b.Segments, SegmentLengths{Coxa: 52, Femur: 70, Tibia: 120})
	}
	b.Servos = NewDefaultServoMapping(b.NumLegs)
	b.Servos.Model = "STS3215"

	return b
}

// Example insect like hexapod with twisted joints (see twist.go): a long, narrow body carried low, short coxas,
// and knees high above the body. The front legs are swept forward and twisted so the feet reach forward, the
// rear legs mirror that backwards, and the middle legs point straight out. A showcase for twisted joints: the
// coxas are too short for real servos, and the CAD export doesn't support twisted joints yet.
func NewExampleInsect() *BodyDefinition {
	d, err := NewRectangularDesign(3, 150, 50, SegmentLengths{Coxa: 25, Femur: 70, Tibia: 110}, ServoAngles{})
	if err != nil {
		panic(err)
	}
	front, rear := &d.Mounts[0], &d.Mounts[2]
	front.Rest.Coxa, rear.Rest.Coxa = 35, -35
	front.Twists = JointTwists{Coxa: -15, Femur: -25, Tibia: 15}
	rear.Twists = JointTwists{Coxa: 15, Femur: 25, Tibia: -15}

	b, err := d.BodyDefinition(nil)
	if err != nil {
		panic(err)
	}
	if err := applyStance(b, &Stance{Height: 55, Reach: 95}); err != nil {
		panic(err)
	}
	return b
}
