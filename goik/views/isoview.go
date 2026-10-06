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

package views

import (
	"GOIK/robot"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
	"gonum.org/v1/gonum/mat"
)

type IsoView struct {
	x            float32
	y            float32
	size         float32
	legendOffset float32
}

func NewIsoView(x float32, y float32, size float32) *IsoView {
	return &IsoView{x: x, y: y, size: size, legendOffset: 10}
}

func (v *IsoView) TranslateX(x float64) int {
	return int(v.x) + int(v.size)/2 - int(v.legendOffset) + int(x)
}

func (v *IsoView) TranslateY(y float64) int {
	return int(v.y) + int(v.size)/2 - int(v.legendOffset) + int(y)
}

func (v *IsoView) Render(screen *ebiten.Image, p *robot.Pod) {
	DrawFrame(screen, "Isometric View", v.size, v.x, v.y, v.legendOffset)
	joints := p.GroundJoints()

	angle_z := 0 * math.Pi / 180.0
	angle_x := 20 * math.Pi / 180.0
	angle_y := 20 * math.Pi / 180.0

	// Z Rotation matrix
	R_Z := mat.NewDense(3, 3, []float64{
		math.Cos(angle_z), -math.Sin(angle_z), 0,
		math.Sin(angle_z), math.Cos(angle_z), 0,
		0, 0, 1,
	})

	// X Rotation matrix
	R_X := mat.NewDense(3, 3, []float64{
		1, 0, 0,
		0, math.Cos(angle_x), -math.Sin(angle_x),
		0, math.Sin(angle_x), math.Cos(angle_x),
	})

	// Y Rotation matrix
	R_Y := mat.NewDense(3, 3, []float64{
		math.Cos(angle_y), 0, math.Sin(angle_y),
		0, 1, 0,
		-math.Sin(angle_y), 0, math.Cos(angle_y),
	})

	var R_Iso2 mat.Dense
	R_Iso2.Mul(R_Z, R_X)
	var R_Iso mat.Dense
	R_Iso.Mul(R_Y, &R_Iso2)

	H_Iso := mat.NewDense(4, 4, []float64{
		R_Iso.At(0, 0), R_Iso.At(0, 1), R_Iso.At(0, 2), 0,
		R_Iso.At(1, 0), R_Iso.At(1, 1), R_Iso.At(1, 2), 0,
		R_Iso.At(2, 0), R_Iso.At(2, 1), R_Iso.At(2, 2), 0,
		0, 0, 0, 1,
	})

	// Ground plane: the world fixed grid at the height of the neutral feet
	groundZ := 0.0
	for _, l := range p.Legs {
		groundZ += l.NeutralEffectorCoordinate.Z / float64(len(p.Legs))
	}
	project := func(x float64, y float64, z float64) [2]float32 {
		return [2]float32{
			float32(v.TranslateX(R_Iso.At(0, 0)*x + R_Iso.At(0, 1)*y + R_Iso.At(0, 2)*z)),
			float32(v.TranslateY(R_Iso.At(2, 0)*x + R_Iso.At(2, 1)*y + R_Iso.At(2, 2)*z)),
		}
	}
	var gridLines [][][2]float32
	for _, l := range GroundGrid(p) {
		gridLines = append(gridLines, [][2]float32{project(l[0][0], l[0][1], groundZ), project(l[1][0], l[1][1], groundZ)})
	}
	StrokePaths(Clip(screen, v.x, v.y, v.size), gridLines, 1, GridClr())

	var IsoJoints = make([][robot.NUM_JOINTS]robot.Coordinate, p.BodyDefinition.NumLegs*robot.NUM_JOINTS)

	for l := 0; l < p.BodyDefinition.NumLegs; l++ {
		for j := 0; j < robot.NUM_JOINTS; j++ {
			J := mat.NewDense(4, 1, []float64{
				joints[l][j].X,
				joints[l][j].Y,
				joints[l][j].Z,
				1,
			})

			var T mat.Dense
			T.Mul(H_Iso, J)
			IsoJoints[l][j].X = T.At(0, 0)
			IsoJoints[l][j].Y = T.At(1, 0)
			IsoJoints[l][j].Z = T.At(2, 0)
		}
	}

	// Draw body frame
	for l := 0; l < p.BodyDefinition.NumLegs-1; l++ {
		vector.StrokeLine(screen,
			float32(v.TranslateX(IsoJoints[l][0].X)),
			float32(v.TranslateY(IsoJoints[l][0].Z)),
			float32(v.TranslateX(IsoJoints[l+1][0].X)),
			float32(v.TranslateY(IsoJoints[l+1][0].Z)),
			5,
			White(),
			true)
	}
	vector.StrokeLine(screen,
		float32(v.TranslateX(IsoJoints[p.BodyDefinition.NumLegs-1][0].X)),
		float32(v.TranslateY(IsoJoints[p.BodyDefinition.NumLegs-1][0].Z)),
		float32(v.TranslateX(IsoJoints[0][0].X)),
		float32(v.TranslateY(IsoJoints[0][0].Z)),
		5,
		White(),
		true)

	// Draw Coxa, Femur and Tibia
	for j := 0; j < robot.NUM_JOINTS-1; j++ {
		for l := 0; l < p.BodyDefinition.NumLegs; l++ {
			col := White()
			width := 3
			if p.IsSwingPhase(l) {
				col = Blue()
			}
			vector.StrokeLine(screen,
				float32(v.TranslateX(IsoJoints[l][j].X)),
				float32(v.TranslateY(IsoJoints[l][j].Z)),
				float32(v.TranslateX(IsoJoints[l][j+1].X)),
				float32(v.TranslateY(IsoJoints[l][j+1].Z)),
				float32(width),
				col,
				true)
		}
	}

	for l := 0; l < p.BodyDefinition.NumLegs; l++ {
		for j := 0; j < robot.NUM_JOINTS; j++ {
			vector.DrawFilledCircle(screen, float32(v.TranslateX(IsoJoints[l][j].X)), float32(v.TranslateY(IsoJoints[l][j].Z)), 5, Red(), true)
		}
	}
}
