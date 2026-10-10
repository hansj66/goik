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

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type XzView struct {
	frame
}

func NewXzView(x float32, y float32, width float32, height float32) *XzView {
	return &XzView{frame: newFrame(x, y, width, height)}
}

func (v *XzView) TranslateX(x float64) int {
	return int(v.x) + int(v.width)/2 - int(v.legendOffset) + int(x-v.centre.X)
}

func (v *XzView) TranslateY(y float64) int {
	return int(v.y) + int(v.height)/2 - int(v.legendOffset) + int(y)
}

func (v *XzView) Render(screen *ebiten.Image, p *robot.Pod) {
	DrawFrame(screen, "XZ View", v.width, v.height, v.x, v.y, v.legendOffset)
	v.centre = p.ViewCentre()
	joints := p.GroundJoints()

	if p.BodyDefinition.IsSegmented() {
		DrawSegmentedBody(screen, p, func(c robot.Coordinate) [2]float32 {
			return [2]float32{float32(v.TranslateX(c.X)), float32(v.TranslateY(c.Z))}
		})
		v.renderLegs(screen, p, joints)
		return
	}

	// Draw body frame
	for l := 0; l < p.BodyDefinition.NumLegs-1; l++ {
		vector.StrokeLine(screen,
			float32(v.TranslateX(joints[l][0].X)),
			float32(v.TranslateY(joints[l][0].Z)),
			float32(v.TranslateX(joints[l+1][0].X)),
			float32(v.TranslateY(joints[l+1][0].Z)),
			5,
			White(),
			true)
	}
	vector.StrokeLine(screen,
		float32(v.TranslateX(joints[p.BodyDefinition.NumLegs-1][0].X)),
		float32(v.TranslateY(joints[p.BodyDefinition.NumLegs-1][0].Z)),
		float32(v.TranslateX(joints[0][0].X)),
		float32(v.TranslateY(joints[0][0].Z)),
		5,
		White(),
		true)

	v.renderLegs(screen, p, joints)
}

// renderLegs draws the legs and their joints
func (v *XzView) renderLegs(screen *ebiten.Image, p *robot.Pod, joints [][robot.NUM_JOINTS]robot.Coordinate) {
	// Draw Coxa, Femur and Tibia
	for j := 0; j < robot.NUM_JOINTS-1; j++ {
		for l := 0; l < p.BodyDefinition.NumLegs; l++ {
			col := White()
			width := 3
			if p.IsSwingPhase(l) {
				col = Blue()
			}
			vector.StrokeLine(screen,
				float32(v.TranslateX(joints[l][j].X)),
				float32(v.TranslateY(joints[l][j].Z)),
				float32(v.TranslateX(joints[l][j+1].X)),
				float32(v.TranslateY(joints[l][j+1].Z)),
				float32(width),
				col,
				true)
		}
	}

	for _, l := range joints {
		for _, j := range l {
			vector.DrawFilledCircle(screen, float32(v.TranslateX(j.X)), float32(v.TranslateY(j.Z)), 5, Red(), true)
		}
	}
}
