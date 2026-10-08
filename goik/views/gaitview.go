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
	"fmt"
	"image/color"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type GaitView struct {
	x            float32
	y            float32
	size         float32
	legendOffset float32
}

func NewGaitView(x float32, y float32, size float32) *GaitView {
	return &GaitView{x: x, y: y, size: size, legendOffset: 10}
}

func (v *GaitView) TranslateX(x float64) int {
	return int(v.x) + int(v.size)/2 - int(v.legendOffset) + int(x)
}

func (v *GaitView) TranslateY_TopView(y float64) int {
	return int(v.y) + int(v.size)/4 - int(v.legendOffset) + int(y)
}

func (v *GaitView) RenderGait(screen *ebiten.Image, legend string, x_legend float32, x_table float32, y float32, p *robot.Pod) {

	var width float32 = 50
	var height float32 = 40

	ebitenutil.DebugPrintAt(screen, legend, int(v.x+v.legendOffset), int(y-2*v.legendOffset))

	ebitenutil.DebugPrintAt(screen, fmt.Sprintf("Gait cycles: %2.1f", p.GaitCycles()), int(v.x+v.legendOffset), int(y-4*v.legendOffset))

	for leg := 0; leg < p.BodyDefinition.NumLegs; leg++ {
		ebitenutil.DebugPrintAt(screen, fmt.Sprintf("Leg: %d", leg), int(x_legend), int(y+float32(leg)*height))
		for step := 0; step < p.BodyDefinition.Gait.NumIndicesInPattern; step++ {
			var color color.Color
			if (*p.BodyDefinition.Gait.Pattern)[leg][step] == 1 { // Swing
				color = SwingPassiveClr()
				if step == p.CurrentGaitIndex {
					color = SwingActiveClr()
				}
			} else { // Stance
				color = StancePassiveClr()
				if step == p.CurrentGaitIndex {
					color = StanceActiveClr()
				}
			}
			vector.DrawFilledRect(screen,
				x_table+float32(step)*width,
				y+float32(leg)*height,
				width,
				height, color, false)

		}
	}
}

func (v *GaitView) Render(screen *ebiten.Image, p *robot.Pod) {
	var y_offset float32 = v.y + v.legendOffset + 20
	var x_offset float32 = v.x + v.legendOffset + 40
	var pattern_offset float32 = 60

	y_offset += v.size / 5
	v.RenderGait(screen,
		fmt.Sprintf("%s - Phase: white == swing, grey == stance", p.BodyDefinition.Gait.Name),
		x_offset,
		pattern_offset+x_offset,
		y_offset, p)
}
