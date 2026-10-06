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
	"image"
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func Red() color.Color {
	return color.RGBA{255, 0, 0, 1}
}

func Blue() color.Color {
	return color.RGBA{0, 0, 255, 1}
}

func White() color.Color {
	return color.RGBA{255, 255, 255, 1}
}

func SwingPassiveClr() color.Color {
	return color.RGBA{200, 200, 200, 1}
}

func SwingActiveClr() color.Color {
	return color.RGBA{255, 255, 255, 1}
}

func StancePassiveClr() color.Color {
	return color.RGBA{32, 32, 32, 1}
}

func TrailClr() color.Color {
	return color.RGBA{110, 110, 110, 255}
}

func NeutralClr() color.Color {
	return color.RGBA{90, 90, 90, 255}
}

func GridClr() color.Color {
	return color.RGBA{50, 50, 50, 255}
}

func PathClr() color.Color {
	return color.RGBA{210, 150, 40, 255}
}

func StanceActiveClr() color.Color {
	return color.RGBA{64, 64, 64, 1}
}

var (
	whiteImage    = ebiten.NewImage(3, 3)
	whiteSubImage = whiteImage.SubImage(image.Rect(1, 1, 2, 2)).(*ebiten.Image)
)

func init() {
	whiteImage.Fill(color.White)
}

// StrokePaths draws polylines (one per slice of points) with a single draw call.
// Much cheaper than drawing many small shapes one at a time.
func StrokePaths(dst *ebiten.Image, polylines [][][2]float32, width float32, clr color.Color) {
	var path vector.Path
	for _, points := range polylines {
		for i, pt := range points {
			if i == 0 {
				path.MoveTo(pt[0], pt[1])
			} else {
				path.LineTo(pt[0], pt[1])
			}
		}
	}

	vs, is := path.AppendVerticesAndIndicesForStroke(nil, nil, &vector.StrokeOptions{Width: width})
	r, g, b, a := clr.RGBA()
	for i := range vs {
		vs[i].SrcX = 1
		vs[i].SrcY = 1
		vs[i].ColorR = float32(r) / 0xffff
		vs[i].ColorG = float32(g) / 0xffff
		vs[i].ColorB = float32(b) / 0xffff
		vs[i].ColorA = float32(a) / 0xffff
	}

	op := &ebiten.DrawTrianglesOptions{}
	op.ColorScaleMode = ebiten.ColorScaleModePremultipliedAlpha
	dst.DrawTriangles(vs, is, whiteSubImage, op)
}

// Spacing (mm) of the ground grid, and how far around the pod it is drawn
const GRID_SPACING = 50.0
const GRID_RADIUS = 400.0

// GroundGrid returns the lines of a grid that is fixed in the world, in the pod's ground reference
// frame. As the pod walks and turns, the grid moves and rotates under it. Each line is two points.
func GroundGrid(p *robot.Pod) [][2][2]float64 {
	toGround := func(x float64, y float64) (float64, float64) { return x, y }
	wx, wy := 0.0, 0.0
	if p.Engine != nil {
		toGround = p.Engine.WorldToGround
		wx, wy, _ = p.Engine.Odometry()
	}

	var lines [][2][2]float64
	line := func(x0, y0, x1, y1 float64) {
		gx0, gy0 := toGround(x0, y0)
		gx1, gy1 := toGround(x1, y1)
		lines = append(lines, [2][2]float64{{gx0, gy0}, {gx1, gy1}})
	}
	for k := math.Floor((wx - GRID_RADIUS) / GRID_SPACING); k*GRID_SPACING <= wx+GRID_RADIUS; k++ {
		line(k*GRID_SPACING, wy-GRID_RADIUS, k*GRID_SPACING, wy+GRID_RADIUS)
	}
	for k := math.Floor((wy - GRID_RADIUS) / GRID_SPACING); k*GRID_SPACING <= wy+GRID_RADIUS; k++ {
		line(wx-GRID_RADIUS, k*GRID_SPACING, wx+GRID_RADIUS, k*GRID_SPACING)
	}
	return lines
}

// Clip returns the part of the screen covered by a view. Drawing on it is clipped to the view.
func Clip(screen *ebiten.Image, x float32, y float32, size float32) *ebiten.Image {
	return screen.SubImage(image.Rect(int(x), int(y), int(x+size), int(y+size))).(*ebiten.Image)
}

type View interface {
	Render(screen *ebiten.Image, p *robot.Pod)
}

type RenderViews []View

func DrawFrame(screen *ebiten.Image, title string, size float32, x float32, y float32, titleOffset float32) {
	framecolor := color.RGBA{64, 64, 64, 1}
	vector.StrokeRect(screen, x, y, size, size, 2, framecolor, false)
	ebitenutil.DebugPrintAt(screen, title, int(x+titleOffset), int(y)+int(titleOffset))
}

func DrawAxis(screen *ebiten.Image, horizontalLegend string, verticalLegend string, size float32, x float32, y float32, legendOffset float32) {
	// Axis
	vector.StrokeLine(screen, x+legendOffset, y+size/2-legendOffset, x+size-legendOffset, y+size/2-legendOffset, 1, color.White, false)
	vector.StrokeLine(screen, x+size-legendOffset, y+legendOffset, x+size-legendOffset, y+size-legendOffset, 1, color.White, false)

	// Legend
	ebitenutil.DebugPrintAt(screen, horizontalLegend, int(x+legendOffset), int(y+size/2-legendOffset-20))
	ebitenutil.DebugPrintAt(screen, verticalLegend, int(x+size-legendOffset-20), int(y+legendOffset))
}
