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
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type XyView struct {
	x            float32
	y            float32
	size         float32
	legendOffset float32
	// Recent foot positions per leg
	trails [][]robot.Coordinate
	// The body's path in world coordinates, and the gait engine it was recorded from
	path       [][2]float64
	pathEngine *robot.GaitEngine
}

func NewXyView(x float32, y float32, size float32) *XyView {
	return &XyView{x: x, y: y, size: size, legendOffset: 10}
}

func (v *XyView) TranslateX(x float64) int {
	return int(v.x) + int(v.size)/2 - int(v.legendOffset) + int(x)
}

func (v *XyView) TranslateY(y float64) int {
	return int(v.y) + int(v.size)/2 - int(v.legendOffset) + int(y)
}

// Number of positions kept per foot for the trail
const TRAIL_LENGTH = 120

// Number of positions kept for the body's path, and the minimum distance (mm) between them
const PATH_LENGTH = 3000
const PATH_RESOLUTION = 2.0

func (v *XyView) Render(screen *ebiten.Image, p *robot.Pod) {

	DrawFrame(screen, "XY View ", v.size, v.x, v.y, v.legendOffset)
	joints := p.GroundJoints()
	n := p.BodyDefinition.NumLegs

	// Ground grid and the body's path. They are fixed in the world, so they move as the pod walks
	clip := Clip(screen, v.x, v.y, v.size)
	var gridLines [][][2]float32
	for _, l := range GroundGrid(p) {
		gridLines = append(gridLines, [][2]float32{
			{float32(v.TranslateX(l[0][0])), float32(v.TranslateY(l[0][1]))},
			{float32(v.TranslateX(l[1][0])), float32(v.TranslateY(l[1][1]))},
		})
	}
	StrokePaths(clip, gridLines, 1, GridClr())

	v.updatePath(p.Engine)
	if p.Engine != nil && len(v.path) > 1 {
		var path [][2]float32
		for _, w := range v.path {
			x, y := p.Engine.WorldToGround(w[0], w[1])
			path = append(path, [2]float32{float32(v.TranslateX(x)), float32(v.TranslateY(y))})
		}
		StrokePaths(clip, [][][2]float32{path}, 2, PathClr())
	}

	// Neutral foot positions
	for _, l := range p.Legs {
		c := l.NeutralEffectorCoordinate
		vector.StrokeCircle(screen, float32(v.TranslateX(c.X)), float32(v.TranslateY(c.Y)), 4, 1, NeutralClr(), true)
	}

	// Foot trails (ground frame), drawn as one path
	v.updateTrails(joints)
	polylines := make([][][2]float32, len(v.trails))
	for l, trail := range v.trails {
		for _, c := range trail {
			polylines[l] = append(polylines[l], [2]float32{float32(v.TranslateX(c.X)), float32(v.TranslateY(c.Y))})
		}
	}
	StrokePaths(screen, polylines, 1, TrailClr())

	// Where swinging feet will land
	if p.Engine != nil {
		for l := range p.Legs {
			if t, swinging := p.Engine.LandingTarget(l); swinging {
				vector.StrokeCircle(screen, float32(v.TranslateX(t.X)), float32(v.TranslateY(t.Y)), 8, 2, Blue(), true)
			}
		}
	}

	// Draw body frame
	for l := 0; l < n; l++ {
		next := (l + 1) % n
		vector.StrokeLine(screen,
			float32(v.TranslateX(joints[l][0].X)),
			float32(v.TranslateY(joints[l][0].Y)),
			float32(v.TranslateX(joints[next][0].X)),
			float32(v.TranslateY(joints[next][0].Y)),
			5,
			White(),
			true)
	}

	// Draw Coxa, Femur and Tibia
	for j := 0; j < robot.NUM_JOINTS-1; j++ {
		for l := 0; l < n; l++ {
			col := White()
			width := 3
			if p.IsSwingPhase(l) {
				col = Blue()
			}
			vector.StrokeLine(screen,
				float32(v.TranslateX(joints[l][j].X)),
				float32(v.TranslateY(joints[l][j].Y)),
				float32(v.TranslateX(joints[l][j+1].X)),
				float32(v.TranslateY(joints[l][j+1].Y)),
				float32(width),
				col,
				true)
		}
	}

	// Draw joints
	for _, l := range joints {
		for _, j := range l {
			vector.DrawFilledCircle(screen, float32(v.TranslateX(j.X)), float32(v.TranslateY(j.Y)), 5, Red(), true)
		}
	}

	// Annotate legs with lex indexes
	for l := 0; l < n; l++ {
		ebitenutil.DebugPrintAt(screen, fmt.Sprintf("%d", l), v.TranslateX(joints[l][3].X+15), v.TranslateY(joints[l][3].Y+5))
	}
}

// ClearTrails forgets the body's path and the foot trails
func (v *XyView) ClearTrails() {
	v.path = nil
	v.trails = nil
}

// updatePath records the body's position in the world. The path restarts when the engine is recreated.
func (v *XyView) updatePath(e *robot.GaitEngine) {
	if e != v.pathEngine {
		v.path = nil
		v.pathEngine = e
	}
	if e == nil {
		return
	}
	x, y, _ := e.Odometry()
	if len(v.path) > 0 {
		last := v.path[len(v.path)-1]
		if math.Hypot(x-last[0], y-last[1]) < PATH_RESOLUTION {
			return
		}
	}
	v.path = append(v.path, [2]float64{x, y})
	if len(v.path) > PATH_LENGTH {
		v.path = v.path[len(v.path)-PATH_LENGTH:]
	}
}

// updateTrails records the foot positions, skipping positions that haven't moved
func (v *XyView) updateTrails(joints [][robot.NUM_JOINTS]robot.Coordinate) {
	if len(v.trails) != len(joints) {
		v.trails = make([][]robot.Coordinate, len(joints))
	}
	for l := range joints {
		foot := joints[l][robot.EFFECTOR_ORIGIN_INDEX]
		trail := v.trails[l]
		if len(trail) > 0 {
			last := trail[len(trail)-1]
			if math.Hypot(foot.X-last.X, foot.Y-last.Y) < 0.5 {
				continue
			}
		}
		trail = append(trail, foot)
		if len(trail) > TRAIL_LENGTH {
			trail = trail[len(trail)-TRAIL_LENGTH:]
		}
		v.trails[l] = trail
	}
}
