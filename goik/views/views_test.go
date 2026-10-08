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
	"testing"

	"github.com/hajimehoshi/ebiten/v2"
)

// BenchmarkRender measures the CPU time each view needs per frame (go test -bench . ./views).
// The shell lock is held while drawing, so slow views make the command shell sluggish.
func BenchmarkRender(b *testing.B) {
	screen := ebiten.NewImage(1024, 1024)
	p := robot.NewPod(robot.NewExampleHexapod1())
	e, err := robot.NewGaitEngine(p)
	if err != nil {
		b.Fatal(err)
	}
	p.Engine = e
	e.SetTwist(robot.Twist{Y: 60, Yaw: 10})

	xy := NewXyView(512, 0, 512, 512)
	// Walk for a while to fill the foot trails
	for i := 0; i < 600; i++ {
		p.Update()
		xy.Render(screen, p)
	}

	for _, v := range []struct {
		name string
		view View
	}{
		{"xz", NewXzView(0, 0, 512, 512)},
		{"xy", xy},
		{"iso", NewIsoView(512, 512, 512, 512)},
		{"gait", NewGaitView(0, 512, 512, 512)},
	} {
		b.Run(v.name, func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				v.view.Render(screen, p)
			}
		})
	}
}
