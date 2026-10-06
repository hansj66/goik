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

package simulator

import (
	"GOIK/robot"
	"GOIK/views"
	"fmt"
	"log"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
)

const window_size = 1024

type Game struct {
	views views.RenderViews
	xy    *views.XyView
	Shell *Shell
}

func NewGame(s *Shell) *Game {
	g := Game{Shell: s}

	g.views = append(g.views, views.NewXzView(0, 0, window_size/2))
	g.xy = views.NewXyView(window_size/2, 0, window_size/2)
	g.views = append(g.views, g.xy)
	g.views = append(g.views, views.NewIsoView(window_size/2, window_size/2, window_size/2))
	g.views = append(g.views, views.NewGaitView(0, window_size/2, window_size/2))

	return &g
}

var DELAY_COUNTER int = 10
var counter int = 0

func (g *Game) Update() error {
	// Close the window when the shell has quit
	select {
	case <-g.Shell.Done():
		return ebiten.Termination
	default:
	}

	g.Shell.mu.Lock()
	defer g.Shell.mu.Unlock()

	counter++
	if counter >= 3*DELAY_COUNTER {
		g.Shell.tickScript()
		g.Shell.Pod.Update()
		counter = 0
	}

	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
	g.Shell.mu.Lock()
	defer g.Shell.mu.Unlock()

	if g.Shell.clearTrails {
		g.xy.ClearTrails()
		g.Shell.clearTrails = false
	}

	for _, v := range g.views {
		v.Render(screen, g.Shell.Pod)
	}

	// Gait engine and script status (bottom of the gait view)
	if e := g.Shell.Pod.Engine; e != nil {
		state := "walking"
		if e.IsIdle() {
			state = "idle"
		} else if e.IsTransitioning() {
			state = "transitioning"
		}
		ebitenutil.DebugPrintAt(screen, fmt.Sprintf("Engine: %s %s, cycle %2.2f s, IK errors: %d", state, e.GetTwist().String(), e.CycleTime(), e.IKErrors), 20, window_size-60)
	}
	if g.Shell.Script != nil {
		ebitenutil.DebugPrintAt(screen, "Script: "+g.Shell.Script.Status(), 20, window_size-40)
	}
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (screenWidth, screenHeight int) {
	return 1024, 1024
}

func Run() {

	// Create the pod body and define a default gait
	pod := robot.NewPod(robot.NewExampleHexapod2())

	// Create the command shell
	shell := NewShell(pod)
	go shell.Run()
	g := NewGame(shell)

	// Create main window and start the simulation
	ebiten.SetWindowSize(window_size, window_size)
	ebiten.SetWindowTitle("Pod simulator")
	err := ebiten.RunGame(g)

	// The window was closed (or the shell quit): stop the chat UI, so the terminal is restored
	shell.Stop()
	if err != nil {
		log.Fatal(err)
	}
}
