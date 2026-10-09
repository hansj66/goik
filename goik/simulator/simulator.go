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
const window_title = "Pod simulator"

type Game struct {
	views views.RenderViews
	xy    *views.XyView
	Shell *Shell
	// Places the views window and the terminal once the window is open (nil: leave them where they are)
	layout *windowLayout
	// Size of the screen the views are arranged on (see Layout)
	width, height int
}

func NewGame(s *Shell) *Game {
	g := Game{Shell: s}

	g.views = append(g.views, views.NewXzView(0, 0, window_size/2, window_size/2))
	g.xy = views.NewXyView(window_size/2, 0, window_size/2, window_size/2)
	g.views = append(g.views, g.xy)
	g.views = append(g.views, views.NewIsoView(window_size/2, window_size/2, window_size/2, window_size/2))
	g.views = append(g.views, views.NewGaitView(0, window_size/2, window_size/2, window_size/2))
	g.width, g.height = window_size, window_size

	return &g
}

// arrangeViews splits the screen in four: XZ (top left), XY (top right), isometric (bottom right) and gait
// (bottom left)
func (g *Game) arrangeViews(width int, height int) {
	w, h := float32(width)/2, float32(height)/2
	g.views[0].SetBounds(0, 0, w, h)
	g.views[1].SetBounds(w, 0, w, h)
	g.views[2].SetBounds(w, h, w, h)
	g.views[3].SetBounds(0, h, w, h)
	g.width, g.height = width, height
}

// The gait engine advances every 3 * DELAY_COUNTER frames. 0 (speed 10, the default) is every frame: about real time
var DELAY_COUNTER int = 0
var counter int = 0

func (g *Game) Update() error {
	// Close the window when the shell has quit
	select {
	case <-g.Shell.Done():
		return ebiten.Termination
	default:
	}

	if g.layout != nil {
		if note := g.layout.apply(window_title); note != "" {
			g.Shell.outputCh <- note
		}
		g.layout = nil
	}

	gamepad := readGamepad()

	g.Shell.mu.Lock()
	defer g.Shell.mu.Unlock()

	g.Shell.applyGamepad(gamepad)

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

	if b := screen.Bounds(); b.Dx() != g.width || b.Dy() != g.height {
		g.arrangeViews(b.Dx(), b.Dy())
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
		ebitenutil.DebugPrintAt(screen, fmt.Sprintf("Engine: %s %s, cycle %2.2f s, IK errors: %d", state, e.GetTwist().String(), e.CycleTime(), e.IKErrors), 20, g.height-60)
	}
	if g.Shell.Script != nil {
		ebitenutil.DebugPrintAt(screen, "Script: "+g.Shell.Script.Status(), 20, g.height-40)
	}
	ebitenutil.DebugPrintAt(screen, g.Shell.Gamepad.Status(), 20, g.height-80)
}

// Layout gives the views a screen with the window's shape, with its shorter side window_size pixels. The views draw
// at 1 pixel per mm, and Ebitengine scales the screen to the window, so the views fill the window.
func (g *Game) Layout(outsideWidth, outsideHeight int) (screenWidth, screenHeight int) {
	if outsideWidth <= 0 || outsideHeight <= 0 {
		return window_size, window_size
	}
	if outsideWidth >= outsideHeight {
		return window_size * outsideWidth / outsideHeight, window_size
	}
	return window_size, window_size * outsideHeight / outsideWidth
}

// Run starts the simulator. With layout, the views window fills the left 2/3 of the screen and the terminal
// running the shell the right 1/3 (the terminal only on Windows, see windowLayout_windows.go).
func Run(layout bool) {

	// Create the pod body and define a default gait
	pod := robot.NewPod(robot.NewExampleHexapod2())

	// Create the command shell
	shell := NewShell(pod)
	go shell.Run()
	g := NewGame(shell)
	if layout {
		// The terminal is the window in the foreground before the views window opens
		g.layout = newWindowLayout()
	}

	// Create main window and start the simulation. The views are scaled to the window's size
	ebiten.SetWindowSize(window_size, window_size)
	ebiten.SetWindowTitle(window_title)
	ebiten.SetWindowResizingMode(ebiten.WindowResizingModeEnabled)
	err := ebiten.RunGame(g)

	// The window was closed (or the shell quit): stop the chat UI, so the terminal is restored
	shell.Stop()
	if err != nil {
		log.Fatal(err)
	}
}
