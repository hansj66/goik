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

//go:build !windows

package simulator

import "github.com/hajimehoshi/ebiten/v2"

// Room (in device independent pixels) left for the window's title bar, and for menu bars and docks
const screenMargin = 80

// windowLayout places the views window on the left 2/3 of the screen. Moving the terminal is only supported
// on Windows (see windowLayout_windows.go).
type windowLayout struct{}

func newWindowLayout() *windowLayout {
	return &windowLayout{}
}

func (l *windowLayout) apply(title string) string {
	width, height := ebiten.ScreenSizeInFullscreen()
	if width <= 0 || height <= 0 {
		return "Window layout: the screen size is not available"
	}
	ebiten.SetWindowPosition(0, 0)
	ebiten.SetWindowSize(width*2/3, height-screenMargin)
	return "Window layout: the terminal is only placed automatically on Windows"
}
