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

//go:build windows

package simulator

import (
	"os"
	"syscall"
	"unsafe"
)

/*
	Window layout on Windows: the views window fills the left 2/3 of the screen's work area (the screen without the
	taskbar), and the terminal running the shell the right 1/3. Both windows are placed with the Win32 API, so they
	use the same (physical pixel) coordinates.

	The layout is applied from Ebitengine's Update, which doesn't run on the thread that owns the views window, while
	that thread waits for Update. So windows are moved asynchronously (SWP_ASYNCWINDOWPOS, ShowWindowAsync), since a
	synchronous call would wait for the owning thread and deadlock.

	The terminal is the window in the foreground when GOIK starts. It is only moved if it is a terminal window
	(Windows Terminal or the classic console), not if GOIK runs in an editor's terminal, like VS Code's.
*/

var (
	user32                       = syscall.NewLazyDLL("user32.dll")
	dwmapi                       = syscall.NewLazyDLL("dwmapi.dll")
	procGetForegroundWindow      = user32.NewProc("GetForegroundWindow")
	procSetForegroundWindow      = user32.NewProc("SetForegroundWindow")
	procGetClassNameW            = user32.NewProc("GetClassNameW")
	procFindWindowW              = user32.NewProc("FindWindowW")
	procGetWindowThreadProcessId = user32.NewProc("GetWindowThreadProcessId")
	procMonitorFromWindow        = user32.NewProc("MonitorFromWindow")
	procGetMonitorInfoW          = user32.NewProc("GetMonitorInfoW")
	procGetWindowRect            = user32.NewProc("GetWindowRect")
	procSetWindowPos             = user32.NewProc("SetWindowPos")
	procShowWindowAsync          = user32.NewProc("ShowWindowAsync")
	procIsZoomed                 = user32.NewProc("IsZoomed")
	procIsIconic                 = user32.NewProc("IsIconic")
	procDwmGetWindowAttribute    = dwmapi.NewProc("DwmGetWindowAttribute")
)

const (
	monitorDefaultToNearest   = 2
	swRestore                 = 9
	swpNoZOrder               = 0x0004
	swpNoActivate             = 0x0010
	swpAsyncWindowPos         = 0x4000
	dwmwaExtendedFrameBounds  = 9
	windowsTerminalClass      = "CASCADIA_HOSTING_WINDOW_CLASS"
	classicConsoleWindowClass = "ConsoleWindowClass"
)

type rect struct {
	Left, Top, Right, Bottom int32
}

type monitorInfo struct {
	Size    uint32
	Monitor rect
	Work    rect
	Flags   uint32
}

// windowLayout places the views window and the terminal side by side
type windowLayout struct {
	// The terminal window, or 0 if it is not to be moved
	terminal uintptr
	// Why the terminal is not moved (empty if it is)
	note string
}

// newWindowLayout remembers the terminal window. Call it before the views window opens.
func newWindowLayout() *windowLayout {
	hwnd, _, _ := procGetForegroundWindow.Call()
	if hwnd == 0 {
		return &windowLayout{note: "Window layout: no terminal window found, only the views window is placed"}
	}
	switch className(hwnd) {
	case windowsTerminalClass, classicConsoleWindowClass:
		return &windowLayout{terminal: hwnd}
	}
	return &windowLayout{note: "Window layout: GOIK isn't running in a terminal window (an editor's terminal?), so only the views window is placed"}
}

// apply places the windows. Call it once the views window (titled title) is open.
func (l *windowLayout) apply(title string) string {
	views := findOwnWindow(title)
	if views == 0 {
		return "Window layout: the views window was not found"
	}

	// The screen the terminal is on (or the views window, without a terminal)
	reference := l.terminal
	if reference == 0 {
		reference = views
	}
	monitor, _, _ := procMonitorFromWindow.Call(reference, monitorDefaultToNearest)
	info := monitorInfo{Size: uint32(unsafe.Sizeof(monitorInfo{}))}
	if ok, _, _ := procGetMonitorInfoW.Call(monitor, uintptr(unsafe.Pointer(&info))); ok == 0 {
		return "Window layout: the screen size is not available"
	}
	work := info.Work
	width, height := work.Right-work.Left, work.Bottom-work.Top
	split := work.Left + width*2/3

	place(views, rect{work.Left, work.Top, split, work.Bottom})
	if l.terminal != 0 {
		place(l.terminal, rect{split, work.Top, work.Left + width, work.Top + height})
		// Back to the shell, so commands can be typed right away
		procSetForegroundWindow.Call(l.terminal)
	}
	return l.note
}

// place moves and resizes a window so that its visible frame covers r. Windows have invisible borders (for resizing)
// outside the visible frame, which are added to r.
func place(hwnd uintptr, r rect) {
	zoomed, _, _ := procIsZoomed.Call(hwnd)
	iconic, _, _ := procIsIconic.Call(hwnd)
	if zoomed != 0 || iconic != 0 {
		procShowWindowAsync.Call(hwnd, swRestore)
	}

	var outer, visible rect
	procGetWindowRect.Call(hwnd, uintptr(unsafe.Pointer(&outer)))
	if res, _, _ := procDwmGetWindowAttribute.Call(hwnd, dwmwaExtendedFrameBounds, uintptr(unsafe.Pointer(&visible)), unsafe.Sizeof(visible)); res != 0 {
		visible = outer
	}
	left, top := visible.Left-outer.Left, visible.Top-outer.Top
	right, bottom := outer.Right-visible.Right, outer.Bottom-visible.Bottom

	x, y := r.Left-left, r.Top-top
	w, h := r.Right-r.Left+left+right, r.Bottom-r.Top+top+bottom
	procSetWindowPos.Call(hwnd, 0, uintptr(x), uintptr(y), uintptr(w), uintptr(h), swpNoZOrder|swpNoActivate|swpAsyncWindowPos)
}

// findOwnWindow returns this process's top level window with the given title, or 0
func findOwnWindow(title string) uintptr {
	t, err := syscall.UTF16PtrFromString(title)
	if err != nil {
		return 0
	}
	hwnd, _, _ := procFindWindowW.Call(0, uintptr(unsafe.Pointer(t)))
	if hwnd == 0 {
		return 0
	}
	var pid uint32
	procGetWindowThreadProcessId.Call(hwnd, uintptr(unsafe.Pointer(&pid)))
	if int(pid) != os.Getpid() {
		return 0
	}
	return hwnd
}

// className returns the window class name of a window
func className(hwnd uintptr) string {
	buf := make([]uint16, 256)
	n, _, _ := procGetClassNameW.Call(hwnd, uintptr(unsafe.Pointer(&buf[0])), uintptr(len(buf)))
	return syscall.UTF16ToString(buf[:n])
}
