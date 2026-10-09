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
	"fmt"
	"strings"
)

type helpSection struct {
	name     string
	title    string
	commands []string
}

var helpSections = []helpSection{
	{"design", "Pod design", []string{
		"reset <0-7>                                - Load example pod <n> (6: AX-12A servos, 7: STS3215 servos)",
		"save <filename> / load <filename>          - Save / load the pod definition (including servo mapping)",
		"set_coxa_length <ALL | legNum> <mm>        - Segment lengths (with a design, a leg's mirror image changes with it)",
		"set_femur_length <ALL | legNum> <mm>",
		"set_tibia_length <ALL | legNum> <mm>",
		"set_coxa_angle <ALL | legNum> <angle>      - Rest angles (degrees)",
		"set_femur_angle <ALL | legNum> <angle>       (femur and tibia: only without a stance)",
		"set_tibia_angle <ALL | legNum> <angle>",
		"stance <height> [reach | vertical]         - Body height above the ground (mm). The femur and tibia rest",
		"                                             angles follow. Reach: foot distance from the femur joint (default:",
		"                                             tibia vertical). 'stance' shows it, with the possible heights",
		"stance reach <legNum> <reach | default>    - Reach for one leg (and its mirror image)",
		"stance off                                 - Set the femur and tibia rest angles by hand again",
		"effectors                                  - Output current end effector positions",
		"design                                     - Show the pod's design (symmetric about the Y axis)",
		"design import                              - Make a design from the current pod (if it is symmetric)",
		"design round <legs> <radius>               - New design: legs evenly spaced on a circle",
		"design rectangle <legs per side> <length> <width> - New design: rectangular body, legs along the sides",
		"design add <x> <y> <angle>                 - Add a leg on the +X side (mirrored) or on the axis (x = 0)",
		"design move <leg> <x> <y> [angle]          - Move a leg (and its mirror image). Also design angle <leg> <angle>",
		"design remove <leg>                        - Remove a leg and its mirror image",
		"design outline <auto | x y x y ...>        - Body outline, +X half from X = 0 to X = 0 (auto: through the legs)",
		"design off                                 - Drop the design, so legs can be changed one at a time",
	}},
	{"walk", "Walking", []string{
		"walk <x> <y> <yaw> [for <s> | cycles <n>]  - Walk with velocity x, y (mm/s) and yaw (deg/s).",
		"                                             Can be changed at any time, transitions are smooth",
		"halt                                       - Slow down and step back into the neutral stance",
		"gait <tripod | ripple | wave>              - Select gait (blends smoothly while walking)",
		"swing_time <seconds>                       - Duration of a leg swing (default 0.4)",
		"step_height <mm>                           - Height of the swing arc",
		"speed <1-10>                               - Simulation speed (10, the default, is real time)",
		"engine                                     - Show the gait engine state",
		"clear                                      - Clear the body path and foot trails in the XY view",
		"gamepad [on | off]                         - Drive with a gamepad (Xbox layout): show the mapping, or turn it on/off",
	}},
	{"pose", "Body pose (while standing or walking)", []string{
		"pitch <deg>                                - Positive raises the front (+Y)",
		"roll <deg>                                 - Positive lowers the +X side",
		"yaw <deg>                                  - Positive turns from +X towards +Y",
		"up <mm> / down <mm>                        - Body height relative to the neutral stance",
		"shift <x> <y>                              - Move the body (and centre of gravity) in mm",
		"level                                      - Return to the neutral pose",
	}},
	{"scripts", "Scripts", []string{
		"run <script>                               - Run a motion script from the scripts folder",
		"abort                                      - Abort the running script and halt",
		"scripts                                    - List scripts",
	}},
	{"servos", "Servos and CAD export", []string{
		"servos                                     - Show the servo mapping",
		"servo_model <AX-12A | STS3215 | XL-320>    - Select the servo type (default AX-12A)",
		"servo <ALL | legNum> <coxa|femur|tibia> id <n> | invert <on|off> | offset <deg> | limits <min> <max>",
		"                                             | case <deg> | axis_offset <mm>   (CAD: servo case rotation and horn offset)",
		"export_cad <name>                          - Export the pod in its rest pose to cad/out/<name>.json, and build",
		"                                             <name>.step and .stl with CadQuery (see docs/cad-export.md)",
	}},
}

func (s *Shell) executeHelpCmd(args []string) error {
	if len(args) == 2 {
		for _, section := range helpSections {
			if section.name == strings.ToLower(args[1]) {
				s.printHelpSection(section)
				return nil
			}
		}
	}
	if len(args) > 1 {
		var names []string
		for _, section := range helpSections {
			names = append(names, section.name)
		}
		return fmt.Errorf("unknown help section. Sections: %s", strings.Join(names, ", "))
	}

	for _, section := range helpSections {
		s.printHelpSection(section)
	}
	s.outputCh <- "Use 'help <section>' to show one section: design, walk, pose, scripts, servos"
	return nil
}

func (s *Shell) printHelpSection(section helpSection) {
	s.outputCh <- section.title + " (help " + section.name + "):"
	for _, c := range section.commands {
		s.outputCh <- "\t" + c
	}
}
