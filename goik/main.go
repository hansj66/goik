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

package main

import (
	"GOIK/simulator"
	"flag"
)

func main() {
	// go tool pprof cpu.pprof
	// defer profile.Start(profile.ProfilePath(".")).Stop()
	layout := flag.Bool("layout", true, "place the views window on the left 2/3 of the screen and the terminal on the right 1/3")
	flag.Parse()
	simulator.Run(*layout)
}
