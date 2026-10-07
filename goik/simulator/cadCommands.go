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
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
)

// CAD export (see CAD_FILES.md and cad/goik_cad.py)

const CAD_FOLDER = "cad"

var CAD_OUTPUT_FOLDER = filepath.Join(CAD_FOLDER, "out")

// cadPython returns the Python of the CAD virtual environment, or "" if it is not installed
func cadPython() string {
	for _, p := range []string{
		filepath.Join(CAD_FOLDER, ".venv", "Scripts", "python.exe"),
		filepath.Join(CAD_FOLDER, ".venv", "bin", "python"),
	} {
		if _, err := os.Stat(p); err == nil {
			return p
		}
	}
	return ""
}

func (s *Shell) executeExportCadCmd(args []string) error {
	s.outputCh <- fmt.Sprintf("%+v", args)

	if len(args) != 2 {
		return fmt.Errorf("syntax error ('export_cad <name>'): %+v", args)
	}
	name := args[1]
	if strings.ContainsAny(name, `/\:`) || strings.HasPrefix(name, ".") {
		return fmt.Errorf("the name can not contain a path")
	}

	assembly, err := robot.NewCadAssembly(s.Pod.BodyDefinition, name)
	if err != nil {
		return err
	}
	if err := os.MkdirAll(CAD_OUTPUT_FOLDER, 0755); err != nil {
		return err
	}
	path := filepath.Join(CAD_OUTPUT_FOLDER, name+".json")
	if err := assembly.Save(path); err != nil {
		return err
	}
	s.outputCh <- fmt.Sprintf("Wrote %s (%s, rest pose)", path, assembly.String())

	python := cadPython()
	if python == "" {
		s.outputCh <- "CadQuery is not installed, so no STEP file was built. Install it with: python cad/tools.py setup"
		return nil
	}
	s.outputCh <- "Building the STEP assembly and STL with CadQuery ..."
	go s.buildCad(python, path)
	return nil
}

// buildCad runs the CadQuery script. It runs on its own goroutine (without the shell lock),
// since building a model takes a few seconds
func (s *Shell) buildCad(python string, path string) {
	output, err := exec.Command(python, filepath.Join(CAD_FOLDER, "goik_cad.py"), path).CombinedOutput()
	for _, line := range strings.Split(strings.TrimSpace(string(output)), "\n") {
		line = strings.TrimSpace(line)
		// CadQuery deprecation warnings are of no interest here
		if line == "" || strings.Contains(line, "Warning") || strings.HasPrefix(line, "warn(") {
			continue
		}
		s.outputCh <- "\t" + line
	}
	if err != nil {
		s.outputCh <- fmt.Sprintf("Building the CAD model failed: %v", err)
	}
}
