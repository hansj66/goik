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

package robot

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestToRaw(t *testing.T) {
	ax := NewDefaultServoMapping(1)
	sts := NewDefaultServoMapping(1)
	sts.Model = "STS3215"

	tests := []struct {
		name    string
		mapping *ServoMapping
		joint   JointMapping
		angle   float64
		raw     uint16
		clamped bool
	}{
		{"centre", ax, JointMapping{}, 0, 512, false},
		{"positive", ax, JointMapping{}, 75, 768, false},
		{"inverted", ax, JointMapping{Inverted: true}, 75, 256, false},
		{"offset", ax, JointMapping{Offset: 75}, 0, 768, false},
		// Used to wrap around to 0xFF56
		{"below range", ax, JointMapping{}, -200, 0, true},
		{"above range", ax, JointMapping{}, 150, 1023, true},
		{"soft limit", ax, JointMapping{Min: -30, Max: 30}, 45, 614, true},
		{"sts centre", sts, JointMapping{}, 0, 2048, false},
		{"sts positive", sts, JointMapping{}, 90, 3072, false},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			raw, clamped := tt.mapping.ToRaw(tt.joint, tt.angle)
			if raw != tt.raw || clamped != tt.clamped {
				t.Errorf("ToRaw(%v) = %d, %t, want %d, %t", tt.angle, raw, clamped, tt.raw, tt.clamped)
			}
		})
	}
}

func TestTargetsUseReadmeAddressing(t *testing.T) {
	m := NewDefaultServoMapping(2)
	targets := m.Targets([]ServoAngles{{}, {Coxa: 75}})
	ids := []int{1, 2, 3, 4, 5, 6}
	for i, target := range targets {
		if target.Id != ids[i] {
			t.Errorf("target %d has id %d, want %d", i, target.Id, ids[i])
		}
	}
	if targets[3].Position != 768 {
		t.Errorf("leg 1 coxa = %d, want 768", targets[3].Position)
	}
}

func TestValidate(t *testing.T) {
	m := NewDefaultServoMapping(6)
	if err := m.Validate(6); err != nil {
		t.Fatal(err)
	}
	if err := m.Validate(5); err == nil {
		t.Error("expected an error for a mapping with the wrong number of legs")
	}

	m.Legs[1].Femur.Id = 1
	if err := m.Validate(6); err == nil || !strings.Contains(err.Error(), "already used") {
		t.Errorf("expected a duplicate id error, got %v", err)
	}

	m = NewDefaultServoMapping(6)
	m.Model = "SG90"
	if err := m.Validate(6); err == nil {
		t.Error("expected an error for an unknown servo model")
	}
}

func TestSaveLoadServoMapping(t *testing.T) {
	path := filepath.Join(t.TempDir(), "spider.json")

	// The spider is larger than the 1024 bytes the old loader could read
	b := NewSpider()
	b.ServoMapping().Model = "STS3215"
	b.Servos.Legs[2].Tibia.Inverted = true
	if err := b.Save(path); err != nil {
		t.Fatal(err)
	}

	loaded, err := b.Load(path)
	if err != nil {
		t.Fatal(err)
	}
	if loaded.Servos == nil || loaded.Servos.Model != "STS3215" || !loaded.Servos.Legs[2].Tibia.Inverted {
		t.Errorf("servo mapping was not restored: %+v", loaded.Servos)
	}
}

func TestLoadWithoutServoMapping(t *testing.T) {
	path := filepath.Join(t.TempDir(), "old.json")
	b := NewExampleHexapod1()
	if err := b.Save(path); err != nil {
		t.Fatal(err)
	}
	if data, _ := os.ReadFile(path); strings.Contains(string(data), "Servos") {
		t.Fatal("a pod without a mapping should be saved without one")
	}

	loaded, err := b.Load(path)
	if err != nil {
		t.Fatal(err)
	}
	if m := loaded.ServoMapping(); m.Model != DEFAULT_SERVO_MODEL || len(m.Legs) != 6 {
		t.Errorf("expected the default mapping, got %+v", m)
	}
}
