# Convenience targets for GOIK. Everything here can also be run directly, which is the way to go
# on Windows without make: see the commands below and CAD_FILES.md.

PYTHON ?= python

.PHONY: build test run cad-setup cad-vendor cad-check

build:
	go build

test:
	go test ./...

run:
	go run .

# Install CadQuery in cad/.venv (needs Python 3.9 - 3.12)
cad-setup:
	$(PYTHON) cad/tools.py setup

# Download the vendor STEP files that can be downloaded automatically, and explain the others
cad-vendor:
	$(PYTHON) cad/tools.py vendor

# Show what is installed, downloaded and measured
cad-check:
	$(PYTHON) cad/tools.py check
