# ArduCopter SITL: thrust-only altitude PID

A controller that flies an ArduCopter SITL drone using **thrust-only** `SET_ATTITUDE_TARGET` commands (`GUID_OPTIONS=8`). Mission: arm → GUIDED → climb to 10 m → hold → descend to 5 m → hold → LAND, with PID altitude control.

**Status:** the SITL environment works. The controller is in progress.

## Requirements

- macOS (tested on 26.5, Apple Silicon) with Xcode Command Line Tools and Homebrew
- Python 3.10 (any install; pyenv used here)
- `cmake` (`brew install cmake`), for the C++ controller
- Optional: QGroundControl, to watch the flight

Nothing else is installed system-wide. ArduPilot and all Python packages live inside the project directory.

## Setup

```bash
brew install cmake

# ArduPilot source (ignored by git)
git clone --recurse-submodules --shallow-submodules --depth 1 \
    https://github.com/ArduPilot/ardupilot.git ardupilot

# Project-local Python environment
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# Build SITL (~2 min)
source .venv/bin/activate
cd ardupilot && ./waf configure --board sitl && ./waf copter && cd ..
```

This skips ArduPilot's `install-prereqs-mac.sh`, which installs globally and edits `~/.zshrc`. The SITL build only needs the packages in `requirements.txt`. The ARM toolchain, wxPython and ccache are not required.

## Run

```bash
scripts/run_sitl.sh --wipe        # SITL + MAVLink router; --wipe resets params
```

| Endpoint | Use |
|---|---|
| `udp:127.0.0.1:14550` | GCS (QGroundControl auto-connects) or `mavproxy.py --master udp:127.0.0.1:14550` |
| `udp:127.0.0.1:14551` | our controller / test scripts |

`sitl/guided_thrust.parm` sets `GUID_OPTIONS 8` on startup. Runtime state (EEPROM, tlogs) goes to `sitl/run/`.

Smoke test (GUIDED takeoff to 5 m, then LAND):

```bash
source .venv/bin/activate && python scripts/sitl_smoke_test.py
```

## Known issues: paths with spaces

If the project path contains a space:

- **`sim_vehicle.py`** crashes SITL (`PANIC: Failed to load defaults`) because it splits the `--defaults` argument. `scripts/run_sitl.sh` avoids this by running the `arducopter` binary directly with relative paths.
- **venv CLI scripts** (`mavproxy.py` etc.) fail with `bad interpreter`. Fix after each `pip install`:
  ```bash
  sed -i '' '1s|^#!.*/\.venv/bin/python.*|#!/usr/bin/env python3|' .venv/bin/*
  ```

## Layout

```
requirements.txt      Python dependencies (build tooling, MAVProxy, pymavlink)
scripts/run_sitl.sh   SITL + MAVProxy router launcher
scripts/              test and helper scripts
sitl/                 SITL parameter file; sitl/run/ = runtime state (ignored)
```
