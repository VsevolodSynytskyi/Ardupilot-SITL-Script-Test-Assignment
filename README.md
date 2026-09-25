# ArduCopter SITL: thrust-only altitude PID

A controller that flies an ArduCopter SITL drone using **thrust-only** `SET_ATTITUDE_TARGET` commands (`GUID_OPTIONS=8`). Mission: arm → GUIDED → climb to 10 m → hold → descend to 5 m → hold → LAND, with PID altitude control.

**Status:** SITL environment and risk validation done; C++ controller in progress.

## Requirements

- macOS (tested on 26.5, Apple Silicon) with Xcode Command Line Tools and Homebrew
- Python 3.10 (any install; pyenv used here)
- `cmake` (`brew install cmake`), for the C++ controller and MAVSDK
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

# MAVSDK v4.0.0 C++ library, built into third_party/mavsdk-install (~10 min, ignored by git)
scripts/build_mavsdk.sh

# Controller + unit tests (GoogleTest is fetched by CMake)
cmake -S . -B build && cmake --build build -j
(cd build && ctest)
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

Controller (with SITL running):

```bash
./build/altitude_control                       # full mission, uses config/mission.conf
./build/altitude_control --set vel_kp=0.06     # override single values
./build/altitude_control --run telemetry       # bring-up: print altitude/mode, check 50 Hz rate
./build/altitude_control --run open-loop       # bring-up: GUIDED, arm, fixed thrust to 4 m, LAND
```

Wait ~20 s after starting SITL: `LOCAL_POSITION_NED` is only sent once the EKF origin is set from GPS.

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
CMakeLists.txt            C++ build (finds MAVSDK in third_party/mavsdk-install)
config/mission.conf       mission and controller settings
include/altctl/, src/     C++ controller
tests/                    unit tests (PID, cascade on a vertical-dynamics model)
requirements.txt          Python dependencies (build tooling, MAVProxy, pymavlink)
scripts/run_sitl.sh       SITL + MAVProxy router launcher
scripts/build_mavsdk.sh   builds MAVSDK locally
scripts/risk_tests.py     SITL experiments behind the design decisions (see "Design notes")
scripts/sitl_smoke_test.py  environment check
sitl/guided_thrust.parm   SITL parameters (GUID_OPTIONS, GUID_TIMEOUT); sitl/run/ = runtime state (ignored)
```

## Design notes

These come from SITL experiments (`scripts/risk_tests.py`) and the ArduCopter source (`ModeGuided::angle_control_run`).

- **Altitude:** `alt = -LOCAL_POSITION_NED.z`, `climb = -vz` (NED is z-down).
- **`SET_ATTITUDE_TARGET`:** `type_mask = 7` (ignore all body rates; ArduPilot rejects a partial mask). The quaternion is level, with the yaw captured at the start and held.
- **Takeoff:** thrust-only works from the ground. Positive thrust releases ArduPilot's landed state. Liftoff comes ~3 s after arming (motor spool-up) and must happen within `DISARM_DELAY` (10 s).
- **Hover thrust:** `MOT_THST_HOVER` (~0.36–0.39, re-learned in flight) is only a feedforward. Thrust is very sensitive (≈25 m/s² per unit), so the velocity loop needs an integrator.
- **Stream gaps:** until `GUID_TIMEOUT` expires, ArduPilot keeps applying the **last thrust**, so a stall is dangerous. After the timeout it levels out and holds altitude. We send at 50 Hz, switch to LAND if our loop stalls > 0.5 s, and set `GUID_TIMEOUT 1` as an autopilot-side backstop.
- **Controller:** a cascade. A trapezoidal setpoint trajectory (rate, acceleration and braking limited) feeds an outer P loop on altitude, with the trajectory velocity as feedforward. That gives a climb-rate setpoint for an inner PID on climb rate, whose output is a correction added to `MOT_THST_HOVER`. The PID uses derivative on measurement, a low-pass filtered D, clamping anti-windup, and an integrator frozen below 0.3 m (on the ground). A plain rate-limited ramp overshot ~0.5 m on the model, because the feedforward stops abruptly.
- **MAVProxy router:** runs with `--streamrate=-1`. Otherwise it keeps re-requesting 4 Hz stream rates and overrides the 50 Hz the controller asks for.
- **MAVSDK v4:** `MavlinkPassthrough` is deprecated. `SET_ATTITUDE_TARGET` and `DO_SET_MODE` go through its replacement, `MavlinkDirect`.
