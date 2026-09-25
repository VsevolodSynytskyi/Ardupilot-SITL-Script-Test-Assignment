# ArduCopter SITL: thrust-only altitude PID

A C++ program that flies an ArduCopter (SITL) through a simple altitude mission, controlling it **only through thrust**. It arms the copter, switches to GUIDED, climbs to 10 m, holds, descends to 5 m, holds, and lands. Altitude is controlled by our own cascaded PID: the program streams `SET_ATTITUDE_TARGET` messages with a level attitude and a thrust value, and ArduPilot (with `GUID_OPTIONS=8`) applies that thrust directly.

## How it works

```
 LOCAL_POSITION_NED (50 Hz)                                    SET_ATTITUDE_TARGET (50 Hz)
 altitude, climb rate ──► AltitudeController ──► thrust ──► level attitude, held yaw, thrust
                          │
                          ├─ setpoint trajectory   trapezoidal: rate / acceleration / braking limited
                          ├─ outer P  (altitude)    climb-rate setpoint = trajectory speed + kp·error
                          └─ inner PID (climb rate) thrust = MOT_THST_HOVER + correction
```

| Component | Role |
|---|---|
| `DroneInterface` | MAVSDK connection: telemetry, parameters, mode changes, arming, thrust commands. Owns the NED → altitude conversion. |
| `AltitudeController` | Cascade controller above, plus the take-off phase. |
| `PIDController` | PID with derivative on measurement, low-pass filtered D, clamping anti-windup, output limits. |
| `MissionRunner` | Mission state machine and the fixed-rate control loop, with safety checks. |
| `DataLogger` | CSV log of every control tick. |

The mission states are:

```
INIT → SET_PARAMS → SET_GUIDED → ARM → CLIMB_TO_HIGH → HOLD_HIGH → DESCEND_TO_LOW → HOLD_LOW → LAND → DONE
                                            any failure → ERROR → LAND
```

A target counts as reached when the altitude error stays below 0.25 m for 2 s. Each altitude is then held for 10 s.

Safety checks run on every tick. Any of these switches the vehicle to LAND:
- telemetry older than 0.2 s
- a control-loop stall, or thrust commands failing to send, for more than 0.5 s
- an unexpected disarm
- altitude above 15 m
- a climb or descent taking longer than 60 s
- Ctrl+C (a second Ctrl+C quits immediately)

The LAND command is retried for 30 s, so it still gets through after a short link loss. Meanwhile ArduPilot holds altitude, because the thrust stream has stopped.

Two cases where the controller deliberately does not take control:
- **External mode change** (an operator or ground station switched mode): the controller stops streaming and exits with code 3, leaving the vehicle to the operator.
- **Unsafe start:** it refuses to start on an armed vehicle, and it won't arm until `LOCAL_POSITION_NED` arrives at 25 Hz or more.

Exit codes: 0 mission complete, 1 error (landed), 3 control released to the operator, 130 interrupted.

## Requirements

- Xcode Command Line Tools and Homebrew (macOS)
- Python 3.10+
- CMake 3.20+

The project was developed on macOS against a native SITL build. Linux should work with the equivalent packages.

## Setup

```bash
brew install cmake

# ArduPilot source (not tracked in this repo)
git clone --recurse-submodules --shallow-submodules --depth 1 \
    https://github.com/ArduPilot/ardupilot.git ardupilot

# Python environment: SITL build tooling, MAVProxy, pymavlink, matplotlib
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# SITL copter build (~2 min)
source .venv/bin/activate
cd ardupilot && ./waf configure --board sitl && ./waf copter && cd ..

# MAVSDK v4.0.0 C++ library, built into third_party/mavsdk-install (~10 min)
scripts/build_mavsdk.sh

# Controller and unit tests (GoogleTest is fetched by CMake)
cmake -S . -B build && cmake --build build -j
(cd build && ctest)
```

ArduPilot's `install-prereqs-mac.sh` isn't needed: the packages in `requirements.txt` are enough for the SITL build.

## Running

Start SITL. It runs the `arducopter` binary plus a MAVProxy instance that routes MAVLink to two UDP ports:

```bash
scripts/run_sitl.sh --wipe        # --wipe resets parameters to defaults + sitl/guided_thrust.parm
```

| Endpoint | Use |
|---|---|
| `udp:127.0.0.1:14550` | ground station (QGroundControl connects automatically) or `mavproxy.py --master udp:127.0.0.1:14550` |
| `udp:127.0.0.1:14551` | the controller and test scripts |

`sitl/guided_thrust.parm` sets `GUID_OPTIONS 8` and `GUID_TIMEOUT 1`. Give SITL about 20 s after start: `LOCAL_POSITION_NED` is only sent once the EKF origin is set from GPS.

Run the mission:

```bash
./build/altitude_control                       # full mission, config/mission.conf, log in logs/flight.csv
./build/altitude_control --set vel_kp=0.5      # override any config value
./build/altitude_control --run telemetry       # print altitude and mode, check the telemetry rate
./build/altitude_control --run open-loop       # GUIDED, arm, fixed thrust up to 4 m, LAND
```

Plot and evaluate a flight:

```bash
source .venv/bin/activate
python scripts/plot.py logs/flight.csv            # logs/flight.png: altitude, climb rate, thrust
python scripts/flight_metrics.py logs/flight.csv  # overshoot, settling time, hold error
python scripts/tune.py "vel_kp=0.5" "vel_kp=0.7"  # one mission per gain set, comparison table
```

Fault-injection tests (SITL running):

```bash
python scripts/fault_tests.py   # early_interrupt, armed_start, mode_change, stall, link_loss
```

`link_loss` freezes the MAVProxy router for 3 s, and the router's stream stays bursty for a while afterwards. The controller then refuses to fly or lands early, so restart `scripts/run_sitl.sh` before the next flight.

## Design notes

These come from experiments in SITL (`scripts/risk_tests.py`) and from reading the ArduCopter source (`ModeGuided::angle_control_run`, `GCS_MAVLINK_Copter::handle_message_set_attitude_target`).

- **Altitude:** `alt = -LOCAL_POSITION_NED.z`, `climb = -vz` (NED is z-down).
- **`SET_ATTITUDE_TARGET`:** `type_mask = 7` ignores all body rates. ArduPilot rejects a mask that ignores only some of them. The quaternion is level, with the yaw captured at arming and held.
- **Take-off with thrust only works.** Positive thrust releases ArduPilot's landed state. Liftoff comes about 3 s after arming (motor spool-up), and must happen within `DISARM_DELAY` (10 s).
- **Take-off phase:** until the copter is 0.3 m above its arming altitude, thrust is fixed at `MOT_THST_HOVER + 0.10`, with the PID held in reset so nothing winds up on the ground. The cascade then takes over without a jump: the trajectory starts at the current altitude and climb rate.
- **Hover thrust:** `MOT_THST_HOVER` (0.36–0.39, re-learned in flight) is only a feedforward. Thrust is very sensitive (about 25 m/s² per unit), so the climb-rate loop needs an integrator.
- **Command stream gaps are dangerous:** until `GUID_TIMEOUT` expires, ArduPilot keeps applying the last thrust it received. After the timeout it levels out and holds altitude. The controller therefore sends a freshly computed thrust every tick from a single 50 Hz loop (no separate sender repeating stale values), and lands if the loop stalls for more than 0.5 s. `GUID_TIMEOUT 1` is a backstop on the autopilot side.
- **Setpoint trajectory:** a plain rate-limited ramp overshot by about 0.5 m, because its speed feedforward drops to zero in one tick. The trapezoidal profile brakes early enough to arrive with zero speed.
- **MAVProxy** runs with `--streamrate=-1`. Otherwise it keeps re-requesting 4 Hz stream rates, overriding the 50 Hz the controller asks for.
- **MAVSDK v4:** `MavlinkPassthrough` is deprecated, so `SET_ATTITUDE_TARGET` and `DO_SET_MODE` go through its replacement, `MavlinkDirect`. Telemetry, Param and Action cover the rest. `Action::land()` sends `MAV_CMD_NAV_LAND`, which switches ArduCopter to LAND.

## Tuning and results

Tuned in SITL with `scripts/tune.py`, inner loop first, then outer:

| Gain | Value | Notes |
|---|---|---|
| `vel_kp` | 0.7 | Thrust chatter (a limit cycle) starts around 2.0, so this is about 3× below it. |
| `vel_ki` | 0.15 | Best hold error without extra overshoot. |
| `vel_kd` | 0 | D increased overshoot. Thrust sets acceleration, so the climb-rate loop is essentially first order. |
| `alt_kp` | 2.0 | Overshoot fell from 0.06 m at 0.7 to 0.02 m at 2.0. |

The setpoint trajectory is limited to 1.5 m/s up, 1.0 m/s down and 0.7 m/s² acceleration.

| | Climb to 10 m | Descend to 5 m |
|---|---|---|
| Overshoot | 0.024 m | 0.026 m |
| Settling time (±0.10 m) | 10.1 s after arming (incl. ~3 s spool-up) | 5.8 s |
| Hold error, RMS / max | 0.001 / 0.004 m | 0.002 / 0.004 m |

## Project layout

```
CMakeLists.txt              C++ build (finds MAVSDK in third_party/mavsdk-install)
config/mission.conf         mission and controller settings
include/altctl/, src/       controller
tests/                      unit tests: PID, and the cascade on a vertical-dynamics model
requirements.txt            Python dependencies
scripts/run_sitl.sh         SITL + MAVProxy router
scripts/build_mavsdk.sh     local MAVSDK build
scripts/plot.py             flight plot
scripts/flight_metrics.py   step-response metrics from a flight log
scripts/tune.py             automated gain sweeps in SITL
scripts/fault_tests.py      fault-injection tests in SITL
scripts/risk_tests.py       SITL experiments behind the design notes
scripts/sitl_smoke_test.py  quick SITL check: GUIDED take-off to 5 m and land
sitl/guided_thrust.parm     SITL parameters; sitl/run/ holds runtime state
```

## Known issues: paths with spaces

If the project path contains a space:

- **`sim_vehicle.py`** crashes SITL (`PANIC: Failed to load defaults`), because it splits the `--defaults` argument. `scripts/run_sitl.sh` runs the `arducopter` binary directly with relative paths to avoid it.
- **venv scripts** (`mavproxy.py` etc.) fail with `bad interpreter`. Fix the shebangs after each `pip install`:
  ```bash
  sed -i '' '1s|^#!.*/\.venv/bin/python.*|#!/usr/bin/env python3|' .venv/bin/*
  ```
