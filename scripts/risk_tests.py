#!/usr/bin/env python3
"""Stage 2 risk validation against ArduCopter SITL (throwaway test code, pymavlink).

Tests, in order:
  zsign     - NED z/vz sign vs GLOBAL_POSITION_INT.relative_alt during a standard takeoff to 5 m
  typemask  - in the air: SET_ATTITUDE_TARGET thrust steps around MOT_THST_HOVER; attitude stays level
  takeoff   - from the ground: GUIDED, arm, stream thrust > hover only (no NAV_TAKEOFF)
  timeout   - in the air: pause the stream > GUID_TIMEOUT, observe, resume, check control returns

Usage: python scripts/risk_tests.py [zsign typemask takeoff timeout] [--url udp:127.0.0.1:14551]
Each test ends with LAND + disarm. Telemetry is written to logs/risk_<test>.csv.
"""
import argparse
import csv
import math
import os
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
TYPE_MASK = (M.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
             | M.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
             | M.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE)  # = 7
LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "logs")


def altitude_from_ned(z):
    """The single NED -> altitude conversion (task 8): altitude up = -z_down."""
    return -z


def level_quat(yaw):
    """Quaternion (w, x, y, z) for roll = pitch = 0 and the given yaw."""
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


class Link:
    def __init__(self, url):
        self.m = mavutil.mavlink_connection(url, source_system=250)
        self.m.wait_heartbeat(timeout=30)
        self.t0 = time.time()
        self.st = dict(z=float("nan"), vz=float("nan"), rel_alt=float("nan"),
                       roll=0.0, pitch=0.0, yaw=0.0, armed=False, mode="?",
                       landed=None)
        for msg_id in (M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, M.MAVLINK_MSG_ID_ATTITUDE,
                       M.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE):
            self.command(M.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 20000)  # 50 Hz
        self.rows = []

    def command(self, cmd, *params):
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)

    def param(self, name):
        self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component,
                                           name.encode(), -1)
        end = time.time() + 5
        while time.time() < end:
            p = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if p and p.param_id == name:
                return p.param_value
        raise RuntimeError(f"no PARAM_VALUE for {name}")

    def pump(self, timeout=0.0):
        """Drain incoming messages and update the state snapshot."""
        end = time.time() + timeout
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                if time.time() >= end:
                    return
                time.sleep(0.002)
                continue
            t = msg.get_type()
            if t == "LOCAL_POSITION_NED":
                self.st["z"], self.st["vz"] = msg.z, msg.vz
            elif t == "GLOBAL_POSITION_INT":
                self.st["rel_alt"] = msg.relative_alt / 1000.0
            elif t == "ATTITUDE":
                self.st.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw)
            elif t == "EXTENDED_SYS_STATE":
                self.st["landed"] = msg.landed_state
            elif t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system and msg.type != M.MAV_TYPE_GCS:
                self.st["armed"] = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                self.st["mode"] = self.m.flightmode
            elif t == "STATUSTEXT":
                print(f"    [AP] {msg.text}")

    def log(self, phase, thrust):
        s = self.st
        self.rows.append(dict(t=round(time.time() - self.t0, 3), phase=phase, thrust=thrust,
                              alt=altitude_from_ned(s["z"]), z=s["z"], vz=s["vz"],
                              rel_alt=s["rel_alt"], roll_deg=math.degrees(s["roll"]),
                              pitch_deg=math.degrees(s["pitch"]), yaw_deg=math.degrees(s["yaw"]),
                              armed=s["armed"], mode=s["mode"], landed=s["landed"]))

    def send_thrust(self, thrust, yaw):
        self.m.mav.set_attitude_target_send(
            int((time.time() - self.t0) * 1000), self.m.target_system, self.m.target_component,
            TYPE_MASK, level_quat(yaw), 0, 0, 0, thrust)

    def stream(self, phase, thrust_fn, duration, yaw, rate_hz=50, until=None):
        """Send SET_ATTITUDE_TARGET at rate_hz for `duration` s (or until `until(st)` is true)."""
        period = 1.0 / rate_hz
        start = time.time()
        nxt = start
        while time.time() - start < duration:
            thrust = thrust_fn(time.time() - start, self.st)
            if thrust is not None:
                self.send_thrust(thrust, yaw)
            self.pump()
            self.log(phase, thrust)
            if until and until(self.st):
                break
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))
        return time.time() - start

    def wait(self, phase, duration, until=None):
        start = time.time()
        while time.time() - start < duration:
            self.pump(0.02)
            self.log(phase, None)
            if until and until(self.st):
                break
        return time.time() - start

    def set_mode(self, mode):
        self.m.set_mode(mode)
        ok = self.wait(f"mode_{mode}", 5, until=lambda s: s["mode"] == mode) < 5
        if not ok:
            raise RuntimeError(f"mode {mode} not confirmed (still {self.st['mode']})")

    def arm(self):
        self.m.arducopter_arm()
        if self.wait("arm", 5, until=lambda s: s["armed"]) >= 5:
            raise RuntimeError("arm not confirmed")

    def wait_ready(self):
        end = time.time() + 120
        while time.time() < end:
            s = self.m.recv_match(type="SYS_STATUS", blocking=True, timeout=2)
            if s and s.onboard_control_sensors_health & M.MAV_SYS_STATUS_PREARM_CHECK:
                return
        raise RuntimeError("pre-arm checks not passing")

    def takeoff_std(self, alt):
        self.set_mode("GUIDED")
        self.arm()
        self.command(M.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
        self.wait("nav_takeoff", 30, until=lambda s: altitude_from_ned(s["z"]) > alt - 0.1)
        self.wait("settle", 3)

    def land(self):
        self.set_mode("LAND")
        self.wait("land", 60, until=lambda s: not s["armed"])
        print(f"    landed & disarmed: {not self.st['armed']}")

    def save(self, name):
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f"risk_{name}.csv")
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)
        self.rows = []
        print(f"    log: {os.path.relpath(path)}")


def rows_in(L, phase):
    return [r for r in L.rows if r["phase"] == phase]


def attitude_summary(rows, yaw0):
    mr = max(abs(r["roll_deg"]) for r in rows)
    mp = max(abs(r["pitch_deg"]) for r in rows)
    dy = max(abs((r["yaw_deg"] - math.degrees(yaw0) + 180) % 360 - 180) for r in rows)
    return f"max|roll|={mr:.2f}deg max|pitch|={mp:.2f}deg max|yaw drift|={dy:.2f}deg"


def test_zsign(L):
    print("== zsign: NED sign check during NAV_TAKEOFF to 5 m")
    L.takeoff_std(5)
    climb = [r for r in rows_in(L, "nav_takeoff") if r["alt"] > 1.0]
    vz_climb = sum(r["vz"] for r in climb) / len(climb)
    s = L.st
    print(f"    hover: z={s['z']:.2f} -> alt={altitude_from_ned(s['z']):.2f}, rel_alt={s['rel_alt']:.2f}")
    print(f"    mean vz while climbing = {vz_climb:.2f} m/s (expect < 0)")
    L.land()
    L.save("zsign")


def test_typemask(L, hover):
    print(f"== typemask: thrust steps around hover={hover:.3f} (type_mask={TYPE_MASK})")
    L.takeoff_std(5)
    yaw0 = L.st["yaw"]
    steps = [("hold", hover, 4), ("up", hover + 0.08, 2), ("hold2", hover, 2),
             ("down", hover - 0.08, 2), ("hold3", hover, 3)]
    for name, thr, dur in steps:
        a0 = altitude_from_ned(L.st["z"])
        L.stream(name, lambda t, s, thr=thr: thr, dur, yaw0)
        a1 = altitude_from_ned(L.st["z"])
        print(f"    {name:6s} thrust={thr:.3f}: alt {a0:5.2f} -> {a1:5.2f} (d={a1 - a0:+.2f}) vz={L.st['vz']:+.2f}")
    att_rows = [r for r in L.rows if r["phase"] in [s[0] for s in steps]]
    print("    " + attitude_summary(att_rows, yaw0))
    L.land()
    L.save("typemask")


def test_takeoff(L, hover):
    thr = min(hover + 0.10, 0.7)
    print(f"== takeoff: GUIDED + arm + thrust-only stream, thrust={thr:.3f}")
    L.set_mode("GUIDED")
    L.arm()
    yaw0 = L.st["yaw"]
    t_arm = time.time()
    # climb open-loop until 4 m, then crude bang-bang around 5 m for a few seconds
    dur = L.stream("climb", lambda t, s: thr, 15, yaw0,
                   until=lambda s: altitude_from_ned(s["z"]) > 4.0 or not s["armed"])
    lift = next((r for r in rows_in(L, "climb") if r["alt"] > 0.2), None)
    if not L.st["armed"]:
        print(f"    DISARMED during thrust-only takeoff after {dur:.1f}s")
    else:
        print(f"    armed->alt>0.2 m after {lift['t'] - (t_arm - L.t0):.2f}s; reached 4 m after {dur:.1f}s; landed_state={L.st['landed']}")
        L.stream("hold", lambda t, s: hover + (0.05 if altitude_from_ned(s["z"]) < 5 else -0.05), 5, yaw0)
        print(f"    after 5 s bang-bang: alt={altitude_from_ned(L.st['z']):.2f}  " +
              attitude_summary(rows_in(L, "climb") + rows_in(L, "hold"), yaw0))
    L.land()
    L.save("takeoff")


def test_timeout(L, hover, guid_timeout):
    print(f"== timeout: pause stream > GUID_TIMEOUT={guid_timeout:.1f}s")
    # Start high: until GUID_TIMEOUT expires, ArduPilot keeps applying the LAST thrust sent,
    # so the vehicle keeps accelerating during the gap (first run hit the ground from 5 m).
    L.takeoff_std(15)
    yaw0 = L.st["yaw"]
    # slightly below hover, so the gap first shows a descent, then the post-timeout behaviour
    L.stream("pre", lambda t, s: hover - 0.02, 1, yaw0)
    a_pause, vz_pause = altitude_from_ned(L.st["z"]), L.st["vz"]
    pause = guid_timeout + 3
    L.wait("pause", pause)
    pr = rows_in(L, "pause")
    for r in pr[:: max(1, len(pr) // 20)]:
        print(f"      t+{r['t'] - pr[0]['t']:4.1f}s alt={r['alt']:5.2f} vz={r['vz']:+.2f}")
    print(f"    pause start: alt={a_pause:.2f} vz={vz_pause:+.2f}; end: alt={altitude_from_ned(L.st['z']):.2f} vz={L.st['vz']:+.2f}")
    a0 = altitude_from_ned(L.st["z"])
    L.stream("resume", lambda t, s: hover + 0.08, 2, yaw0)
    print(f"    resume thrust={hover + 0.08:.3f}: alt {a0:.2f} -> {altitude_from_ned(L.st['z']):.2f} vz={L.st['vz']:+.2f} (expect climbing)")
    # low-rate stream test: 2 Hz is still < timeout, should keep thrust control
    a0 = altitude_from_ned(L.st["z"])
    L.stream("lowrate_2hz", lambda t, s: hover - 0.06, 3, yaw0, rate_hz=2)
    print(f"    2 Hz stream thrust={hover - 0.06:.3f}: alt {a0:.2f} -> {altitude_from_ned(L.st['z']):.2f} vz={L.st['vz']:+.2f} (expect descending)")
    L.land()
    L.save("timeout")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tests", nargs="*", default=["zsign", "typemask", "takeoff", "timeout"])
    ap.add_argument("--url", default="udp:127.0.0.1:14551")
    a = ap.parse_args()

    L = Link(a.url)
    params = {n: L.param(n) for n in ("GUID_OPTIONS", "MOT_THST_HOVER", "MOT_HOVER_LEARN",
                                      "GUID_TIMEOUT", "DISARM_DELAY")}
    print("params:", ", ".join(f"{k}={v:g}" for k, v in params.items()))
    if int(params["GUID_OPTIONS"]) & 8 == 0:
        sys.exit("GUID_OPTIONS bit 3 not set")
    L.wait_ready()
    for name in a.tests:
        hover = L.param("MOT_THST_HOVER")
        if name == "zsign":
            test_zsign(L)
        elif name == "typemask":
            test_typemask(L, hover)
        elif name == "takeoff":
            test_takeoff(L, hover)
        elif name == "timeout":
            test_timeout(L, hover, params["GUID_TIMEOUT"])
        L.wait("between", 3)
    print("MOT_THST_HOVER after tests:", f"{L.param('MOT_THST_HOVER'):.4f}")


if __name__ == "__main__":
    main()
