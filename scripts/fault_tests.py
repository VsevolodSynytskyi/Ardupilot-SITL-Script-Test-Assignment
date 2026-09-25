#!/usr/bin/env python3
"""Fault-injection tests for altitude_control against a running SITL (scripts/run_sitl.sh).

  python scripts/fault_tests.py [mode_change link_loss stall early_interrupt]

  mode_change      at HOLD_HIGH switch to LOITER from the GCS port -> controller releases control (exit 3)
  link_loss        at HOLD_HIGH freeze the MAVProxy router for 3 s -> ERROR (stale telemetry) -> LAND.
                   Runs last by default: afterwards the router's stream stays bursty (gaps > 0.2 s once
                   the controller streams again), so restart scripts/run_sitl.sh before the next flight.
  stall            at HOLD_HIGH freeze the controller process for 1.5 s -> ERROR (loop stalled) -> LAND
  early_interrupt  Ctrl+C during start-up -> exits quickly, never arms
  armed_start      vehicle already flying (GCS take-off) -> controller refuses to start
"""
import os
import signal
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

ROOT = os.path.join(os.path.dirname(__file__), "..")
BIN = os.path.join(ROOT, "build", "altitude_control")
GCS_URL = "udp:127.0.0.1:14550"


class Controller:
    """altitude_control subprocess with its stdout collected line by line."""

    def __init__(self, name):
        self.name = name
        self.lines = []
        self.p = subprocess.Popen(
            [BIN, "--set", f"log_path=logs/fault_{name}.csv"], cwd=ROOT,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        with open(os.path.join(ROOT, "logs", f"fault_{self.name}.txt"), "w") as f:
            for line in self.p.stdout:
                self.lines.append(line.rstrip())
                f.write(line)
                f.flush()

    def wait_for(self, text, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if any(text in l for l in self.lines):
                return True
            if self.p.poll() is not None:
                return False
            time.sleep(0.05)
        return False

    def finish(self, timeout=150):
        return self.p.wait(timeout=timeout)

    def mission_log(self):
        return [l for l in self.lines if l.startswith("[mission]") and "landing:" not in l]


_gcs = None


def gcs():
    """One shared GCS connection (the UDP port can only be bound once)."""
    global _gcs
    if _gcs is None:
        _gcs = mavutil.mavlink_connection(GCS_URL, source_system=254)
        _gcs.wait_heartbeat(timeout=10)
    while _gcs.recv_match(blocking=False):  # drop stale buffered messages
        pass
    return _gcs


def altitude(m, timeout=2):
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
    return msg.relative_alt / 1000.0 if msg else float("nan")


def wait_disarmed(m, timeout=90):
    end = time.time() + timeout
    while time.time() < end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.get_srcSystem() == 1 and not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return True
    return False


def pid_of(pattern):
    out = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True).stdout.split()
    return int(out[0]) if out else None


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}: {name}{(' - ' + detail) if detail else ''}")
    return ok


def test_mode_change():
    print("== mode_change")
    c = Controller("mode_change")
    if not c.wait_for("-> HOLD_HIGH", 60):
        return check("reached HOLD_HIGH", False)
    m = gcs()
    m.set_mode("LOITER")
    rc = c.finish()
    ok = check("exit code 3 (released)", rc == 3, f"rc={rc}")
    ok &= check("no LAND commanded", not any("-> LAND" in l for l in c.lines))
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    ok &= check("vehicle still in LOITER", m.flightmode == "LOITER", m.flightmode)
    print("   ", "\n    ".join(c.mission_log()[-3:]))
    m.set_mode("LAND")  # clean up
    wait_disarmed(m)
    return ok


def freeze_during_hold(name, pid_pattern, freeze_s, expect_text):
    print(f"== {name}")
    c = Controller(name)
    if not c.wait_for("-> HOLD_HIGH", 60):
        return check("reached HOLD_HIGH", False)
    m = gcs()
    pid = pid_of(pid_pattern) if pid_pattern else c.p.pid
    alt0 = altitude(m)
    os.kill(pid, signal.SIGSTOP)
    alts = []
    end = time.time() + freeze_s
    while time.time() < end:
        a = altitude(m, timeout=0.5)
        if a == a:  # not NaN (GCS link is also frozen when MAVProxy is stopped)
            alts.append(a)
    os.kill(pid, signal.SIGCONT)
    rc = c.finish()
    ok = check(f"error detected ({expect_text})", any(expect_text in l for l in c.lines))
    ok &= check("LAND commanded and disarmed", any("landed and disarmed" in l for l in c.lines), f"rc={rc}")
    if alts:
        drift = max(abs(a - alt0) for a in alts)
        ok &= check("altitude held during freeze", drift < 1.0, f"max drift {drift:.2f} m")
    print("   ", "\n    ".join(c.mission_log()[-5:]))
    return ok


def test_link_loss():
    return freeze_during_hold("link_loss", "mavproxy.py", 3.0, "telemetry stale")


def test_stall():
    return freeze_during_hold("stall", None, 1.5, "control loop stalled")


def test_early_interrupt():
    print("== early_interrupt")
    c = Controller("early_interrupt")
    time.sleep(1.0)
    t0 = time.time()
    c.p.send_signal(signal.SIGINT)
    rc = c.finish(timeout=30)
    dt = time.time() - t0
    ok = check("exited quickly", dt < 10, f"{dt:.1f} s, rc={rc}")
    ok &= check("never armed", not any("-> CLIMB_TO_HIGH" in l for l in c.lines))
    print("   ", "\n    ".join(c.mission_log()[-2:]))
    return ok


def test_armed_start():
    print("== armed_start")
    m = gcs()
    m.set_mode("GUIDED")
    m.arducopter_arm()
    m.motors_armed_wait()
    m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 3)
    time.sleep(6)
    c = Controller("armed_start")
    rc = c.finish(timeout=30)
    ok = check("refused to start", rc == 1 and any("already armed" in l for l in c.lines), f"rc={rc}")
    ok &= check("did not stream thrust", not any("-> CLIMB_TO_HIGH" in l for l in c.lines))
    print("   ", "\n    ".join(c.mission_log()[-1:]))
    m.set_mode("LAND")
    wait_disarmed(m)
    return ok


TESTS = {"early_interrupt": test_early_interrupt, "armed_start": test_armed_start,
         "mode_change": test_mode_change, "stall": test_stall, "link_loss": test_link_loss}

if __name__ == "__main__":
    names = sys.argv[1:] or list(TESTS)
    results = {n: TESTS[n]() for n in names}
    print("\n" + ", ".join(f"{n}: {'PASS' if r else 'FAIL'}" for n, r in results.items()))
    sys.exit(0 if all(results.values()) else 1)
