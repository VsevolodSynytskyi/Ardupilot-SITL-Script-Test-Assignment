#!/usr/bin/env python3
"""Run the mission in SITL once per gain set and compare step-response metrics.

Usage (SITL running):
  python scripts/tune.py "vel_kp=0.15" "vel_kp=0.25" "vel_kp=0.25 vel_kd=0.01"
Each argument is one run: space-separated key=value overrides for altitude_control --set.
Logs: logs/tune/<n>.csv; results appended to logs/tune/results.csv.
"""
import csv
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
from flight_metrics import compute  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
BIN = os.path.join(ROOT, "build", "altitude_control")
OUT = os.path.join(ROOT, "logs", "tune")
BASE = ["hold_time_s=5"]  # shorter holds for tuning runs
COLS = ["overshoot_up_m", "settle10_up_s", "overshoot_down_m", "settle10_down_s",
        "hold_rms_up_m", "hold_rms_down_m", "vel_rms_up_mps", "vel_rms_down_mps",
        "thrust_jitter_up", "thrust_max"]
FMT = {"thrust_jitter_up": "13.5f"}


def run(label, overrides, idx):
    os.makedirs(OUT, exist_ok=True)
    log = os.path.join(OUT, f"{idx:02d}.csv")
    args = [BIN, "--set", f"log_path={log}"]
    for kv in BASE + overrides:
        args += ["--set", kv]
    res = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=400)
    if res.returncode != 0:
        tail = "\n".join([l for l in res.stdout.splitlines() if "[mission]" in l][-3:])
        print(f"  run {idx} ({label}) FAILED rc={res.returncode}\n{tail}")
        return None
    return compute(log)


def main():
    runs = sys.argv[1:] or [""]
    results_path = os.path.join(OUT, "results.csv")
    os.makedirs(OUT, exist_ok=True)
    start = len(os.listdir(OUT))
    print(f"{'#':>3} {'overrides':38s} " + " ".join(f"{c[:13]:>13s}" for c in COLS))
    for n, spec in enumerate(runs):
        overrides = spec.split()
        m = run(spec, overrides, start + n)
        if m is None:
            continue
        print(f"{start + n:3d} {spec or '(config)':38s} " + " ".join(format(m[c], FMT.get(c, "13.3f")) for c in COLS),
              flush=True)
        new = not os.path.exists(results_path)
        with open(results_path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["run", "overrides"] + list(m.keys()))
            if new:
                w.writeheader()
            w.writerow({"run": start + n, "overrides": spec, **m})


if __name__ == "__main__":
    main()
