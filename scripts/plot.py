#!/usr/bin/env python3
"""Plot a flight log from altitude_control.

  python scripts/plot.py [logs/flight.csv] [-o flight.png] [--compare other.csv ...]

Panels: altitude (target, trajectory setpoint, actual), climb rate (setpoint vs actual),
thrust (with velocity-PID P/I terms). Mission states are shaded in the background.
"""
import argparse
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

STATE_COLORS = {
    "CLIMB_TO_HIGH": "#dbe9f6", "HOLD_HIGH": "#c6dbef",
    "DESCEND_TO_LOW": "#fde0c5", "HOLD_LOW": "#fdd0a2",
}


def load(path):
    rows = list(csv.DictReader(open(path)))
    cols = {k: [] for k in rows[0].keys()}
    for r in rows:
        for k, v in r.items():
            cols[k].append(v if k == "state" else float(v))
    return cols


def shade_states(ax, d):
    t, st = d["t_s"], d["state"]
    start = 0
    for i in range(1, len(st) + 1):
        if i == len(st) or st[i] != st[start]:
            color = STATE_COLORS.get(st[start])
            if color:
                ax.axvspan(t[start], t[i - 1], color=color, alpha=0.5, lw=0)
            start = i


def label_states(ax, d):
    t, st = d["t_s"], d["state"]
    start = 0
    ymax = ax.get_ylim()[1]
    for i in range(1, len(st) + 1):
        if i == len(st) or st[i] != st[start]:
            ax.text((t[start] + t[i - 1]) / 2, ymax * 0.97, st[start].replace("_", " ").lower(),
                    ha="center", va="top", fontsize=7, color="#555")
            start = i


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", default="logs/flight.csv")
    ap.add_argument("-o", "--output", help="PNG path (default: next to the log)")
    ap.add_argument("--compare", nargs="*", default=[], help="extra logs: altitude overlay only")
    ap.add_argument("--title", default="Thrust-only altitude control, ArduCopter SITL")
    a = ap.parse_args()

    d = load(a.log)
    t = d["t_s"]
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9), sharex=True,
                                        gridspec_kw={"height_ratios": [3, 1.6, 1.6]})
    for ax in (ax1, ax2, ax3):
        shade_states(ax, d)
        ax.grid(True, alpha=0.3)

    ax1.plot(t, d["target_alt_m"], "k--", lw=1, label="target altitude")
    ax1.plot(t, d["setpoint_alt_m"], color="#e6550d", lw=1.2, label="setpoint (trajectory)")
    ax1.plot(t, d["alt_m"], color="#08519c", lw=1.8, label="actual altitude")
    for path in a.compare:
        c = load(path)
        ax1.plot(c["t_s"], c["alt_m"], lw=1, alpha=0.7, label=os.path.basename(path))
    ax1.set_ylabel("altitude above take-off [m]")
    ax1.set_ylim(bottom=min(-0.3, min(d["alt_m"]) - 0.3), top=max(d["target_alt_m"]) + 1.5)
    ax1.legend(loc="center right", fontsize=8)
    label_states(ax1, d)

    ax2.plot(t, d["climb_sp_ms"], color="#e6550d", lw=1, label="climb-rate setpoint")
    ax2.plot(t, d["climb_ms"], color="#08519c", lw=1.4, label="actual climb rate")
    ax2.set_ylabel("climb rate [m/s]")
    ax2.legend(loc="upper right", fontsize=8)

    ax3.plot(t, d["thrust"], color="#31a354", lw=1.4, label="thrust command")
    hover = [th - p - i - dd for th, p, i, dd in zip(d["thrust"], d["p"], d["i"], d["d"])]
    ax3.plot(t, hover, "k:", lw=1, label="hover feedforward (MOT_THST_HOVER)")
    ax3.plot(t, [h + i for h, i in zip(hover, d["i"])], color="#756bb1", lw=1,
             label="feedforward + I term")
    ax3.set_ylabel("thrust [0..1]")
    ax3.set_xlabel("time since arming [s]")
    ax3.legend(loc="upper right", fontsize=8)

    fig.suptitle(a.title)
    fig.tight_layout()
    out = a.output or os.path.splitext(a.log)[0] + ".png"
    fig.savefig(out, dpi=120)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
