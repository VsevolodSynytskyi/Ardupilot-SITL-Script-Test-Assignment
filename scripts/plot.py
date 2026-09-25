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
TARGET_COLOR, SETPOINT_COLOR, ACTUAL_COLOR = "black", "#e6550d", "#08519c"
THRUST_COLOR, INTEGRAL_COLOR = "#31a354", "#756bb1"


def load_columns(path):
    with open(path) as log_file:
        rows = list(csv.DictReader(log_file))
    columns = {name: [] for name in rows[0].keys()}
    for row in rows:
        for name, value in row.items():
            columns[name].append(value if name == "state" else float(value))
    return columns


def state_spans(log):
    times, states = log["time_s"], log["state"]
    start = 0
    for index in range(1, len(states) + 1):
        if index == len(states) or states[index] != states[start]:
            yield states[start], times[start], times[index - 1]
            start = index


def shade_states(axis, log):
    for state, start_s, end_s in state_spans(log):
        if state in STATE_COLORS:
            axis.axvspan(start_s, end_s, color=STATE_COLORS[state], alpha=0.5, lw=0)


def label_states(axis, log):
    label_y = axis.get_ylim()[1] * 0.97
    for state, start_s, end_s in state_spans(log):
        axis.text((start_s + end_s) / 2, label_y, state.replace("_", " ").lower(),
                  ha="center", va="top", fontsize=7, color="#555")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log", nargs="?", default="logs/flight.csv")
    parser.add_argument("-o", "--output", help="PNG path (default: next to the log)")
    parser.add_argument("--compare", nargs="*", default=[], help="extra logs: altitude overlay only")
    parser.add_argument("--title", default="Thrust-only altitude control, ArduCopter SITL")
    args = parser.parse_args()

    log = load_columns(args.log)
    time_s = log["time_s"]
    figure, (altitude_axis, climb_axis, thrust_axis) = plt.subplots(
        3, 1, figsize=(11, 9), sharex=True, gridspec_kw={"height_ratios": [3, 1.6, 1.6]})
    for axis in (altitude_axis, climb_axis, thrust_axis):
        shade_states(axis, log)
        axis.grid(True, alpha=0.3)

    altitude_axis.plot(time_s, log["target_alt_m"], "--", color=TARGET_COLOR, lw=1,
                       label="target altitude")
    altitude_axis.plot(time_s, log["setpoint_alt_m"], color=SETPOINT_COLOR, lw=1.2,
                       label="setpoint (trajectory)")
    altitude_axis.plot(time_s, log["alt_m"], color=ACTUAL_COLOR, lw=1.8, label="actual altitude")
    for path in args.compare:
        other = load_columns(path)
        altitude_axis.plot(other["time_s"], other["alt_m"], lw=1, alpha=0.7,
                           label=os.path.basename(path))
    altitude_axis.set_ylabel("altitude above take-off [m]")
    altitude_axis.set_ylim(bottom=min(-0.3, min(log["alt_m"]) - 0.3),
                           top=max(log["target_alt_m"]) + 1.5)
    altitude_axis.legend(loc="center right", fontsize=8)
    label_states(altitude_axis, log)

    climb_axis.plot(time_s, log["climb_setpoint_mps"], color=SETPOINT_COLOR, lw=1,
                    label="climb-rate setpoint")
    climb_axis.plot(time_s, log["climb_mps"], color=ACTUAL_COLOR, lw=1.4, label="actual climb rate")
    climb_axis.set_ylabel("climb rate [m/s]")
    climb_axis.legend(loc="upper right", fontsize=8)

    hover_feedforward = [thrust - p - i - d for thrust, p, i, d
                         in zip(log["thrust"], log["p"], log["i"], log["d"])]
    thrust_axis.plot(time_s, log["thrust"], color=THRUST_COLOR, lw=1.4, label="thrust command")
    thrust_axis.plot(time_s, hover_feedforward, ":", color=TARGET_COLOR, lw=1,
                     label="hover feedforward (MOT_THST_HOVER)")
    thrust_axis.plot(time_s, [hover + i for hover, i in zip(hover_feedforward, log["i"])],
                     color=INTEGRAL_COLOR, lw=1, label="feedforward + I term")
    thrust_axis.set_ylabel("thrust [0..1]")
    thrust_axis.set_xlabel("time since arming [s]")
    thrust_axis.legend(loc="upper right", fontsize=8)

    figure.suptitle(args.title)
    figure.tight_layout()
    output_path = args.output or os.path.splitext(args.log)[0] + ".png"
    figure.savefig(output_path, dpi=120)
    print(f"saved {output_path}")


if __name__ == "__main__":
    main()
