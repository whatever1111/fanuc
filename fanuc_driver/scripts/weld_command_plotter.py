#!/usr/bin/env python3
"""
Weld Command Plotter
====================

Subscribe to a WeldCommand topic and plot selected commanded fields over time.

This tool is intended to visually validate that the command generator publishes
the expected values. It only depends on the WeldCommand topic and does not
subscribe to any other messages (e.g., actual state).

Requirements:
- matplotlib (headless): sudo apt install -y python3-matplotlib

Examples:
- Plot target wire speed for 20 seconds and save to PNG:
  ./weld_command_plotter.py --fields target_wire_spd --duration 20 --out wire_cmd.png

- Plot multiple fields (step plots) from a custom topic:
  ./weld_command_plotter.py --topic /weld_command --fields target_wire_spd,jog_feed_cmd,arc_start_cmd \
      --duration 15 --out cmds.png
"""

from __future__ import annotations

import argparse
from typing import Dict, List, Tuple

import matplotlib

# Use non-interactive backend (works without display)
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import rospy  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from fanuc_driver.msg import WeldCommand  # noqa: E402


def parse_fields(csv_fields: str) -> List[str]:
    return [f.strip() for f in csv_fields.split(",") if f.strip()]


class CommandPlotter:
    def __init__(
        self,
        *,
        topic: str,
        fields: List[str],
        duration: float,
        out_path: str,
        hold_last: bool,
        figsize: Tuple[float, float],
        title: str,
    ) -> None:
        self.topic = topic
        self.fields = fields
        self.duration = duration
        self.out_path = out_path
        self.hold_last = hold_last
        self.figsize = figsize
        self.title = title

        # Time origin (first message time)
        self._t0: float | None = None

        # Per-field time/value storage
        self._times: Dict[str, List[float]] = {f: [] for f in fields}
        self._values: Dict[str, List[float]] = {f: [] for f in fields}
        self._last_value: Dict[str, float | None] = {f: None for f in fields}

        self._sub = rospy.Subscriber(self.topic, WeldCommand, self._cb, queue_size=100)

    def _cb(self, msg: WeldCommand) -> None:
        # Prefer command header time; fallback to receipt time
        try:
            t_msg = float(msg.header.stamp.to_sec())
        except Exception:
            t_msg = float(rospy.Time.now().to_sec())

        if self._t0 is None:
            self._t0 = t_msg
        t_rel = t_msg - self._t0

        for field in self.fields:
            if not hasattr(msg, field):
                # Unknown field on message; skip
                continue
            raw_val = getattr(msg, field)

            # WeldCommand uses -1 as NO_CHANGE sentinel for most fields
            val: float | None
            try:
                val_num = float(raw_val)
            except Exception:
                # Non-numeric field; skip
                continue

            if val_num == -1.0:
                if self.hold_last and self._last_value[field] is not None:
                    val = self._last_value[field]
                else:
                    # Skip logging NO_CHANGE if we have no prior value
                    continue
            else:
                val = val_num
                self._last_value[field] = val

            self._times[field].append(t_rel)
            self._values[field].append(val)

    def run(self) -> None:
        end_time = rospy.get_time() + self.duration
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown() and rospy.get_time() < end_time:
            rate.sleep()

    def plot(self) -> None:
        # Count valid series
        valid_fields = [f for f in self.fields if len(self._times[f]) > 1]
        if not valid_fields:
            rospy.logwarn("No data collected to plot. Did any messages arrive on %s?", self.topic)
            return

        num = len(valid_fields)
        fig, axes = plt.subplots(num, 1, figsize=self.figsize, sharex=True)
        if num == 1:
            axes = [axes]

        fig.suptitle(self.title)

        for ax, field in zip(axes, valid_fields):
            ts = self._times[field]
            vs = self._values[field]
            ax.step(ts, vs, where="post", label=field)
            ax.set_ylabel(field)
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best")

        axes[-1].set_xlabel("time [s]")
        fig.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig.savefig(self.out_path, dpi=150)
        rospy.loginfo("Saved plot to: %s", self.out_path)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot selected WeldCommand fields over time",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--topic", type=str, default="/weld_command", help="WeldCommand topic to subscribe")
    parser.add_argument(
        "--fields",
        type=str,
        default="target_wire_spd",
        help="Comma-separated list of WeldCommand fields to plot",
    )
    parser.add_argument("--duration", type=float, default=15.0, help="Recording duration [s]")
    parser.add_argument("--out", type=str, default="weld_command_plot.png", help="Output image file path")
    parser.add_argument(
        "--hold-last",
        action="store_true",
        help="When field is -1 (NO_CHANGE), hold the last known value in the plot",
    )
    parser.add_argument(
        "--figsize",
        type=float,
        nargs=2,
        default=(10.0, 6.0),
        metavar=("W", "H"),
        help="Figure size in inches (width height)",
    )
    parser.add_argument("--title", type=str, default="WeldCommand Plot", help="Figure title")

    args, _ = parser.parse_known_args()

    rospy.init_node("weld_command_plotter", anonymous=True)

    fields = parse_fields(args.fields)
    if not fields:
        rospy.logerr("No fields provided to plot.")
        return

    plotter = CommandPlotter(
        topic=args.topic,
        fields=fields,
        duration=args.duration,
        out_path=args.out,
        hold_last=bool(args.hold_last),
        figsize=(float(args.figsize[0]), float(args.figsize[1])),
        title=args.title,
    )

    rospy.loginfo(
        "Starting plotter: topic=%s fields=%s duration=%.1fs out=%s",
        args.topic,
        ",".join(fields),
        args.duration,
        args.out,
    )

    plotter.run()
    plotter.plot()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

