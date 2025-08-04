#!/usr/bin/env python3
"""
Fanuc Welding Control Frequency Test Tool
=========================================

This script performs two kinds of stress / frequency tests against the Fanuc
welding interface exposed through the *WeldCommand* / *WeldState* messages.

1. Jog-Feed Toggle Test
   Continually toggles the *jog_feed_cmd* (or *jog_retract_cmd*) output at a
   user-defined frequency to determine the maximum command rate that the
   controller can handle.

2. Target Wire-Speed Tracking Test
   Publishes a periodic waveform (sine or step) on *target_wire_spd* and
   measures the latency between the command message time-stamp and the moment
   when the *act_wire_spd* reported by *WeldState* reaches the commanded
   value (within a tolerance).

Both tests publish *WeldCommand* messages on the **/weld_command** topic and,
where applicable, subscribe to **/weld_state**.

Usage examples
--------------
# Jog-feed toggle at 20 Hz for 15 s
./weld_control_freq_test.py jog --freq 20 --duration 15

# Wire-speed sine wave 0.5 Hz, amplitude 40, offset 30, duration 60 s
./weld_control_freq_test.py wire --waveform sine --freq 0.5 --amp 40 --offset 30 --duration 60

Author: YOUR_NAME
Date  : 2024
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque
from typing import Deque, List, Tuple

import rospy
from std_msgs.msg import Header
from fanuc_driver.msg import WeldCommand, WeldState

# -------------- Helper / Core Class -------------------------------------------------

class WeldFreqTester:
    """ROS node that runs the requested frequency / latency test."""

    def __init__(self, threshold: float = 5.0):
        rospy.init_node("weld_control_freq_test", anonymous=True)

        self.cmd_pub = rospy.Publisher("/weld_command", WeldCommand, queue_size=1)
        self.threshold = threshold  # tolerance for considering the command reached

        # For wire-speed latency measurement
        self.cmd_history: Deque[Tuple[int, rospy.Time]] = deque(maxlen=2000)
        self.latencies: List[float] = []

        # Wait for at least one subscriber to /weld_command so we do not drop msgs
        wait_start = rospy.Time.now()
        rospy.loginfo("Waiting for subscriber to connect to /weld_command …")
        while self.cmd_pub.get_num_connections() == 0:
            if rospy.is_shutdown():
                sys.exit(0)
            if rospy.Time.now() - wait_start > rospy.Duration(10):
                rospy.logwarn("Timeout waiting for subscriber. Continuing anyway …")
                break
            rospy.sleep(0.1)

        # Subscribe to weld state for latency tests (always, harmless for jog test)
        self.state_sub = rospy.Subscriber(
            "/weld_state", WeldState, self._state_cb, queue_size=10
        )

    # ---------------------------------------------------------------------
    # ROS message helpers
    # ---------------------------------------------------------------------

    @staticmethod
    def _base_cmd() -> WeldCommand:
        """Return a *WeldCommand* with all fields initialised to the NO_CHANGE (-1) sentinel."""
        cmd = WeldCommand()
        cmd.header = Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "weld_control_freq_test"

        # Group Outputs
        cmd.target_wire_spd = -1
        cmd.correction_val = -1
        cmd.dyn_setting = -1
        cmd.operation_mode = -1
        cmd.std_pulse_val = -1
        cmd.program_number = -1

        # Digital Outputs
        cmd.arc_start_cmd = -1
        cmd.gas_control = -1
        cmd.jog_feed_cmd = -1
        cmd.jog_retract_cmd = -1
        return cmd

    def _publish_cmd(self, cmd: WeldCommand):
        """Publish a command, ensuring the header is up-to-date."""
        cmd.header.stamp = rospy.Time.now()
        self.cmd_pub.publish(cmd)

    # ---------------------------------------------------------------------
    # Jog-feed toggle test
    # ---------------------------------------------------------------------

    def run_jog_toggle_test(
        self, *, freq: float, duration: float, retract: bool = False
    ) -> None:
        """Toggle jog feed (or retract) on/off at *freq* Hz for *duration* seconds."""
        period = 1.0 / freq  # Desired message period (one command per toggle)
        # Each loop iteration publishes **one** message (ON or OFF) then sleeps for 'period'.
        # This makes the effective message frequency match the requested 'freq'.

        field_name = "jog_retract_cmd" if retract else "jog_feed_cmd"
        rospy.loginfo(
            "Starting jog-toggle test: field=%s   freq=%.2f Hz   duration=%.1f s",
            field_name,
            freq,
            duration,
        )

        t_end = rospy.get_time() + duration
        state_val = 1  # start with ON
        sent_msgs = 0
        while not rospy.is_shutdown() and rospy.get_time() < t_end:
            cmd = self._base_cmd()
            setattr(cmd, field_name, state_val)
            self._publish_cmd(cmd)
            sent_msgs += 1
            state_val ^= 1  # toggle 1 ↔ 0
            rospy.sleep(period)

        rospy.loginfo(
            "Jog-toggle test completed. Sent %d messages (~%.1f Hz).",
            sent_msgs,
            sent_msgs / duration if duration > 0 else 0,
        )

    # ---------------------------------------------------------------------
    # Wire-speed latency test
    # ---------------------------------------------------------------------

    def _state_cb(self, msg: WeldState):
        """Callback that measures latency between commanded and actual wire speed."""
        if not self.cmd_history:
            return

        current_val = msg.act_wire_spd
        target_val, t_cmd = self.cmd_history[0]  # oldest pending command
        if abs(current_val - target_val) <= self.threshold:
            latency = (rospy.Time.now() - t_cmd).to_sec()
            self.latencies.append(latency)
            self.cmd_history.popleft()
            rospy.logdebug(
                "Latency measured: target=%d actual=%d  Δt=%.4f s",
                target_val,
                current_val,
                latency,
            )

    def run_wire_speed_test(
        self,
        *,
        waveform: str,
        freq: float,
        amp: float,
        offset: float,
        duration: float,
        sample_rate: float = 50.0,
    ) -> None:
        """Publish a periodic waveform on *target_wire_spd* and measure latency.

        The function sends values:
            value(t) = offset + amp * f(t)
        where f(t) is either sin(2π f t) in the *sine* case or a square wave (+1/-1)
        in the *step* case.
        """
        if waveform not in {"sine", "step"}:
            raise ValueError("Unsupported waveform: %s" % waveform)

        rospy.loginfo(
            "Starting wire-speed %s test: freq=%.3f Hz   amp=%.1f   offset=%.1f   duration=%.1f s",
            waveform,
            freq,
            amp,
            offset,
            duration,
        )

        rate = rospy.Rate(sample_rate)
        start_time = rospy.get_time()
        t_end = start_time + duration

        while not rospy.is_shutdown() and rospy.get_time() < t_end:
            t_now = rospy.get_time() - start_time

            if waveform == "sine":
                func_val = math.sin(2 * math.pi * freq * t_now)  # −1 … +1
            else:  # step / square wave
                phase = (t_now * freq) % 1.0
                func_val = 1.0 if phase < 0.5 else -1.0

            target_val = int(offset + amp * func_val)
            #print(target_val)
            cmd = self._base_cmd()
            cmd.target_wire_spd = target_val
            self._publish_cmd(cmd)

            # Store command for latency measurement
            self.cmd_history.append((target_val, rospy.Time.now()))

            rate.sleep()

        if self.latencies:
            avg_lat = sum(self.latencies) / len(self.latencies)
            max_lat = max(self.latencies)
            min_lat = min(self.latencies)
            rospy.loginfo(
                "Wire-speed test finished. Latency samples: %d  avg=%.4f s  min=%.4f s  max=%.4f s",
                len(self.latencies),
                avg_lat,
                min_lat,
                max_lat,
            )
        else:
            rospy.logwarn("No latency samples collected. Check threshold/connection.")


# -------------- main / argument parsing ---------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Fanuc Welding Control Frequency / Latency Test Tool",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    subparsers = parser.add_subparsers(dest="test", required=True, help="Test type")

    # Jog-feed toggle test -----------------------------------------------------------------
    jog_p = subparsers.add_parser("jog", help="Jog feed or retract ON/OFF toggle test")
    jog_p.add_argument(
        "-f", "--freq", type=float, default=10.0, help="Toggle frequency [Hz]"
    )
    jog_p.add_argument(
        "-d", "--duration", type=float, default=10.0, help="Test duration [s]"
    )
    jog_p.add_argument(
        "--retract",
        action="store_true",
        help="Toggle *jog_retract_cmd* instead of *jog_feed_cmd*.",
    )

    # Wire-speed latency test ------------------------------------------------------------
    wire_p = subparsers.add_parser("wire", help="Target wire-speed tracking / latency test")
    wire_p.add_argument(
        "-w",
        "--waveform",
        choices=["sine", "step"],
        default="sine",
        help="Waveform type sent to target_wire_spd",
    )
    wire_p.add_argument("-f", "--freq", type=float, default=1.0, help="Waveform frequency [Hz]")
    wire_p.add_argument("--amp", type=float, default=15.0, help="Amplitude of waveform")
    wire_p.add_argument(
        "--offset",
        type=float,
        default=20.0,
        help="Offset / centre value of waveform",
    )
    wire_p.add_argument(
        "-d", "--duration", type=float, default=30.0, help="Test duration [s]"
    )
    wire_p.add_argument(
        "--threshold",
        type=float,
        default=1.0,
        help="Tolerance between commanded and actual wire speed considered 'reached'",
    )
    wire_p.add_argument(
        "--sample-rate",
        type=float,
        default=40.0,
        help="Publishing sample rate [Hz]",
    )

    args = parser.parse_args()

    try:
        tester = WeldFreqTester(threshold=getattr(args, "threshold", 5.0))

        if args.test == "jog":
            tester.run_jog_toggle_test(
                freq=args.freq, duration=args.duration, retract=args.retract
            )
        elif args.test == "wire":
            tester.run_wire_speed_test(
                waveform=args.waveform,
                freq=args.freq,
                amp=args.amp,
                offset=args.offset,
                duration=args.duration,
                sample_rate=args.sample_rate,
            )
        else:
            parser.error("Unknown test type: %s" % args.test)

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nUser interrupted. Shutting down …")


if __name__ == "__main__":
    main()
