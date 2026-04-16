#!/usr/bin/env python3
"""Record /joint_states to CSV for sim-to-real trajectory comparison.

This is a READ-ONLY observer — it subscribes to /joint_states and writes
(timestamp, joint_positions) rows to a CSV file. It does NOT send any
commands to the robot or any controller.

Auto-detection:
    - Waits silently until joint positions start changing (motion detected).
    - Stops recording when positions have been idle for --idle seconds.
    - Or stops after --duration seconds of total recording (whichever first).

Usage (inside container or natively, with ROS 2 sourced):
    # Start recorder, then trigger a motion from another terminal
    python3 trajectory_recorder.py -o sim_traj.csv
    python3 trajectory_recorder.py -o real_traj.csv --topic /joint_states

The output CSV has columns:
    time_sec, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6
where time_sec is seconds since motion was first detected (t=0).
"""

import argparse
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class TrajectoryRecorder(Node):
    def __init__(self, topic, output, duration, idle_timeout, threshold):
        super().__init__("trajectory_recorder")
        self.output = output
        self.duration = duration
        self.idle_timeout = idle_timeout
        self.threshold = threshold

        self.rows = []
        self.prev_pos = None
        self.motion_detected = False
        self.motion_start_time = None
        self.last_motion_time = None
        self.done = False

        self.sub = self.create_subscription(JointState, topic, self._cb, 10)
        self.get_logger().info(
            f"Waiting for motion on '{topic}' (threshold={threshold:.6f} rad) ..."
        )

    def _cb(self, msg):
        if self.done:
            return

        pos_map = {n: p for n, p in zip(msg.name, msg.position)}
        try:
            pos = [pos_map[j] for j in JOINT_ORDER]
        except KeyError:
            return

        now = time.time()

        if self.prev_pos is not None:
            delta = sum(abs(a - b) for a, b in zip(pos, self.prev_pos))
            moving = delta > self.threshold

            if not self.motion_detected:
                if moving:
                    self.motion_detected = True
                    self.motion_start_time = now
                    self.last_motion_time = now
                    self.get_logger().info("Motion detected — recording started.")
            else:
                if moving:
                    self.last_motion_time = now

        self.prev_pos = list(pos)

        if not self.motion_detected:
            return

        t = now - self.motion_start_time
        self.rows.append([round(t, 6)] + [round(p, 8) for p in pos])

        if self.duration > 0 and t >= self.duration:
            self.get_logger().info(f"Duration limit ({self.duration}s) reached.")
            self._finish()
        elif (now - self.last_motion_time) >= self.idle_timeout:
            self.get_logger().info(
                f"Idle for {self.idle_timeout}s — stopping recording."
            )
            self._finish()

    def _finish(self):
        self.done = True
        with open(self.output, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_sec"] + JOINT_ORDER)
            writer.writerows(self.rows)
        total = self.rows[-1][0] if self.rows else 0
        self.get_logger().info(
            f"Saved {len(self.rows)} samples ({total:.2f}s) to {self.output}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", required=True, help="Output CSV path")
    parser.add_argument("--topic", default="/joint_states", help="Joint state topic")
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Max recording duration in seconds (0=unlimited)",
    )
    parser.add_argument(
        "--idle",
        type=float,
        default=2.0,
        help="Stop after this many seconds of no motion",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.0001,
        help="Sum of absolute joint deltas to consider as 'moving' (rad)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryRecorder(
        topic=args.topic,
        output=args.output,
        duration=args.duration,
        idle_timeout=args.idle,
        threshold=args.threshold,
    )

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if node.rows:
            node._finish()
        else:
            node.get_logger().info("Interrupted before any motion — no output.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
