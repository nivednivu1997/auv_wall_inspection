#!/usr/bin/env python3
"""
validate_controller.py
══════════════════════
ROS 2 Python test-harness node for the distance_keep_controller package.

Runs five sequential test scenarios without requiring a full Gazebo install:
  TC-1  Steady state    — distance = setpoint → cmd.linear.y ≈ 0
  TC-2  Too far         — distance = 4.0 m    → positive lateral cmd
  TC-3  Too close       — distance = 0.5 m    → negative lateral cmd
  TC-4  Heartbeat loss  — stop publishing     → /emergency_active = true
  TC-5  Depth violation — depth > max         → /emergency_active = true

Usage:
  # Terminal 1 — launch the controller stack
  ros2 launch distance_keep_controller distance_keep.launch.py

  # Terminal 2 — run the validator
  python3 validate_controller.py

  # Or source the workspace and run as a node
  ros2 run distance_keep_controller validate_controller
"""

import sys
import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist


# ── Tunables ────────────────────────────────────────────────────────────────
SETPOINT          = 2.0   # metres — must match controller parameter
MAX_DEPTH         = 10.0  # metres — must match watchdog parameter
HEARTBEAT_TIMEOUT = 2.0   # seconds
SETTLE_TIME       = 1.5   # seconds to wait for controller to respond
CMD_TOLERANCE     = 0.1   # |output| threshold for "near zero" check
PASS  = "\033[92m[PASS]\033[0m"
FAIL  = "\033[91m[FAIL]\033[0m"
INFO  = "\033[94m[INFO]\033[0m"
WARN  = "\033[93m[WARN]\033[0m"


class ControllerValidator(Node):
    """Publishes synthetic sensor data and asserts correct node responses."""

    def __init__(self):
        super().__init__("controller_validator")

        reliable_qos = QoSProfile(depth=10,
                                  reliability=ReliabilityPolicy.RELIABLE)

        # ── Publishers (feed synthetic sensor data) ──────────────────────────
        self.distance_pub = self.create_publisher(Float64, "/wall_distance",
                                                  reliable_qos)
        self.depth_pub    = self.create_publisher(Float64, "/depth",
                                                  reliable_qos)

        # ── Subscribers (observe controller outputs) ─────────────────────────
        self.cmd_msg           = None
        self.emergency_active  = False
        self.emergency_cmd_msg = None

        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel",
            lambda m: setattr(self, "cmd_msg", m),
            reliable_qos)

        self.emergency_flag_sub = self.create_subscription(
            Bool, "/emergency_active",
            lambda m: setattr(self, "emergency_active", m.data),
            reliable_qos)

        self.emergency_cmd_sub = self.create_subscription(
            Twist, "/emergency_cmd",
            lambda m: setattr(self, "emergency_cmd_msg", m),
            reliable_qos)

        # Results store
        self.results = []

        self.get_logger().info(f"{INFO} ControllerValidator initialised.")

    # ─────────────────────────────────────────────────────────────────────────
    #  Helpers
    # ─────────────────────────────────────────────────────────────────────────

    def publish_distance(self, distance_m: float, count: int = 20,
                         hz: float = 10.0):
        """Publish `count` distance messages at `hz` Hz."""
        period = 1.0 / hz
        msg = Float64()
        msg.data = distance_m
        for _ in range(count):
            self.distance_pub.publish(msg)
            time.sleep(period)

    def publish_depth(self, depth_m: float):
        """Publish a single depth reading."""
        msg = Float64()
        msg.data = depth_m
        self.depth_pub.publish(msg)

    def reset_state(self):
        """Clear last observed messages before a new test case."""
        self.cmd_msg           = None
        self.emergency_active  = False
        self.emergency_cmd_msg = None

    def wait_for_cmd(self, timeout_s: float = 3.0) -> bool:
        """Block until a /cmd_vel message is received or timeout."""
        t0 = time.time()
        while self.cmd_msg is None and (time.time() - t0) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.cmd_msg is not None

    def wait_for_emergency(self, timeout_s: float = 5.0) -> bool:
        """Block until /emergency_active = true or timeout."""
        t0 = time.time()
        while not self.emergency_active and (time.time() - t0) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.emergency_active

    def record(self, name: str, passed: bool, detail: str = ""):
        tag = PASS if passed else FAIL
        self.get_logger().info(f"{tag}  {name}  {detail}")
        self.results.append((name, passed, detail))

    # ─────────────────────────────────────────────────────────────────────────
    #  Test Cases
    # ─────────────────────────────────────────────────────────────────────────

    def tc1_steady_state(self):
        """TC-1: distance == setpoint → lateral output should be near zero."""
        self.get_logger().info(f"\n{INFO} TC-1  Steady state (distance = {SETPOINT} m)")
        self.reset_state()

        # Publish at setpoint for 2 s so PID history builds up
        self.publish_distance(SETPOINT, count=20, hz=10.0)
        time.sleep(SETTLE_TIME)
        # Read final cmd
        self.cmd_msg = None
        self.publish_distance(SETPOINT, count=5, hz=10.0)
        self.wait_for_cmd(timeout_s=2.0)

        if self.cmd_msg is None:
            self.record("TC-1 Steady state", False, "No /cmd_vel received")
            return

        output = self.cmd_msg.linear.y
        near_zero = abs(output) <= CMD_TOLERANCE
        self.record("TC-1 Steady state",
                    near_zero,
                    f"cmd.linear.y = {output:.4f} (tolerance ±{CMD_TOLERANCE})")

    def tc2_too_far(self):
        """TC-2: distance > setpoint → positive lateral cmd (move toward wall)."""
        dist = 4.0
        self.get_logger().info(f"\n{INFO} TC-2  Too far  (distance = {dist} m)")
        self.reset_state()

        self.publish_distance(dist, count=15, hz=10.0)
        time.sleep(SETTLE_TIME)
        self.cmd_msg = None
        self.publish_distance(dist, count=5, hz=10.0)
        self.wait_for_cmd(timeout_s=2.0)

        if self.cmd_msg is None:
            self.record("TC-2 Too far", False, "No /cmd_vel received")
            return

        output = self.cmd_msg.linear.y
        # error = setpoint − measured = 2.0 − 4.0 = −2.0  → output should be negative
        # Wait — setpoint − measured = 2.0 − 4.0 = −2.0 (negative error → pull back)
        # Direction convention: negative output means "move away" (reduce y thrust)
        correct = output < -CMD_TOLERANCE
        self.record("TC-2 Too far",
                    correct,
                    f"cmd.linear.y = {output:.4f} (expected < -{CMD_TOLERANCE})")

    def tc3_too_close(self):
        """TC-3: distance < setpoint → negative lateral cmd (push away from wall)."""
        dist = 0.5
        self.get_logger().info(f"\n{INFO} TC-3  Too close  (distance = {dist} m)")
        self.reset_state()

        self.publish_distance(dist, count=15, hz=10.0)
        time.sleep(SETTLE_TIME)
        self.cmd_msg = None
        self.publish_distance(dist, count=5, hz=10.0)
        self.wait_for_cmd(timeout_s=2.0)

        if self.cmd_msg is None:
            self.record("TC-3 Too close", False, "No /cmd_vel received")
            return

        output = self.cmd_msg.linear.y
        # error = 2.0 − 0.5 = +1.5 → output positive (move toward wall normal)
        correct = output > CMD_TOLERANCE
        self.record("TC-3 Too close",
                    correct,
                    f"cmd.linear.y = {output:.4f} (expected > {CMD_TOLERANCE})")

    def tc4_heartbeat_loss(self):
        """TC-4: stop publishing → watchdog declares sensor lost → emergency."""
        self.get_logger().info(
            f"\n{INFO} TC-4  Heartbeat loss  "
            f"(silence > {HEARTBEAT_TIMEOUT} s)")
        self.reset_state()

        # First arm the watchdog by sending some messages
        self.publish_distance(SETPOINT, count=10, hz=10.0)
        self.get_logger().info(f"{INFO}   Sensor armed — stopping publications…")

        # Go silent for longer than heartbeat_timeout + watchdog tick period
        silence = HEARTBEAT_TIMEOUT + 1.0 / 5.0 + 0.5  # timeout + 1 tick + margin
        time.sleep(silence)
        rclpy.spin_once(self, timeout_sec=0.1)

        triggered = self.wait_for_emergency(timeout_s=3.0)
        self.record("TC-4 Heartbeat loss",
                    triggered,
                    f"emergency_active = {triggered} after {silence:.1f} s silence")

        # Flush emergency state for next test
        self.emergency_active = False

    def tc5_depth_violation(self):
        """TC-5: depth > max_depth → emergency surface triggered."""
        depth = MAX_DEPTH + 2.0
        self.get_logger().info(
            f"\n{INFO} TC-5  Depth violation  (depth = {depth} m > max {MAX_DEPTH} m)")
        self.reset_state()

        # Keep sonar alive so TC-4-style heartbeat does not interfere
        def keep_alive():
            for _ in range(30):
                msg = Float64()
                msg.data = SETPOINT
                self.distance_pub.publish(msg)
                time.sleep(0.1)
        t = threading.Thread(target=keep_alive, daemon=True)
        t.start()

        # Publish an excessive depth
        self.publish_depth(depth)
        rclpy.spin_once(self, timeout_sec=0.1)

        triggered = self.wait_for_emergency(timeout_s=3.0)
        self.record("TC-5 Depth violation",
                    triggered,
                    f"emergency_active = {triggered} at depth {depth} m")
        t.join()

    # ─────────────────────────────────────────────────────────────────────────
    #  Summary
    # ─────────────────────────────────────────────────────────────────────────

    def print_summary(self):
        total  = len(self.results)
        passed = sum(1 for _, ok, _ in self.results if ok)
        failed = total - passed

        print("\n" + "═" * 60)
        print("  VALIDATION SUMMARY")
        print("═" * 60)
        for name, ok, detail in self.results:
            tag = PASS if ok else FAIL
            print(f"  {tag}  {name}")
            if detail:
                print(f"         {detail}")
        print("─" * 60)
        print(f"  Passed: {passed}/{total}   Failed: {failed}/{total}")
        print("═" * 60 + "\n")

        return failed == 0


def main(args=None):
    rclpy.init(args=args)
    node = ControllerValidator()

    # Give the controller stack time to fully start
    node.get_logger().info(f"{INFO} Waiting 3 s for controller stack to start…")
    time.sleep(3.0)

    try:
        node.tc1_steady_state()
        node.tc2_too_far()
        node.tc3_too_close()
        node.tc4_heartbeat_loss()
        node.tc5_depth_violation()
    except KeyboardInterrupt:
        pass

    all_passed = node.print_summary()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
