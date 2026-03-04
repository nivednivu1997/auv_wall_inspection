"""
distance_keep.launch.py
───────────────────────
Launches the full Distance-Keep Controller stack:

  1. sonar_lifecycle_node   – simulated sonar sensor (Lifecycle Node)
  2. distance_keep_controller_node – PID wall-distance controller
  3. safety_watchdog_node   – depth-limit & heartbeat watchdog

The sonar node is automatically transitioned through:
  Unconfigured -> (configure) -> Inactive -> (activate) -> Active
using timed ExecuteProcess actions (2 s gap each).

To trigger lifecycle transitions manually instead:
  ros2 lifecycle set /sonar_lifecycle_node configure
  ros2 lifecycle set /sonar_lifecycle_node activate
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 logger level (debug|info|warn|error)")

    log_level = LaunchConfiguration("log_level")

    # ── 1. Sonar Lifecycle Node ───────────────────────────────────────────────
    sonar_node = LifecycleNode(
        package    = "distance_keep_controller",
        executable = "sonar_lifecycle_node",
        name       = "sonar_lifecycle_node",
        namespace  = "",
        parameters = [{
            "publish_rate":     10.0,
            "initial_distance": 3.0,
        }],
        arguments  = ["--ros-args", "--log-level", log_level],
        output     = "screen",
    )

    # ── 2. Distance-Keep PID Controller ──────────────────────────────────────
    controller_node = Node(
        package    = "distance_keep_controller",
        executable = "distance_keep_controller_node",
        name       = "distance_keep_controller",
        parameters = [{
            "setpoint":   2.0,
            "kp":         2.0,
            "ki":         0.0,
            "kd":         0.5,
            "max_output": 50.0,
        }],
        arguments  = ["--ros-args", "--log-level", log_level],
        output     = "screen",
    )

    # ── 3. Safety Watchdog ────────────────────────────────────────────────────
    watchdog_node = Node(
        package    = "distance_keep_controller",
        executable = "safety_watchdog_node",
        name       = "safety_watchdog",
        parameters = [{
            "max_depth":         10.0,
            "heartbeat_timeout":  2.0,
            "watchdog_rate":      5.0,
        }],
        arguments  = ["--ros-args", "--log-level", log_level],
        output     = "screen",
    )

    # ── Automatic lifecycle management for sonar node ─────────────────────────
    # Allow 2 s for the node to start before sending configure, then activate.
    configure_sonar = TimerAction(
        period  = 2.0,
        actions = [
            LogInfo(msg="[lifecycle] Sending 'configure' to sonar_lifecycle_node"),
            ExecuteProcess(
                cmd    = ["ros2", "lifecycle", "set",
                          "/sonar_lifecycle_node", "configure"],
                output = "screen",
            ),
        ],
    )

    activate_sonar = TimerAction(
        period  = 4.0,
        actions = [
            LogInfo(msg="[lifecycle] Sending 'activate' to sonar_lifecycle_node"),
            ExecuteProcess(
                cmd    = ["ros2", "lifecycle", "set",
                          "/sonar_lifecycle_node", "activate"],
                output = "screen",
            ),
        ],
    )

    # ── Launch description ────────────────────────────────────────────────────
    return LaunchDescription([
        log_level_arg,
        sonar_node,
        controller_node,
        watchdog_node,
        configure_sonar,
        activate_sonar,
    ])
