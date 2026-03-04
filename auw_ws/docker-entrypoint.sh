#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace if available
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# ROS environment
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

echo "────────────────────────────────────────"
echo "ROS_DISTRO         : $ROS_DISTRO"
echo "ROS_DOMAIN_ID      : $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION : $RMW_IMPLEMENTATION"
echo "────────────────────────────────────────"

# If no command passed → launch gazebo simulation
if [ $# -eq 0 ]; then
    echo "Launching Gazebo Simulation..."
    exec ros2 launch distance_keep_controller gazebo_sim.launch.py
else
    exec "$@"
fi
