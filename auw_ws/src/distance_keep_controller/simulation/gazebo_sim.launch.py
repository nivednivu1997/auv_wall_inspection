"""
gazebo_sim.launch.py
════════════════════
Launches the full AUV simulation stack using Gazebo Classic (Gazebo 11)
with gazebo_ros_pkgs on ROS 2 Humble.

Architecture
────────────
  Gazebo Classic (gzserver + gzclient)
    └── underwater_world.world  (buoyancy + hydrodynamics)
    └── AUV model               (BlueROV2-style)
          ├── sonar plugin  → publishes /wall_distance
          └── depth plugin  → publishes /depth

  distance_keep_controller stack
    ├── sonar_lifecycle_node (or replaced by Gazebo plugin)
    ├── distance_keep_controller_node  (PID)
    └── safety_watchdog_node


Usage
─────
  source /opt/ros/humble/setup.bash
  source /home/thedush/auw_ws/install/setup.bash
  ros2 launch distance_keep_controller gazebo_sim.launch.py

  # With debug logging:
  ros2 launch distance_keep_controller gazebo_sim.launch.py log_level:=debug

  # Headless (no GUI):
  ros2 launch distance_keep_controller gazebo_sim.launch.py gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


# ── Inline Gazebo world SDF ──────────────────────────────────────────────────
# Written here for portability; in a real project save as worlds/underwater.world
UNDERWATER_WORLD_SDF = """
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="underwater_world">

    <!-- Physics: use ODE with higher iteration count for stability -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
        </solver>
      </ode>
    </physics>

    <!-- Gravity (underwater: apparent gravity reduced by buoyancy) -->
    <gravity>0 0 -9.81</gravity>

    <!-- Minimal lighting -->
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- Vertical concrete wall at Y = 4 m (vehicle starts at Y ≈ 0) -->
    <model name="inspection_wall">
      <static>true</static>
      <pose>0 4 1.5  0 0 0</pose>
      <link name="wall_link">
        <collision name="wall_collision">
          <geometry><box><size>10 0.2 3</size></box></geometry>
        </collision>
        <visual name="wall_visual">
          <geometry><box><size>10 0.2 3</size></box></geometry>
          <material>
            <ambient>0.5 0.5 0.8 1</ambient>
            <diffuse>0.5 0.5 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ── ROS 2 Gazebo bridge plugins ─────────────────────────────────────
         These require ros-humble-gazebo-ros-pkgs installed.
         In a real AUV model the sonar/depth plugins would be inside
         the robot URDF/SDF, not the world file.
    -->

  </world>
</sdf>
"""

# ── Inline minimal AUV model SDF ─────────────────────────────────────────────
AUV_MODEL_SDF = """
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="auv">
    <pose>0 0 -1.5  0 0 0</pose>

    <link name="base_link">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.2</ixx><iyy>0.5</iyy><izz>0.5</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <collision name="body_col">
        <geometry><cylinder><radius>0.15</radius><length>0.5</length></cylinder></geometry>
      </collision>
      <visual name="body_vis">
        <geometry><cylinder><radius>0.15</radius><length>0.5</length></cylinder></geometry>
        <material><ambient>0.2 0.6 0.9 1</ambient></material>
      </visual>

      <!--
        Sonar plugin: publishes Float64 distance to nearest obstacle
        on /wall_distance.  The ray is cast along the +Y body axis.
        Requires: ros-humble-gazebo-ros-pkgs
      -->
      <sensor name="sonar" type="ray">
        <pose>0 0.15 0  0 0 0</pose>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>1</samples>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>10.0</max>
          </range>
        </ray>
        <plugin name="sonar_plugin" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <remapping>~/out:=/wall_distance</remapping>
          </ros>
          <output_type>sensor_msgs/Range</output_type>
          <frame_name>base_link</frame_name>
        </plugin>
      </sensor>

    </link>

    <!--
      Diff-drive plugin repurposed for AUV lateral thrust:
      subscribes /cmd_vel and applies force to the vehicle body.
      Replace with a proper hydrodynamics plugin (UnderwaterObject) for
      realistic simulation.
    -->
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <remapping>cmd_vel:=/cmd_vel</remapping>
      </ros>
      <left_joint>dummy_left</left_joint>
      <right_joint>dummy_right</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>10</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>

  </model>
</sdf>
"""


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Launch Gazebo GUI (gzclient). Set false for headless.")

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 logger level (debug|info|warn|error)")

    paused_arg = DeclareLaunchArgument(
        "paused", default_value="false",
        description="Start Gazebo paused (useful for debugging)")

    gui       = LaunchConfiguration("gui")
    log_level = LaunchConfiguration("log_level")
    paused    = LaunchConfiguration("paused")

    pkg_share = FindPackageShare("distance_keep_controller")

    # ── Write world file to /tmp for Gazebo to load ───────────────────────────
    write_world = ExecuteProcess(
        cmd=["bash", "-c",
             f"echo '{UNDERWATER_WORLD_SDF}' > /tmp/underwater_world.world"],
        output="screen")

    # ── Gazebo server ─────────────────────────────────────────────────────────
    gzserver = ExecuteProcess(
        cmd=["gzserver",
             "--verbose",
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so",
             "-s", "libgazebo_ros_force_system.so",
             "/tmp/underwater_world.world"],
        output="screen")

    # ── Gazebo client (GUI) ───────────────────────────────────────────────────
    gzclient = ExecuteProcess(
        cmd=["gzclient", "--verbose"],
        condition=IfCondition(gui),
        output="screen")

    # ── Spawn AUV model ───────────────────────────────────────────────────────
    # Waits for gzserver to be ready before spawning
    spawn_auv = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="Spawning AUV model into Gazebo…"),
            ExecuteProcess(
                cmd=["bash", "-c",
                     f"echo '{AUV_MODEL_SDF}' > /tmp/auv_model.sdf && "
                     "ros2 run gazebo_ros spawn_entity.py "
                     "-file /tmp/auv_model.sdf -entity auv "
                     "-x 0 -y 0 -z -1.5"],
                output="screen"),
        ])

    # ── depth topic bridge ────────────────────────────────────────────────────
    # Publishes a simulated depth value since the minimal AUV model above
    # does not include a depth sensor plugin.
    # In a full model, replace with a proper depth sensor plugin.
    depth_publisher = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="distance_keep_controller",
                executable="python3",
                name="depth_sim",
                arguments=["-c",
                    "import rclpy, sys; from rclpy.node import Node; "
                    "from std_msgs.msg import Float64; "
                    "rclpy.init(); n=Node('depth_sim'); "
                    "p=n.create_publisher(Float64,'/depth',10); "
                    "t=n.create_timer(0.1,lambda:"
                    " p.publish(type('m',(object,),{'data':1.5})())); "
                    "rclpy.spin(n)"],
                output="screen",
            )
        ])

    # ── Controller stack ──────────────────────────────────────────────────────
    sonar_node = LifecycleNode(
        package    = "distance_keep_controller",
        executable = "sonar_lifecycle_node",
        name       = "sonar_lifecycle_node",
        namespace  = "",
        parameters = [{"publish_rate": 10.0, "initial_distance": 3.0}],
        arguments  = ["--ros-args", "--log-level", log_level],
        output     = "screen")

    controller_node = Node(
        package    = "distance_keep_controller",
        executable = "distance_keep_controller_node",
        name       = "distance_keep_controller",
        parameters = [{
            "setpoint": 2.0, "kp": 2.0, "ki": 0.0,
            "kd": 0.5, "max_output": 50.0}],
        arguments  = ["--ros-args", "--log-level", log_level],
        output     = "screen")

    watchdog_node = Node(
        package    = "distance_keep_controller",
        executable = "safety_watchdog_node",
        name       = "safety_watchdog",
        parameters = [{
            "max_depth": 10.0, "heartbeat_timeout": 2.0, "watchdog_rate": 5.0}],
        arguments  = ["--ros-args", "--log-level", log_level],
        output     = "screen")

    # ── Lifecycle transitions for sonar node ──────────────────────────────────
    configure_sonar = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="[lifecycle] Configuring sonar_lifecycle_node"),
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set",
                     "/sonar_lifecycle_node", "configure"],
                output="screen")])

    activate_sonar = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg="[lifecycle] Activating sonar_lifecycle_node"),
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set",
                     "/sonar_lifecycle_node", "activate"],
                output="screen")])


    return LaunchDescription([
        # Arguments
        gui_arg, log_level_arg, paused_arg,

        # Simulation
        write_world,
        gzserver,
        gzclient,
        spawn_auv,

        # Controller stack
        sonar_node,
        controller_node,
        watchdog_node,

        # Lifecycle management
        configure_sonar,
        activate_sonar
    ])


# ──────────────────────────────────────────────────────────────────────────────
#  UUV SIMULATOR / PLANKTON QUICK-START GUIDE
# ──────────────────────────────────────────────────────────────────────────────
"""

Validation without Gazebo
──────────────────────────
  # Run the controller stack first:
  ros2 launch distance_keep_controller distance_keep.launch.py

  # Then run the Python validator:
  python3 simulation/validate_controller.py
"""
