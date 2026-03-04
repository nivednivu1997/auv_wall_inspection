# distance_keep_controller

A ROS 2 C++ package implementing real-time wall-distance keeping for an Autonomous Underwater Vehicle (AUV). The package provides three composable nodes, a Nav2 Behavior Tree mission, a Gazebo simulation stack, and a headless validation harness.

---

## Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
- [Prerequisites](#prerequisites)
- [Building](#building)
- [Nodes](#nodes)
  - [DistanceKeepController](#1-distancekeepcontroller)
  - [SonarLifecycleNode](#2-sonarlifecyclenode)
  - [SafetyWatchdog](#3-safetywatchdog)
- [Topic Graph](#topic-graph)
- [Parameters](#parameters)
- [Launch Files](#launch-files)
- [Behavior Tree](#behavior-tree)
- [Simulation](#simulation)
- [Validation](#validation)
- [Docker](#docker)
- [Runtime Parameter Tuning](#runtime-parameter-tuning)
- [Deployment as Components](#deployment-as-components)

---

## Overview

```
                      ┌──────────────────────────────────────┐
                      │       ROS 2 Component Container       │
                      │                                       │
   /wall_distance ───►│  SonarLifecycleNode  ────────────►   │
   (Float64, metres)  │  (Lifecycle sensor)  /wall_distance  │
                      │                          │            │
                      │  DistanceKeepController ◄┘            │──► /cmd_vel
                      │  (PID, setpoint 2.0 m)               │    (Twist)
                      │                                       │
   /depth ───────────►│  SafetyWatchdog                       │──► /emergency_cmd
   (Float64, metres)  │  (depth + heartbeat)                  │──► /emergency_active
                      └──────────────────────────────────────┘
```

**Key features:**

| Feature | Implementation |
|---|---|
| Real-time PID control | `DistanceKeepController` — event-driven, configurable gains |
| Sensor lifecycle management | `SonarLifecycleNode` — full ROS 2 Lifecycle state machine |
| Safety supervision | `SafetyWatchdog` — depth limit + heartbeat monitor |
| Mission orchestration | Nav2 Behavior Tree (BehaviorTree.CPP v3) |
| Composability | All nodes built as ROS 2 Components (`.so` shared libraries) |
| Simulation | Gazebo Classic 11 launch + headless Python validator |
| Containerisation | Multi-stage Dockerfile (deps → builder → runtime) |

---

## Package Structure

```
distance_keep_controller/
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── include/distance_keep_controller/
│   ├── distance_keep_controller.hpp    # PID node interface
│   ├── sonar_lifecycle_node.hpp        # Lifecycle node interface
│   └── safety_watchdog.hpp             # Watchdog interface
│
├── src/
│   ├── distance_keep_controller.cpp    # PID control loop
│   ├── sonar_lifecycle_node.cpp        # Lifecycle sonar sensor
│   └── safety_watchdog.cpp             # Safety supervisor
│
├── launch/
│   └── distance_keep.launch.py         # Full controller stack
│
├── bt/
│   └── inspection_mission.xml          # Nav2 Behavior Tree (XML)
│
└── simulation/
    ├── validate_controller.py          # Headless test harness (5 test cases)
    └── gazebo_sim.launch.py            # Gazebo Classic simulation stack
```

Workspace root:

```
auw_ws/
├── Dockerfile                          # Multi-stage container definition
├── docker-entrypoint.sh                # Container entrypoint script
└── src/
    └── distance_keep_controller/
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| ROS 2 | Humble Hawksbill (Ubuntu 22.04) |
| CMake | ≥ 3.8 |
| C++ standard | C++17 |
| colcon | any recent |
| Python | 3.10+ (for launch files and validator) |

**Required ROS 2 packages** (all available via `apt`):

```bash
sudo apt install -y \
  ros-humble-rclcpp \
  ros-humble-rclcpp-lifecycle \
  ros-humble-rclcpp-components \
  ros-humble-std-msgs \
  ros-humble-geometry-msgs
```

**Optional — for Gazebo simulation:**

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  gazebo
```

**Optional — for Nav2 Behavior Tree execution:**

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-behaviortree-cpp-v3
```

---

## Building

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Navigate to workspace root
cd ~/auw_ws

# 3. Install rosdep dependencies
rosdep install --from-paths src --ignore-src -y

# 4. Build
colcon build --packages-select distance_keep_controller \
             --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 5. Source the workspace
source install/setup.bash
```

Verify the build produced three shared libraries and three executables:

```bash
ls install/distance_keep_controller/lib/
# libdistance_keep_controller_component.so
# libsafety_watchdog_component.so
# libsonar_lifecycle_component.so

ls install/distance_keep_controller/lib/distance_keep_controller/
# distance_keep_controller_node
# safety_watchdog_node
# sonar_lifecycle_node
```

---

## Nodes

### 1. DistanceKeepController

**File:** [src/distance_keep_controller.cpp](src/distance_keep_controller.cpp)

Implements a discrete-time PID feedback controller. Subscribes to sonar distance readings and publishes a lateral thrust command to maintain a configurable setpoint distance from the wall.

**Control equation:**

```
error      = setpoint − measured_distance
integral  += error × dt
derivative = (error − prev_error) / dt

output = Kp·error  +  Ki·integral  +  Kd·derivative
output = clamp(output, −max_output, +max_output)

cmd.linear.y = output      ← lateral (wall-normal) axis
```

**Design decisions:**

- **Event-driven** — the control loop fires on each incoming sonar message, not on a fixed timer. Sample rate equals sensor publish rate.
- **First-run guard** — the first message only initialises the timestamp; no command is published until `dt` can be computed, preventing a derivative spike.
- **`dt ≤ 0` guard** — skips cycles where the clock has not advanced (duplicate messages, sim pause), avoiding division by zero.
- **Hard clamp** — output is clamped before publishing to prevent sending unrealisable commands to thrusters.

---

### 2. SonarLifecycleNode

**File:** [src/sonar_lifecycle_node.cpp](src/sonar_lifecycle_node.cpp)

Simulates a sonar sensor with full ROS 2 Lifecycle state machine management. Ensures the sensor is properly configured before it begins publishing data to the controller.

**State machine:**

```
                ┌────────────────────┐
    [created]   │    UNCONFIGURED    │ ◄──────── on_cleanup() SUCCESS
                └─────────┬──────────┘
                          │ configure
                          ▼
                ┌────────────────────┐
                │      INACTIVE      │ ◄── on_deactivate() SUCCESS
                └─────────┬──────────┘
                          │ activate
                          ▼
                ┌────────────────────┐
                │      ACTIVE        │ — publishing /wall_distance
                └────────────────────┘
                          │ (any state)
                          ▼
                ┌────────────────────┐
                │     FINALIZED      │ ← on_shutdown() SUCCESS
                └────────────────────┘
```

**State callback responsibilities:**

| Callback | Action | Outcome |
|---|---|---|
| `on_configure` | Declare parameters, create publisher + timer (paused) | `UNCONFIGURED → INACTIVE` |
| `on_activate` | Start timer, activate managed publisher | `INACTIVE → ACTIVE` |
| `on_deactivate` | Cancel timer, deactivate managed publisher | `ACTIVE → INACTIVE` |
| `on_cleanup` | Destroy publisher and timer | `INACTIVE → UNCONFIGURED` |
| `on_shutdown` | Release all resources | `any → FINALIZED` |

**Simulated distance model** (Active state):

```
distance(t) = initial_distance + 1.5 × sin(0.2 × t)
```

With `initial_distance = 3.0 m`: range is **1.5 m → 4.5 m**, period **≈ 31 s**.
This exercises both sides of the PID error (too close and too far from setpoint).

**Trigger lifecycle transitions manually:**

```bash
ros2 lifecycle set /sonar_lifecycle_node configure
ros2 lifecycle set /sonar_lifecycle_node activate
ros2 lifecycle set /sonar_lifecycle_node deactivate
ros2 lifecycle set /sonar_lifecycle_node cleanup
```

**List current state:**

```bash
ros2 lifecycle get /sonar_lifecycle_node
```

---

### 3. SafetyWatchdog

**File:** [src/safety_watchdog.cpp](src/safety_watchdog.cpp)

An independent safety supervisor that monitors two failure conditions and triggers an **Emergency Surface** manoeuvre when either is detected. The watchdog latches — once triggered, it continuously republishes the emergency command until the node is restarted.

**Trigger conditions:**

| Condition | Mechanism | Topic |
|---|---|---|
| Depth exceeded | `depth_callback` — synchronous check on every message | `/depth` |
| Sensor heartbeat lost | `watchdog_tick` — periodic timer checks time since last `/wall_distance` | `/wall_distance` |

**Emergency Surface command:**

```
/emergency_cmd  →  Twist { linear.z = 1.0 }   ← full upward thrust
/emergency_active → Bool { data = true }
```

**Heartbeat timeline example:**

```
t = 0.0 s   sonar message → last_sensor_stamp_ updated
t = 1.0 s   sonar message → last_sensor_stamp_ updated
t = 2.0 s   sonar goes silent
t = 2.2 s   watchdog ticks → elapsed = 0.2 s  (OK)
t = 4.2 s   watchdog ticks → elapsed = 2.2 s  > 2.0 s timeout → EMERGENCY
```

**Startup grace period:** The heartbeat check is not armed until at least one sonar message has been received (`sensor_ever_received_ = true`). This prevents a false trigger during the lifecycle node's startup delay.

---

## Topic Graph

```
┌───────────────────────┐     /wall_distance (Float64)
│  SonarLifecycleNode   │ ─────────────────────────────┐
└───────────────────────┘                              │
                                                       ▼
                                         ┌─────────────────────────┐      /cmd_vel (Twist)
                                         │ DistanceKeepController  │ ──────────────────────►
                                         └─────────────────────────┘
                                                       ▲
                                                       │ /wall_distance (shared)
                                                       │
┌───────────────────────┐     /wall_distance           │         /emergency_cmd (Twist)
│   SafetyWatchdog      │ ────────────────────────────►│ ───────────────────────────────►
│                       │     /depth (Float64)          │         /emergency_active (Bool)
└───────────────────────┘ ◄───────────────────────────  ─────────────────────────────────►
```

**Full topic table:**

| Topic | Type | Direction | Publisher | Subscriber(s) |
|---|---|---|---|---|
| `/wall_distance` | `std_msgs/Float64` | sensor → controller | `SonarLifecycleNode` | `DistanceKeepController`, `SafetyWatchdog` |
| `/depth` | `std_msgs/Float64` | sensor → watchdog | depth sensor / simulator | `SafetyWatchdog` |
| `/cmd_vel` | `geometry_msgs/Twist` | controller → thruster | `DistanceKeepController` | thruster driver |
| `/emergency_cmd` | `geometry_msgs/Twist` | watchdog → thruster | `SafetyWatchdog` | thruster driver |
| `/emergency_active` | `std_msgs/Bool` | watchdog → system | `SafetyWatchdog` | mission planner |

---

## Parameters

### DistanceKeepController

| Parameter | Type | Default | Description |
|---|---|---|---|
| `setpoint` | `double` | `2.0` | Desired wall distance [m] |
| `kp` | `double` | `2.0` | Proportional gain |
| `ki` | `double` | `0.0` | Integral gain |
| `kd` | `double` | `0.5` | Derivative gain |
| `max_output` | `double` | `50.0` | Output saturation limit [N or m/s] |

### SonarLifecycleNode

| Parameter | Type | Default | Description |
|---|---|---|---|
| `publish_rate` | `double` | `10.0` | Sensor publish frequency [Hz] |
| `initial_distance` | `double` | `3.0` | Centre of simulated sinusoid [m] |

> Parameters are declared in `on_configure`, not the constructor — they are only visible in the ROS 2 parameter server after the `configure` transition.

### SafetyWatchdog

| Parameter | Type | Default | Description |
|---|---|---|---|
| `max_depth` | `double` | `10.0` | Maximum allowed depth [m, positive deeper] |
| `heartbeat_timeout` | `double` | `2.0` | Sonar silence threshold [s] |
| `watchdog_rate` | `double` | `5.0` | Heartbeat check frequency [Hz] |

---

## Launch Files

### Controller stack (standard)

```bash
source install/setup.bash
ros2 launch distance_keep_controller distance_keep.launch.py
```

**Optional arguments:**

```bash
# Change log verbosity
ros2 launch distance_keep_controller distance_keep.launch.py log_level:=debug
```

This launch file:
1. Starts all three nodes
2. Automatically sends `configure` to `sonar_lifecycle_node` at t = 2 s
3. Automatically sends `activate` to `sonar_lifecycle_node` at t = 4 s

### Gazebo simulation

```bash
ros2 launch distance_keep_controller gazebo_sim.launch.py

# Headless (no GUI)
ros2 launch distance_keep_controller gazebo_sim.launch.py gui:=false

# With debug logging
ros2 launch distance_keep_controller gazebo_sim.launch.py gui:=false log_level:=debug
```

---

## Behavior Tree

**File:** [bt/inspection_mission.xml](bt/inspection_mission.xml)

A Nav2 / BehaviorTree.CPP v3 XML tree for a complete AUV inspection mission.

### Mission phases

```
RootWithSafety (ReactiveSequence)           ← safety re-checked every tick
├── SafetyMonitor (ReactiveFallback)
│   ├── DepthWithinLimits  [C]              ← depth > 10 m? → FAILURE
│   ├── SonarHeartbeatOK   [C]              ← silence > 2 s? → FAILURE
│   └── EmergencySurface   [A]              ← fires on FAILURE, stays RUNNING
│
└── MissionSequence (Sequence)
    ├── Phase 0 – PreFlightChecks           ← verify all systems
    ├── Phase 1 – SearchWithRecovery        ← lawnmower pattern + 360° scan
    │             (RecoveryNode, 3 retries)
    ├── Phase 2 – ApproachPhase             ← navigate to 2 m standoff
    ├── Phase 3 – InspectionWithRecovery    ← PID active, traverse wall
    │             (RecoveryNode, 2 retries)
    └── Phase 4 – ReturnPhase               ← navigate home
```

### Custom BT node types

Nodes marked ✦ require implementation as BT plugins:

| Node ID | Type | Description |
|---|---|---|
| `DepthWithinLimits` | Condition | Returns SUCCESS if `depth ≤ max_depth` |
| `SonarHeartbeatOK` | Condition | Returns SUCCESS if sonar messaged within `timeout_sec` |
| `WallDetected` | Condition | Returns SUCCESS if sonar reading `< max_detection_range` |
| `DistanceAtSetpoint` | Condition | Returns SUCCESS if `\|distance − setpoint\| ≤ tolerance` |
| `EmergencySurface` | Action | Publishes upward Twist; returns RUNNING indefinitely |
| `ActivateDistanceController` | Action | Enables PID output via ROS 2 parameter service |
| `DeactivateDistanceController` | Action | Zeros PID output |

Standard Nav2 nodes used: `NavigateToPose`, `NavigateThroughPoses`, `Spin`, `Wait`, `BackUp`, `ClearEntireCostmap`, `SetBlackboard`, `RecoveryNode`.

### Visualising with Groot

```bash
# Install
sudo apt install ros-humble-groot

# Open the tree
groot

# File → Load → bt/inspection_mission.xml
```

### Loading in Nav2

```python
# In your Nav2 BT navigator configuration (nav2_params.yaml):
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "<path>/bt/inspection_mission.xml"
```

---

## Simulation

### Gazebo Classic (gazebo_sim.launch.py)

The Gazebo launch sets up:

- An **underwater world** with a vertical concrete inspection wall at Y = 4 m
- A minimal **AUV model** (cylinder body) with a forward-facing ray-sensor sonar plugin
- The sonar plugin bridges to `/wall_distance` via `libgazebo_ros_ray_sensor.so`
- `rqt_plot` opens automatically showing `/wall_distance`, `/cmd_vel/linear/y`, `/depth`

**Timeline after launch:**

```
t = 0 s   gzserver starts, controller nodes launch
t = 3 s   AUV model spawned at (0, 0, −1.5)
t = 5 s   sonar lifecycle → configure
t = 7 s   sonar lifecycle → activate  → /wall_distance begins publishing
t = 7 s+  PID controller and watchdog fully operational
```


## Validation

Runs 5 automated test cases without Gazebo by publishing synthetic sensor data:

```bash
# Terminal 1 — start controller stack
ros2 launch distance_keep_controller distance_keep.launch.py

# Terminal 2 — run tests
python3 src/distance_keep_controller/simulation/validate_controller.py
```

**Test cases:**

| # | Name | Stimulus | Pass condition |
|---|---|---|---|
| TC-1 | Steady state | `distance = 2.0 m` (setpoint) | `\|cmd.linear.y\| ≤ 0.1` |
| TC-2 | Too far | `distance = 4.0 m` | `cmd.linear.y < −0.1` |
| TC-3 | Too close | `distance = 0.5 m` | `cmd.linear.y > +0.1` |
| TC-4 | Heartbeat loss | Stop publishing `/wall_distance` for > 2 s | `/emergency_active = true` |
| TC-5 | Depth violation | Publish `depth = 12.0 m` (> max 10.0) | `/emergency_active = true` |

**Expected output:**

```
[PASS]  TC-1 Steady state         cmd.linear.y = 0.0000 (tolerance ±0.1)
[PASS]  TC-2 Too far              cmd.linear.y = -4.0321 (expected < -0.1)
[PASS]  TC-3 Too close            cmd.linear.y = 3.0180 (expected > 0.1)
[PASS]  TC-4 Heartbeat loss       emergency_active = True after 2.7 s silence
[PASS]  TC-5 Depth violation      emergency_active = True at depth 12.0 m
════════════════════════════════════════════════════════════
  Passed: 5/5   Failed: 0/5
════════════════════════════════════════════════════════════
```

Exit code `0` = all passed, `1` = one or more failed.

---

## Docker

### Build

```bash
cd ~/auw_ws
docker build -t auv_controller:humble .
```

### Run — interactive development shell

```bash
docker run -it --rm \
    -v $(pwd)/src:/ws/src \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --network host \
    auv_controller:humble bash
```

### Run — launch controller stack

```bash
docker run -it --rm --network host auv_controller:humble
```

### Run — validation tests

```bash
docker run -it --rm --network host auv_controller:humble bash -c \
    "ros2 launch distance_keep_controller distance_keep.launch.py & \
     sleep 5 && \
     python3 /ws/src/distance_keep_controller/simulation/validate_controller.py"
```

### Run — headless Gazebo simulation

```bash
docker run -it --rm --network host auv_controller:humble \
    ros2 launch distance_keep_controller gazebo_sim.launch.py gui:=false
```

### Image stages

| Stage | Purpose | Contents |
|---|---|---|
| `deps` | Shared base | ROS 2 Humble + Nav2 + Gazebo 11 + rqt tools |
| `builder` | Compile | Copies `src/`, runs `rosdep install`, `colcon build` |
| `runtime` | Deploy | Only `install/` directory — minimal footprint |

---

## Runtime Parameter Tuning

All parameters can be changed at runtime without restarting nodes:

```bash
# Tighten PID gains
ros2 param set /distance_keep_controller kp 3.5
ros2 param set /distance_keep_controller kd 0.8
ros2 param set /distance_keep_controller ki 0.05

# Change setpoint
ros2 param set /distance_keep_controller setpoint 1.5

# Tighten safety limits
ros2 param set /safety_watchdog max_depth 8.0
ros2 param set /safety_watchdog heartbeat_timeout 1.0

# List all parameters
ros2 param list /distance_keep_controller
ros2 param dump /distance_keep_controller
```

---

## Deployment as Components

Each node is built as a ROS 2 Component (shared library), enabling intra-process communication with zero serialisation overhead when loaded into a shared `ComponentManager`:

```bash
# Start a component container
ros2 run rclcpp_components component_container --ros-args -r __node:=AUVContainer

# Load all three components into the same process
ros2 component load /AUVContainer \
    distance_keep_controller distance_keep_controller::DistanceKeepController

ros2 component load /AUVContainer \
    distance_keep_controller distance_keep_controller::SonarLifecycleNode

ros2 component load /AUVContainer \
    distance_keep_controller distance_keep_controller::SafetyWatchdog

# List loaded components
ros2 component list
```

---

## License

Apache-2.0 — see [package.xml](package.xml).
