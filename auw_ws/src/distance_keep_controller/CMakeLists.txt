cmake_minimum_required(VERSION 3.8)
project(distance_keep_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclcpp_lifecycle   REQUIRED)
find_package(rclcpp_components  REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(geometry_msgs      REQUIRED)

# ── Helper macro: apply common settings to a component library ──────────────
macro(configure_component target)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${target} PUBLIC cxx_std_17)
endmacro()

# ── Component 1: DistanceKeepController (PID wall-distance control) ─────────
add_library(distance_keep_controller_component SHARED
  src/distance_keep_controller.cpp)
configure_component(distance_keep_controller_component)
ament_target_dependencies(distance_keep_controller_component
  rclcpp rclcpp_components std_msgs geometry_msgs)
rclcpp_components_register_node(
  distance_keep_controller_component
  PLUGIN     "distance_keep_controller::DistanceKeepController"
  EXECUTABLE distance_keep_controller_node)

# ── Component 2: SonarLifecycleNode (Lifecycle-managed sonar sensor) ────────
add_library(sonar_lifecycle_component SHARED
  src/sonar_lifecycle_node.cpp)
configure_component(sonar_lifecycle_component)
ament_target_dependencies(sonar_lifecycle_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs)
rclcpp_components_register_node(
  sonar_lifecycle_component
  PLUGIN     "distance_keep_controller::SonarLifecycleNode"
  EXECUTABLE sonar_lifecycle_node)

# ── Component 3: SafetyWatchdog (depth + heartbeat monitor) ─────────────────
add_library(safety_watchdog_component SHARED
  src/safety_watchdog.cpp)
configure_component(safety_watchdog_component)
ament_target_dependencies(safety_watchdog_component
  rclcpp rclcpp_components std_msgs geometry_msgs)
rclcpp_components_register_node(
  safety_watchdog_component
  PLUGIN     "distance_keep_controller::SafetyWatchdog"
  EXECUTABLE safety_watchdog_node)

# ── Install libraries ────────────────────────────────────────────────────────
install(TARGETS
  distance_keep_controller_component
  sonar_lifecycle_component
  safety_watchdog_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY bt/
  DESTINATION share/${PROJECT_NAME}/bt)

install(DIRECTORY simulation/
  DESTINATION share/${PROJECT_NAME}/simulation
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.xml"
    PATTERN "*.yaml")

# ── Python scripts (directly executable) ────────────────────────────────────
install(PROGRAMS
  simulation/validate_controller.py
  DESTINATION lib/${PROJECT_NAME})

# ── Export ───────────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(
  distance_keep_controller_component
  sonar_lifecycle_component
  safety_watchdog_component)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs geometry_msgs)

ament_package()
