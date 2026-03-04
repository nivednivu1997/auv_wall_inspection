#ifndef DISTANCE_KEEP_CONTROLLER__DISTANCE_KEEP_CONTROLLER_HPP_
#define DISTANCE_KEEP_CONTROLLER__DISTANCE_KEEP_CONTROLLER_HPP_

#include <memory>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace distance_keep_controller
{

/**
 * @brief PID-based wall-distance keep controller.
 *
 * Subscribes to /wall_distance (simulated sonar, Float64 [m]) and publishes
 * /cmd_vel (Twist) such that the vehicle maintains `setpoint` metres from
 * the wall.  All gains are tunable via ROS 2 parameters at runtime.
 *
 * Registered as a ROS 2 Component (shared library).
 */
class DistanceKeepController : public rclcpp::Node
{
public:
  explicit DistanceKeepController(const rclcpp::NodeOptions & options);

private:
  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  // ── PID state ─────────────────────────────────────────────────────────────
  double setpoint_;
  double kp_, ki_, kd_, max_output_;
  double integral_;
  double prev_error_;
  bool   first_run_;
  rclcpp::Time prev_time_;

  // ── Helpers ───────────────────────────────────────────────────────────────
  void load_parameters();
  void distance_callback(const std_msgs::msg::Float64::SharedPtr msg);
};

}  // namespace distance_keep_controller

#endif  // DISTANCE_KEEP_CONTROLLER__DISTANCE_KEEP_CONTROLLER_HPP_
