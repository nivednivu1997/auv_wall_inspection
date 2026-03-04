#ifndef DISTANCE_KEEP_CONTROLLER__SAFETY_WATCHDOG_HPP_
#define DISTANCE_KEEP_CONTROLLER__SAFETY_WATCHDOG_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace distance_keep_controller
{

/**
 * @brief Safety Watchdog – monitors depth limits and sensor heartbeat.
 *
 * Triggers an "Emergency Surface" action (publishes a full upward Twist on
 * /emergency_cmd and a flag on /emergency_active) when:
 *   1. The vehicle depth exceeds `max_depth` (from /depth topic).
 *   2. The sonar heartbeat on /wall_distance is lost for > `heartbeat_timeout`
 *      seconds.
 *
 * Once triggered the watchdog latches and continues publishing the emergency
 * command at watchdog_rate Hz until the node is restarted.
 *
 * Registered as a ROS 2 Component (shared library).
 */
class SafetyWatchdog : public rclcpp::Node
{
public:
  explicit SafetyWatchdog(const rclcpp::NodeOptions & options);

private:
  // ── Callbacks ─────────────────────────────────────────────────────────────
  void depth_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void distance_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void watchdog_tick();

  // ── Actions ───────────────────────────────────────────────────────────────
  void trigger_emergency(const std::string & reason);

  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr emergency_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr       emergency_flag_pub_;
  rclcpp::TimerBase::SharedPtr                            watchdog_timer_;

  // ── Parameters ────────────────────────────────────────────────────────────
  double max_depth_;           ///< [m] depth magnitude limit (positive value)
  double heartbeat_timeout_;   ///< [s] max silence before declaring sensor lost
  double watchdog_rate_;       ///< [Hz] watchdog tick rate

  // ── State ─────────────────────────────────────────────────────────────────
  rclcpp::Time last_sensor_stamp_;
  bool         sensor_ever_received_;
  bool         emergency_triggered_;
};

}  // namespace distance_keep_controller

#endif  // DISTANCE_KEEP_CONTROLLER__SAFETY_WATCHDOG_HPP_
