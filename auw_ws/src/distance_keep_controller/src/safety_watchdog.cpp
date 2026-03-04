#include "distance_keep_controller/safety_watchdog.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace distance_keep_controller
{

SafetyWatchdog::SafetyWatchdog(const rclcpp::NodeOptions & options)
: Node("safety_watchdog", options),
  sensor_ever_received_(false),
  emergency_triggered_(false)
{
  // ── Parameters ─────────────────────────────────────────────────────────────
  this->declare_parameter<double>("max_depth",          10.0);  // [m] magnitude
  this->declare_parameter<double>("heartbeat_timeout",   2.0);  // [s]
  this->declare_parameter<double>("watchdog_rate",       5.0);  // [Hz]

  this->get_parameter("max_depth",         max_depth_);
  this->get_parameter("heartbeat_timeout", heartbeat_timeout_);
  this->get_parameter("watchdog_rate",     watchdog_rate_);

  // ── Subscribers ────────────────────────────────────────────────────────────
  // /depth  : vehicle depth below surface [m, positive = deeper]
  depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/depth", rclcpp::QoS(10).reliable(),
    std::bind(&SafetyWatchdog::depth_callback, this, std::placeholders::_1));

  // /wall_distance : sonar heartbeat monitor
  distance_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/wall_distance", rclcpp::QoS(10).reliable(),
    std::bind(&SafetyWatchdog::distance_callback, this, std::placeholders::_1));

  // ── Publishers ─────────────────────────────────────────────────────────────
  // Emergency Twist: full upward thrust (linear.z = 1.0)
  emergency_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/emergency_cmd", rclcpp::QoS(10).reliable());

  // Boolean flag so other nodes can react
  emergency_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/emergency_active", rclcpp::QoS(10).reliable());

  // ── Watchdog timer ──────────────────────────────────────────────────────────
  auto period = std::chrono::milliseconds(
    static_cast<int>(1000.0 / watchdog_rate_));
  watchdog_timer_ = this->create_wall_timer(
    period, std::bind(&SafetyWatchdog::watchdog_tick, this));

  RCLCPP_INFO(this->get_logger(),
    "SafetyWatchdog started | max_depth=%.1f m  heartbeat_timeout=%.1f s",
    max_depth_, heartbeat_timeout_);
}

// ── depth_callback ────────────────────────────────────────────────────────────
void SafetyWatchdog::depth_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  const double depth = msg->data;  // positive = deeper below surface

  if (depth > max_depth_) {
    trigger_emergency(
      "Depth limit exceeded: " + std::to_string(depth) +
      " m > max " + std::to_string(max_depth_) + " m");
  }
}

// ── distance_callback ─────────────────────────────────────────────────────────
// Simply records receipt time – used by watchdog_tick to detect heartbeat loss.
void SafetyWatchdog::distance_callback(
  const std_msgs::msg::Float64::SharedPtr /*msg*/)
{
  last_sensor_stamp_     = this->now();
  sensor_ever_received_  = true;
}

// ── watchdog_tick ─────────────────────────────────────────────────────────────
// Periodically verifies the sonar heartbeat is alive.
void SafetyWatchdog::watchdog_tick()
{
  if (emergency_triggered_) {
    // Latch: keep sending emergency command
    trigger_emergency("Emergency latched");
    return;
  }

  if (!sensor_ever_received_) {
    // Haven't received even one reading yet – don't false-trigger immediately,
    // but warn after a startup grace period of 2× heartbeat_timeout
    const double uptime = (this->now() - this->get_clock()->now()).seconds();
    (void)uptime;
    // Grace period: wait for first message before arming heartbeat check
    return;
  }

  const double elapsed = (this->now() - last_sensor_stamp_).seconds();
  if (elapsed > heartbeat_timeout_) {
    trigger_emergency(
      "Sonar heartbeat lost: no message for " +
      std::to_string(elapsed) + " s (timeout=" +
      std::to_string(heartbeat_timeout_) + " s)");
  }
}

// ── trigger_emergency ─────────────────────────────────────────────────────────
void SafetyWatchdog::trigger_emergency(const std::string & reason)
{
  if (!emergency_triggered_) {
    RCLCPP_ERROR(this->get_logger(),
      "=== EMERGENCY SURFACE TRIGGERED === Reason: %s", reason.c_str());
    emergency_triggered_ = true;
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Emergency active – %s", reason.c_str());
  }

  // ── Emergency Surface command ──────────────────────────────────────────────
  // Zero all thrusters except full positive Z (upward)
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = 0.0;
  cmd.linear.y  = 0.0;
  cmd.linear.z  = 1.0;   // full upward thrust
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;
  emergency_cmd_pub_->publish(cmd);

  std_msgs::msg::Bool flag;
  flag.data = true;
  emergency_flag_pub_->publish(flag);
}

}  // namespace distance_keep_controller

RCLCPP_COMPONENTS_REGISTER_NODE(distance_keep_controller::SafetyWatchdog)
