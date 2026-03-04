#include "distance_keep_controller/distance_keep_controller.hpp"

#include <algorithm>
#include <chrono>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace distance_keep_controller
{

DistanceKeepController::DistanceKeepController(const rclcpp::NodeOptions & options)
: Node("distance_keep_controller", options),
  integral_(0.0),
  prev_error_(0.0),
  first_run_(true)
{
  // ── Declare configurable parameters ───────────────────────────────────────
  this->declare_parameter<double>("setpoint",   2.0);   // desired distance [m]
  this->declare_parameter<double>("kp",         2.0);
  this->declare_parameter<double>("ki",         0.0);
  this->declare_parameter<double>("kd",         0.5);
  this->declare_parameter<double>("max_output", 50.0);  // clamp limit [N or m/s]

  load_parameters();

  // ── Subscriber: sonar distance ────────────────────────────────────────────
  distance_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/wall_distance",
    rclcpp::QoS(10).reliable(),
    std::bind(&DistanceKeepController::distance_callback, this,
              std::placeholders::_1));

  // ── Publisher: velocity / force command ───────────────────────────────────
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(),
    "DistanceKeepController started | setpoint=%.2f m  kp=%.2f  ki=%.2f  kd=%.2f",
    setpoint_, kp_, ki_, kd_);
}

void DistanceKeepController::load_parameters()
{
  this->get_parameter("setpoint",   setpoint_);
  this->get_parameter("kp",         kp_);
  this->get_parameter("ki",         ki_);
  this->get_parameter("kd",         kd_);
  this->get_parameter("max_output", max_output_);
}

void DistanceKeepController::distance_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  const double measured = msg->data;
  const rclcpp::Time now = this->now();

  // First message: initialise timer, skip control output
  if (first_run_) {
    prev_time_ = now;
    first_run_ = false;
    RCLCPP_DEBUG(this->get_logger(),
      "First sonar reading received: %.3f m", measured);
    return;
  }

  const double dt = (now - prev_time_).seconds();
  prev_time_ = now;

  if (dt <= 0.0) {
    return;
  }

  // ── PID computation ────────────────────────────────────────────────────────
  const double error      = setpoint_ - measured;
  integral_              += error * dt;
  const double derivative = (error - prev_error_) / dt;
  double output           = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // Anti-windup: clamp output and back-calculate integral
  output = std::clamp(output, -max_output_, max_output_);
  prev_error_ = error;

  // ── Publish lateral thrust command ────────────────────────────────────────
  // Positive output => move toward wall (error was positive => too far)
  // linear.y = lateral axis (wall-normal direction)
  geometry_msgs::msg::Twist cmd;
  cmd.linear.y = output;

  cmd_pub_->publish(cmd);

  RCLCPP_DEBUG(this->get_logger(),
    "dist=%.3f m  err=%.3f  out=%.3f", measured, error, output);
}

}  // namespace distance_keep_controller

RCLCPP_COMPONENTS_REGISTER_NODE(distance_keep_controller::DistanceKeepController)
