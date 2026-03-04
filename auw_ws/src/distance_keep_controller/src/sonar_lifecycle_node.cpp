#include "distance_keep_controller/sonar_lifecycle_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace distance_keep_controller
{

SonarLifecycleNode::SonarLifecycleNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sonar_lifecycle_node", options),
  publish_rate_(10.0),
  initial_distance_(3.0)
{
  RCLCPP_INFO(this->get_logger(),
    "SonarLifecycleNode created  [state: Unconfigured]");
}

// ── on_configure ─────────────────────────────────────────────────────────────
// Allocate resources; node moves Unconfigured → Inactive on SUCCESS.
CallbackReturn SonarLifecycleNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring sonar node…");

  this->declare_parameter<double>("publish_rate",      10.0);
  this->declare_parameter<double>("initial_distance",   3.0);
  this->get_parameter("publish_rate",     publish_rate_);
  this->get_parameter("initial_distance", initial_distance_);

  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "publish_rate must be > 0");
    return CallbackReturn::FAILURE;
  }

  // Lifecycle publisher: stays inactive (silent) until on_activate.
  // LifecycleNode::create_publisher returns a LifecyclePublisher in Humble.
  sonar_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/wall_distance", rclcpp::QoS(10));

  // Timer is created but immediately cancelled – activated in on_activate
  auto period = std::chrono::milliseconds(
    static_cast<int>(1000.0 / publish_rate_));
  timer_ = this->create_wall_timer(
    period,
    std::bind(&SonarLifecycleNode::publish_sonar_data, this));
  timer_->cancel();

  RCLCPP_INFO(this->get_logger(),
    "Sonar configured  [rate=%.1f Hz  initial_dist=%.2f m]  → Inactive",
    publish_rate_, initial_distance_);
  return CallbackReturn::SUCCESS;
}

// ── on_activate ──────────────────────────────────────────────────────────────
// Enable the lifecycle publisher and start the publish timer.
// Node moves Inactive → Active on SUCCESS.
CallbackReturn SonarLifecycleNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Activating sonar node…");

  // Call base: activates all managed entities (lifecycle publisher)
  LifecycleNode::on_activate(state);

  timer_->reset();  // un-cancel: start publishing

  RCLCPP_INFO(this->get_logger(),
    "Sonar ACTIVE – publishing /wall_distance @ %.1f Hz", publish_rate_);
  return CallbackReturn::SUCCESS;
}

// ── on_deactivate ────────────────────────────────────────────────────────────
// Stop publishing; node moves Active → Inactive on SUCCESS.
CallbackReturn SonarLifecycleNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating sonar node…");

  timer_->cancel();
  LifecycleNode::on_deactivate(state);  // deactivates managed entities

  RCLCPP_INFO(this->get_logger(), "Sonar INACTIVE  → Inactive");
  return CallbackReturn::SUCCESS;
}

// ── on_cleanup ───────────────────────────────────────────────────────────────
// Release all resources; node moves Inactive → Unconfigured on SUCCESS.
CallbackReturn SonarLifecycleNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up sonar node…");

  timer_.reset();
  sonar_pub_.reset();

  RCLCPP_INFO(this->get_logger(), "Sonar cleaned up  → Unconfigured");
  return CallbackReturn::SUCCESS;
}

// ── on_shutdown ──────────────────────────────────────────────────────────────
// Called from any state; release remaining resources gracefully.
CallbackReturn SonarLifecycleNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down sonar node…");

  timer_.reset();
  sonar_pub_.reset();

  return CallbackReturn::SUCCESS;
}

// ── Publish helper ────────────────────────────────────────────────────────────
// Simulates a sinusoidally varying wall distance (1.5 – 4.5 m at 0.05 rad/s).
void SonarLifecycleNode::publish_sonar_data()
{
  if (!sonar_pub_->is_activated()) {
    return;
  }

  // Slow sinusoidal variation around initial_distance_
  static double t = 0.0;
  t += 1.0 / publish_rate_;
  const double distance = initial_distance_ + 1.5 * std::sin(0.2 * t);

  std_msgs::msg::Float64 msg;
  msg.data = distance;
  sonar_pub_->publish(msg);

  RCLCPP_DEBUG(this->get_logger(), "Sonar: %.3f m", distance);
}

}  // namespace distance_keep_controller

RCLCPP_COMPONENTS_REGISTER_NODE(distance_keep_controller::SonarLifecycleNode)
