#ifndef DISTANCE_KEEP_CONTROLLER__SONAR_LIFECYCLE_NODE_HPP_
#define DISTANCE_KEEP_CONTROLLER__SONAR_LIFECYCLE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64.hpp"

namespace distance_keep_controller
{

/**
 * @brief Lifecycle-managed simulated sonar sensor node.
 *
 * Implements the full ROS 2 Lifecycle state machine so that the sensor
 * startup sequence is explicitly managed:
 *
 *   Unconfigured ──configure──> Inactive ──activate──> Active
 *                                        <─deactivate──
 *                <──cleanup────
 *
 * In the Active state the node publishes simulated wall-distance readings
 * on /wall_distance at the configured rate.
 *
 * Registered as a ROS 2 Component (shared library).
 */
class SonarLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SonarLifecycleNode(const rclcpp::NodeOptions & options);

  // ── Lifecycle callbacks ───────────────────────────────────────────────────
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void publish_sonar_data();

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr sonar_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double publish_rate_;      ///< Hz
  double initial_distance_;  ///< Starting simulated distance [m]
};

}  // namespace distance_keep_controller

#endif  // DISTANCE_KEEP_CONTROLLER__SONAR_LIFECYCLE_NODE_HPP_
