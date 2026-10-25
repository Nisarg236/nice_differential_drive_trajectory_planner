#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nice_differential_drive_trajectory_planner/controller_core.hpp"
#include "nice_differential_drive_trajectory_planner/visualization.hpp"

namespace nice_differential_drive_trajectory_planner
{
/** Reads <name>.* into ControllerParams; the footprint comes from the costmap. Throws on invalid values. */
ControllerParams loadParameters(const nav2::LifecycleNode::SharedPtr& node, const std::string& name,
                                const Footprint& footprint, double controller_frequency);

/**
 * nav2 controller plugin around ControllerCore. Plans in the costmap frame on the path handler's pruned plan,
 * turns to the goal heading once the goal checker reports the position reached, and raises NoValidControl when it
 * has braked to a stop without finding a valid candidate for max_fallback_cycles cycles.
 */
class PredictiveController : public nav2_core::Controller
{
public:
  void configure(const nav2::LifecycleNode::WeakPtr& parent, std::string name, nav2::TransformBuffer::SharedPtr tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  void newPathReceived(const nav_msgs::msg::Path& raw_global_path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                           const geometry_msgs::msg::Twist& velocity,
                                                           nav2_core::GoalChecker* goal_checker,
                                                           const nav_msgs::msg::Path& transformed_global_plan,
                                                           const geometry_msgs::msg::PoseStamped& global_goal) override;

  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

private:
  void publish(const ControlOutput& output, const RobotState& state, const rclcpp::Time& stamp);
  unsigned char circumscribedCost() const;

  nav2::LifecycleNode::WeakPtr node_;
  std::string name_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{ rclcpp::get_logger("PredictiveController") };
  rclcpp::Clock::SharedPtr clock_;

  std::unique_ptr<ControllerCore> core_;
  std::mutex mutex_;
  double speed_limit_ = 0.0;
  bool goal_latched_ = false;
  geometry_msgs::msg::PoseStamped latched_goal_;
  bool publish_plan_ = true;
  VisualizationOptions visualization_;
  double diagnostics_period_ = 0.5;
  rclcpp::Time last_diagnostics_{ 0, 0, RCL_ROS_TIME };

  nav2::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  nav2::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  nav2::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

}  // namespace nice_differential_drive_trajectory_planner
