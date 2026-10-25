#include "nice_differential_drive_trajectory_planner/predictive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/inflation_layer_interface.hpp>
#include <nav2_util/robot_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.hpp>

namespace nice_differential_drive_trajectory_planner
{
namespace
{
template <typename T>
T declared(const nav2::LifecycleNode::SharedPtr& node, const std::string& name, const std::string& key,
           const T& default_value)
{
  return node->declare_or_get_parameter(name + "." + key, default_value);
}

size_t declaredSize(const nav2::LifecycleNode::SharedPtr& node, const std::string& name, const std::string& key,
                    size_t default_value)
{
  const int64_t value = declared(node, name, key, static_cast<int64_t>(default_value));
  if (value < 0)
  {
    throw std::invalid_argument(name + "." + key + " must not be negative");
  }
  return static_cast<size_t>(value);
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string& key, const std::string& value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string& key, double value)
{
  return keyValue(key, std::to_string(value));
}
}  // namespace

ControllerParams loadParameters(const nav2::LifecycleNode::SharedPtr& node, const std::string& name,
                                const Footprint& footprint, double controller_frequency)
{
  ControllerParams p;
  p.footprint = footprint;

  p.limits.max_v = declared(node, name, "limits.max_v", p.limits.max_v);
  p.limits.max_omega = declared(node, name, "limits.max_omega", p.limits.max_omega);
  p.limits.max_accel = declared(node, name, "limits.max_accel", p.limits.max_accel);
  p.limits.max_brake = declared(node, name, "limits.max_brake", p.limits.max_brake);
  p.limits.max_angular_accel = declared(node, name, "limits.max_angular_accel", p.limits.max_angular_accel);
  p.limits.wheel_radius = declared(node, name, "limits.wheel_radius", p.limits.wheel_radius);
  p.limits.wheel_separation = declared(node, name, "limits.wheel_separation", p.limits.wheel_separation);
  p.limits.max_wheel_speed = declared(node, name, "limits.max_wheel_speed", p.limits.max_wheel_speed);

  auto& d = p.nominal_dynamics;
  d.linear_time_constant = declared(node, name, "dynamics.linear_time_constant", d.linear_time_constant);
  d.angular_time_constant = declared(node, name, "dynamics.angular_time_constant", d.angular_time_constant);
  d.delay = declared(node, name, "dynamics.delay", d.delay);

  auto& sp = p.speed_profile;
  sp.max_speed = declared(node, name, "speed_profile.max_speed", p.limits.max_v);
  sp.max_lateral_accel = declared(node, name, "speed_profile.max_lateral_accel", sp.max_lateral_accel);
  sp.decel = declared(node, name, "speed_profile.decel", sp.decel);
  sp.curvature_window = declared(node, name, "speed_profile.curvature_window", sp.curvature_window);

  auto& pr = p.primitives;
  pr.duration = declared(node, name, "primitives.duration", pr.duration);
  pr.integration_dt = declared(node, name, "primitives.integration_dt", pr.integration_dt);
  pr.lateral_offsets = declared(node, name, "primitives.lateral_offsets", pr.lateral_offsets);
  pr.speed_levels = static_cast<int>(declared(node, name, "primitives.speed_levels", int64_t{ pr.speed_levels }));
  pr.angular_levels = static_cast<int>(declared(node, name, "primitives.angular_levels", int64_t{ pr.angular_levels }));
  pr.dense_angular_levels =
      static_cast<int>(declared(node, name, "primitives.dense_angular_levels", int64_t{ pr.dense_angular_levels }));
  pr.lookahead_time = declared(node, name, "primitives.lookahead_time", pr.lookahead_time);
  pr.min_lookahead = declared(node, name, "primitives.min_lookahead", pr.min_lookahead);
  pr.rotate_heading_threshold =
      declared(node, name, "primitives.rotate_heading_threshold", pr.rotate_heading_threshold);
  pr.rotate_speed = declared(node, name, "primitives.rotate_speed", pr.rotate_speed);
  pr.max_successors = declaredSize(node, name, "primitives.max_successors", pr.max_successors);

  auto& se = p.search;
  se.frontier_size = declaredSize(node, name, "search.frontier_size", se.frontier_size);
  se.horizon_time = declared(node, name, "search.horizon_time", se.horizon_time);
  se.horizon_distance = declared(node, name, "search.horizon_distance", se.horizon_distance);
  se.max_nodes = declaredSize(node, name, "search.max_nodes", se.max_nodes);
  se.max_primitive_evaluations =
      declaredSize(node, name, "search.max_primitive_evaluations", se.max_primitive_evaluations);
  se.time_budget = declared(node, name, "search.time_budget", se.time_budget);
  se.min_per_maneuver = declaredSize(node, name, "search.min_per_maneuver", se.min_per_maneuver);
  se.max_candidates = declaredSize(node, name, "search.max_candidates", se.max_candidates);

  auto& sc = p.scoring;
  sc.filter_dist = declared(node, name, "corridor.filter_dist", sc.filter_dist);
  sc.max_route_deviation = declared(node, name, "corridor.max_route_deviation", sc.max_route_deviation);
  sc.obstacle_distance_scale = declared(node, name, "scoring.obstacle_distance_scale", sc.obstacle_distance_scale);
  for (size_t i = 0; i < kCostTermCount; ++i)
  {
    const std::string term = toString(static_cast<CostTerm>(i));
    sc.weights[i] = declared(node, name, "scoring." + term + "_weight", sc.weights[i]);
  }
  for (const auto& disabled : declared(node, name, "scoring.disabled_terms", std::vector<std::string>{}))
  {
    bool found = false;
    for (size_t i = 0; i < kCostTermCount; ++i)
    {
      if (disabled == toString(static_cast<CostTerm>(i)))
      {
        sc.enabled[i] = false;
        found = true;
      }
    }
    if (!found)
    {
      throw std::invalid_argument(name + ".scoring.disabled_terms: unknown term '" + disabled + "'");
    }
  }

  p.collision.resolution = declared(node, name, "collision.resolution", p.collision.resolution);
  p.collision.dynamic_margin = declared(node, name, "collision.dynamic_margin", p.collision.dynamic_margin);
  p.collision.unknown_is_lethal = declared(node, name, "collision.unknown_is_lethal", p.collision.unknown_is_lethal);

  auto& sa = p.safety;
  sa.collision.resolution = declared(node, name, "safety.resolution", sa.collision.resolution);
  sa.collision.dynamic_margin = declared(node, name, "safety.dynamic_margin", sa.collision.dynamic_margin);
  sa.collision.unknown_is_lethal = p.collision.unknown_is_lethal;
  sa.brake_factor = declared(node, name, "safety.brake_factor", sa.brake_factor);
  sa.min_delay = declared(node, name, "safety.min_delay", sa.min_delay);
  sa.control_period = 1.0 / controller_frequency;
  sa.max_state_age = declared(node, name, "safety.max_state_age", sa.max_state_age);
  sa.max_costmap_age = declared(node, name, "safety.max_costmap_age", sa.max_costmap_age);

  p.route_start_window = declared(node, name, "route_start_window", p.route_start_window);
  p.goal_yaw_gain = declared(node, name, "goal_yaw_gain", p.goal_yaw_gain);
  p.goal_yaw_tolerance = declared(node, name, "goal_yaw_tolerance", p.goal_yaw_tolerance);
  p.max_fallback_cycles =
      static_cast<int>(declared(node, name, "max_fallback_cycles", int64_t{ p.max_fallback_cycles }));

  const std::string error = p.validate();
  if (!error.empty())
  {
    throw std::invalid_argument(name + ": " + error);
  }
  return p;
}

void PredictiveController::configure(const nav2::LifecycleNode::WeakPtr& parent, std::string name,
                                     nav2::TransformBuffer::SharedPtr tf,
                                     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  Footprint footprint;
  for (const auto& point : costmap_ros_->getRobotFootprint())
  {
    footprint.push_back({ point.x, point.y });
  }
  double controller_frequency = 20.0;
  node->get_parameter("controller_frequency", controller_frequency);

  try
  {
    core_ = std::make_unique<ControllerCore>(loadParameters(node, name_, footprint, controller_frequency));
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(logger_, "Invalid configuration: %s", e.what());
    throw;
  }
  publish_plan_ = declared(node, name_, "publish_plan", true);
  diagnostics_period_ = declared(node, name_, "diagnostics_period", 0.5);
  visualization_.candidates = declared(node, name_, "visualization.candidates", visualization_.candidates);
  visualization_.footprints = declared(node, name_, "visualization.footprints", visualization_.footprints);
  visualization_.reference = declared(node, name_, "visualization.reference", visualization_.reference);
  visualization_.obstacles = declared(node, name_, "visualization.obstacles", visualization_.obstacles);
  visualization_.corridor = declared(node, name_, "visualization.corridor", visualization_.corridor);
  visualization_.labels = declared(node, name_, "visualization.labels", visualization_.labels);
  visualization_.tree = declared(node, name_, "visualization.tree", visualization_.tree);
  core_->setRecordTree(visualization_.tree);

  plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/local_plan");
  diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(name_ + "/diagnostics");
  markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/markers");
  RCLCPP_INFO(logger_, "Configured %s: max_v %.2f m/s, frontier %zu, horizon %.1f s", name_.c_str(),
              core_->params().limits.max_v, core_->params().search.frontier_size, core_->params().search.horizon_time);
}

void PredictiveController::cleanup()
{
  plan_pub_.reset();
  diagnostics_pub_.reset();
  markers_pub_.reset();
  core_.reset();
}

void PredictiveController::activate()
{
  plan_pub_->on_activate();
  diagnostics_pub_->on_activate();
  markers_pub_->on_activate();
}

void PredictiveController::deactivate()
{
  plan_pub_->on_deactivate();
  diagnostics_pub_->on_deactivate();
  markers_pub_->on_deactivate();
}

void PredictiveController::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  core_->reset();
  goal_latched_ = false;
}

void PredictiveController::newPathReceived(const nav_msgs::msg::Path& raw_global_path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (raw_global_path.poses.empty() || !goal_latched_)
  {
    return;
  }
  const auto& end = raw_global_path.poses.back().pose;
  const auto& latched = latched_goal_.pose;
  if (std::hypot(end.position.x - latched.position.x, end.position.y - latched.position.y) > 1e-3)
  {
    goal_latched_ = false;
  }
}

unsigned char PredictiveController::circumscribedCost() const
{
  auto costmap_ros = costmap_ros_;
  const auto inflation = nav2_costmap_2d::InflationLayerInterface::getInflationLayer(costmap_ros);
  if (!inflation)
  {
    return 0;
  }
  const double radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  return inflation->computeCost(radius / costmap_ros_->getCostmap()->getResolution());
}

geometry_msgs::msg::TwistStamped PredictiveController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker, const nav_msgs::msg::Path& transformed_global_plan,
    const geometry_msgs::msg::PoseStamped& global_goal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const rclcpp::Time now = clock_->now();

  ControlInput input;
  input.now = now.seconds();
  input.state.pose = { pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation) };
  input.state.v = std::max(0.0, velocity.linear.x);
  input.state.omega = velocity.angular.z;
  const rclcpp::Time pose_stamp(pose.header.stamp, now.get_clock_type());
  input.ages.state = pose_stamp.nanoseconds() == 0 ? 0.0 : (now - pose_stamp).seconds();
  input.ages.costmap = costmap_ros_->isCurrent() ? 0.0 : std::numeric_limits<double>::infinity();

  for (const auto& p : transformed_global_plan.poses)
  {
    input.route.push_back({ p.pose.position.x, p.pose.position.y });
  }

  geometry_msgs::msg::PoseStamped goal;
  const bool goal_known =
      nav2_util::transformPoseInTargetFrame(global_goal, goal, *tf_, pose.header.frame_id, 0.1);
  if (goal_known && !transformed_global_plan.poses.empty())
  {
    const auto& end = transformed_global_plan.poses.back().pose.position;
    input.route_ends_at_goal = std::hypot(end.x - goal.pose.position.x, end.y - goal.pose.position.y) < 0.1;
    input.goal_heading = tf2::getYaw(goal.pose.orientation);
    if (!goal_latched_ && goal_checker != nullptr &&
        goal_checker->isGoalXYReached(pose.pose, goal.pose, velocity, transformed_global_plan))
    {
      goal_latched_ = true;
      latched_goal_ = global_goal;
    }
  }
  input.goal_position_reached = goal_latched_;
  input.speed_limit = speed_limit_;

  ControlOutput output;
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_ros_->getCostmap()->getMutex());
    input.costmap = costmap_ros_->getCostmap();
    input.circumscribed_cost = circumscribedCost();
    output = core_->compute(input);
  }
  publish(output, input.state, now);

  if (output.status == ControlStatus::NoValidControl)
  {
    throw nav2_core::NoValidControl(output.reason);
  }
  if (output.status == ControlStatus::Fallback || output.status == ControlStatus::InvalidInput)
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "Braking: %s", output.reason.c_str());
  }

  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now;
  command.header.frame_id = pose.header.frame_id;
  command.twist.linear.x = output.command.v;
  command.twist.angular.z = output.command.omega;
  return command;
}

void PredictiveController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (speed_limit <= 0.0 || speed_limit >= 100.0 && percentage)
  {
    speed_limit_ = 0.0;
    return;
  }
  speed_limit_ = percentage ? core_->params().limits.max_v * speed_limit / 100.0 : speed_limit;
}

void PredictiveController::publish(const ControlOutput& output, const RobotState& state, const rclcpp::Time& stamp)
{
  if (markers_pub_->get_subscription_count() > 0)
  {
    markers_pub_->publish(buildMarkers(*core_, output, state, core_->snapshots().current()->params, visualization_,
                                       costmap_ros_->getGlobalFrameID(), stamp));
  }

  if (publish_plan_ && output.has_selection && plan_pub_->get_subscription_count() > 0)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = costmap_ros_->getGlobalFrameID();
    for (const auto& state : output.selected.states)
    {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = state.pose.x;
      p.pose.position.y = state.pose.y;
      p.pose.orientation.z = std::sin(state.pose.theta / 2.0);
      p.pose.orientation.w = std::cos(state.pose.theta / 2.0);
      path.poses.push_back(p);
    }
    plan_pub_->publish(path);
  }

  if ((stamp - last_diagnostics_).seconds() < diagnostics_period_ && output.status == ControlStatus::Tracking)
  {
    return;
  }
  last_diagnostics_ = stamp;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name_;
  status.hardware_id = "predictive_controller";
  status.level = output.status == ControlStatus::Tracking || output.status == ControlStatus::TurningToGoal ||
                         output.status == ControlStatus::GoalReached ?
                     diagnostic_msgs::msg::DiagnosticStatus::OK :
                     diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = std::string(toString(output.status)) + (output.reason.empty() ? "" : ": " + output.reason);
  const SearchStats& s = output.stats;
  status.values = { keyValue("model_version", std::to_string(output.model_version)),
                    keyValue("plan_time_ms", s.elapsed * 1e3),
                    keyValue("nodes", static_cast<double>(s.nodes)),
                    keyValue("evaluations", static_cast<double>(s.evaluations)),
                    keyValue("rejected_collision", static_cast<double>(s.rejected_collision)),
                    keyValue("rejected_route", static_cast<double>(s.rejected_route)),
                    keyValue("rejected_dynamics", static_cast<double>(s.rejected_dynamics)),
                    keyValue("rejected_terminal", static_cast<double>(s.rejected_terminal)),
                    keyValue("frontier", static_cast<double>(s.max_frontier)),
                    keyValue("depth", static_cast<double>(s.depth)),
                    keyValue("horizon_reached_s", s.horizon_reached),
                    keyValue("budget_exhausted", s.budget_exhausted ? "true" : "false"),
                    keyValue("candidates", static_cast<double>(output.candidates)),
                    keyValue("candidates_rejected", static_cast<double>(output.candidates_rejected)),
                    keyValue("first_rejection", toString(output.first_rejection)),
                    keyValue("route_deviation", output.route_deviation),
                    keyValue("reference_speed", output.reference_speed),
                    keyValue("command_v", output.command.v),
                    keyValue("command_omega", output.command.omega) };
  if (output.has_selection)
  {
    status.values.push_back(keyValue("maneuver", toString(output.selected.maneuver)));
    status.values.push_back(keyValue("cost_total", output.selected.total));
    status.values.push_back(keyValue("predicted_progress", output.selected.progress));
    status.values.push_back(keyValue("min_dynamic_clearance", output.selected.min_dynamic_clearance));
    for (size_t i = 0; i < kCostTermCount; ++i)
    {
      status.values.push_back(
          keyValue(std::string("cost_") + toString(static_cast<CostTerm>(i)), output.selected.cost[i]));
    }
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

}  // namespace nice_differential_drive_trajectory_planner

PLUGINLIB_EXPORT_CLASS(nice_differential_drive_trajectory_planner::PredictiveController, nav2_core::Controller)
