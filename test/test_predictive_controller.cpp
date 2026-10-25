#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nice_differential_drive_trajectory_planner/predictive_controller.hpp"

using nice_differential_drive_trajectory_planner::PredictiveController;

namespace
{
constexpr double kResolution = 0.05;

/** Reports the goal position reached whenever told to. */
class FakeGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(const nav2::LifecycleNode::WeakPtr&, const std::string&,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
  }
  void reset() override
  {
  }
  bool isGoalReached(const geometry_msgs::msg::Pose&, const geometry_msgs::msg::Pose&,
                     const geometry_msgs::msg::Twist&, const nav_msgs::msg::Path&) override
  {
    return false;
  }
  bool getTolerances(geometry_msgs::msg::Pose&, geometry_msgs::msg::Twist&, double&) override
  {
    return false;
  }
  bool isGoalXYReached(const geometry_msgs::msg::Pose&, const geometry_msgs::msg::Pose&,
                       const geometry_msgs::msg::Twist&, const nav_msgs::msg::Path&) override
  {
    return xy_reached;
  }

  bool xy_reached = false;
};

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

nav_msgs::msg::Path straightPath(double length)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= static_cast<int>(std::round(length / kResolution)); ++i)
  {
    path.poses.push_back(pose(i * kResolution, 0.0));
  }
  return path;
}

class PluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("predictive_controller_test");
    node_->declare_parameter("controller_frequency", 20.0);
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
    // No layers: nav2's default layers never report current without sensor data.
    costmap_ros_->declare_parameter("plugins", std::vector<std::string>{});
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    *costmap_ros_->getCostmap() =
        nav2_costmap_2d::Costmap2D(200, 200, kResolution, -5.0, -5.0, nav2_costmap_2d::FREE_SPACE);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    costmap_ros_->getCostmap()->setDefaultValue(nav2_costmap_2d::FREE_SPACE);
  }

  std::unique_ptr<PredictiveController> make()
  {
    auto controller = std::make_unique<PredictiveController>();
    controller->configure(node_, "FollowPath", tf_, costmap_ros_);
    controller->activate();
    return controller;
  }

  geometry_msgs::msg::TwistStamped step(PredictiveController& controller, double v = 0.0, double yaw = 0.0,
                                        nav2_core::GoalChecker* checker = nullptr)
  {
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = v;
    const auto path = straightPath(4.0);
    return controller.computeVelocityCommands(pose(0.0, 0.0, yaw), velocity, checker ? checker : &checker_, path,
                                              path.poses.back());
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  FakeGoalChecker checker_;
};
}  // namespace

TEST_F(PluginTest, drives_forward_along_a_straight_path)
{
  auto controller = make();
  const auto command = step(*controller, 0.5);
  EXPECT_EQ(command.header.frame_id, costmap_ros_->getGlobalFrameID());
  EXPECT_GT(command.twist.linear.x, 0.5);
  EXPECT_LE(command.twist.linear.x, 1.0 + 1e-9);
  EXPECT_NEAR(command.twist.angular.z, 0.0, 0.05);
  controller->deactivate();
  controller->cleanup();
}

TEST_F(PluginTest, rejects_an_invalid_configuration_by_name)
{
  node_->declare_parameter("FollowPath.search.frontier_size", 0);
  auto controller = std::make_unique<PredictiveController>();
  try
  {
    controller->configure(node_, "FollowPath", tf_, costmap_ros_);
    FAIL() << "configure accepted frontier_size 0";
  }
  catch (const std::invalid_argument& e)
  {
    EXPECT_NE(std::string(e.what()).find("frontier_size"), std::string::npos) << e.what();
  }
}

TEST_F(PluginTest, speed_limit_caps_the_command)
{
  auto controller = make();
  controller->setSpeedLimit(30.0, true);
  double v = 0.0;
  for (int i = 0; i < 10; ++i)
  {
    v = step(*controller, v).twist.linear.x;
    EXPECT_LE(v, 0.3 + 1e-6);
  }
  controller->setSpeedLimit(0.0, false);
  EXPECT_GT(step(*controller, v).twist.linear.x, 0.3);
}

TEST_F(PluginTest, turns_in_place_to_the_goal_heading_once_the_position_is_reached)
{
  auto controller = make();
  checker_.xy_reached = true;
  geometry_msgs::msg::Twist still;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = { pose(0.0, 0.0), pose(0.05, 0.0, M_PI / 2) };
  const auto command =
      controller->computeVelocityCommands(pose(0.0, 0.0), still, &checker_, path, pose(0.05, 0.0, M_PI / 2));
  EXPECT_EQ(command.twist.linear.x, 0.0);
  EXPECT_GT(command.twist.angular.z, 0.0) << "turns left towards +pi/2";
}

TEST_F(PluginTest, raises_no_valid_control_after_repeated_failures_at_rest)
{
  node_->declare_parameter("FollowPath.max_fallback_cycles", 2);
  auto* costmap = costmap_ros_->getCostmap();
  for (unsigned int y = 95; y < 105; ++y)
  {
    for (unsigned int x = 95; x < 105; ++x)
    {
      costmap->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  auto controller = make();
  EXPECT_EQ(step(*controller).twist.linear.x, 0.0);
  EXPECT_EQ(step(*controller).twist.linear.x, 0.0);
  EXPECT_THROW(step(*controller), nav2_core::NoValidControl);
}

TEST_F(PluginTest, publishes_markers_and_diagnostics)
{
  auto controller = make();
  auto listener = rclcpp::Node::make_shared("marker_listener");
  std::set<std::string> namespaces;
  bool got_diagnostics = false;
  auto markers = listener->create_subscription<visualization_msgs::msg::MarkerArray>(
      "FollowPath/markers", 10, [&](const visualization_msgs::msg::MarkerArray& msg) {
        for (const auto& m : msg.markers)
        {
          namespaces.insert(m.ns);
        }
      });
  auto diagnostics = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "FollowPath/diagnostics", 10, [&](const diagnostic_msgs::msg::DiagnosticArray&) { got_diagnostics = true; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline && (namespaces.count("selected") == 0 || !got_diagnostics))
  {
    step(*controller, 0.5);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_TRUE(got_diagnostics);
  for (const char* ns : { "selected", "stop_check", "reference", "corridor", "status" })
  {
    EXPECT_EQ(namespaces.count(ns), 1u) << ns;
  }
  EXPECT_EQ(namespaces.count("candidate_labels"), 0u) << "labels are opt-in";
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
