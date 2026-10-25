#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "test_helpers.hpp"

using namespace nice_differential_drive_trajectory_planner;
using namespace nice_differential_drive_trajectory_planner::test;

namespace
{
constexpr double kPeriod = 1.0 / 15.0;
constexpr double kPruneDistance = 4.0;
constexpr double kGoalTolerance = 0.1;

/** A robot that differs from the planner's nominal model: lag, delay, uneven wheels and weaker brakes. */
DynamicsParams realRobot()
{
  DynamicsParams truth;
  truth.linear_time_constant = 0.1;
  truth.angular_time_constant = 0.08;
  truth.delay = 0.05;
  truth.left_wheel_scale = 0.97;
  truth.right_wheel_scale = 1.03;
  truth.brake_scale = 0.9;
  return truth;
}

class ClosedLoopTest : public ::testing::Test
{
protected:
  ClosedLoopTest() : params_(defaultParams()), costmap_(freeCostmap())
  {
    params_.safety.control_period = kPeriod;
  }

  /** Drives the core against the true model until the goal turn finishes or max_time passes. */
  void run(const RobotState& start, const std::vector<Point2D>& route, double goal_heading, double max_time)
  {
    ControllerCore core(params_);
    const DynamicsModel truth(params_.limits, realRobot());
    CollisionChecker reference(params_.footprint, { 0.01, 0.1, true });
    reference.setCostmap(&costmap_, 0);

    state_ = start;
    trace_.clear();
    statuses_.clear();
    commands_.clear();
    collided_ = false;
    bool goal_latched = false;
    Command sent;
    for (double t = 0.0; t < max_time; t += kPeriod)
    {
      ControlInput input;
      input.now = t;
      input.state = state_;
      input.costmap = &costmap_;
      input.goal_heading = goal_heading;
      input.route = prune(route);
      input.route_ends_at_goal = true;
      goal_latched = goal_latched || distance(position(state_.pose), route.back()) < kGoalTolerance;
      input.goal_position_reached = goal_latched;

      const ControlOutput output = core.compute(input);
      statuses_.push_back(output.status);
      commands_.push_back(output.command);
      if (output.status == ControlStatus::GoalReached)
      {
        break;
      }

      std::vector<RobotState> states;
      truth.rollout(state_, sent, realRobot().delay, output.command, kPeriod, kPeriod / 4.0, states);
      for (const RobotState& s : states)
      {
        collided_ = collided_ || reference.staticCollision(s.pose);
      }
      state_ = states.back();
      sent = output.command;
      trace_.push_back(state_);
    }
  }

  /** What nav2's path handler hands the controller: the route from the closest point, cut at kPruneDistance. */
  std::vector<Point2D> prune(const std::vector<Point2D>& route) const
  {
    size_t closest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < route.size(); ++i)
    {
      const double d = distance(route[i], position(state_.pose));
      if (d < best)
      {
        best = d;
        closest = i;
      }
    }
    std::vector<Point2D> pruned;
    double length = 0.0;
    for (size_t i = closest; i < route.size() && length <= kPruneDistance; ++i)
    {
      if (!pruned.empty())
      {
        length += distance(pruned.back(), route[i]);
      }
      pruned.push_back(route[i]);
    }
    return pruned;
  }

  bool everReversed() const
  {
    return std::any_of(commands_.begin(), commands_.end(), [](const Command& c) { return c.v < 0.0; });
  }

  bool ever(ControlStatus status) const
  {
    return std::find(statuses_.begin(), statuses_.end(), status) != statuses_.end();
  }

  double maxAbsY() const
  {
    double m = 0.0;
    for (const auto& s : trace_)
    {
      m = std::max(m, std::abs(s.pose.y));
    }
    return m;
  }

  ControllerParams params_;
  nav2_costmap_2d::Costmap2D costmap_;
  RobotState state_;
  std::vector<RobotState> trace_;
  std::vector<ControlStatus> statuses_;
  std::vector<Command> commands_;
  bool collided_ = false;
};
}  // namespace

TEST_F(ClosedLoopTest, drives_a_straight_route_to_the_goal)
{
  run(stateAt(0, 0), straightRoute(6.0), 0.0, 20.0);
  ASSERT_FALSE(statuses_.empty());
  EXPECT_EQ(statuses_.back(), ControlStatus::GoalReached);
  EXPECT_LT(distance(position(state_.pose), { 6.0, 0.0 }), 0.15);
  EXPECT_FALSE(collided_);
  EXPECT_FALSE(everReversed());
  double top = 0.0;
  for (const auto& s : trace_)
  {
    top = std::max(top, s.v);
  }
  EXPECT_GT(top, 1.2) << "reaches cruising speed on the straight";
  EXPECT_LT(maxAbsY(), params_.scoring.filter_dist);
}

TEST_F(ClosedLoopTest, passes_a_box_on_the_route_without_touching_it)
{
  fill(costmap_, 2.5, -0.15, 2.8, 0.15);
  inflate(costmap_);
  run(stateAt(0, 0), straightRoute(6.0), 0.0, 25.0);
  EXPECT_FALSE(collided_);
  EXPECT_FALSE(everReversed());
  EXPECT_GT(state_.pose.x, 3.5) << "got past the box";
  EXPECT_GT(maxAbsY(), params_.scoring.filter_dist) << "detoured";
  EXPECT_LT(maxAbsY(), params_.scoring.max_route_deviation);
}

TEST_F(ClosedLoopTest, stops_in_front_of_a_fully_blocked_route)
{
  fill(costmap_, 3.0, -3.0, 3.1, 3.0);
  inflate(costmap_);
  run(stateAt(0, 0), straightRoute(6.0), 0.0, 12.0);
  EXPECT_FALSE(collided_);
  EXPECT_FALSE(everReversed());
  EXPECT_LT(state_.v, 0.05);
  EXPECT_LT(state_.pose.x + params_.footprint.front().x, 3.0);
}

TEST_F(ClosedLoopTest, turns_round_when_starting_backwards_to_the_route)
{
  run(stateAt(0, 0, M_PI), straightRoute(6.0), 0.0, 20.0);
  EXPECT_FALSE(collided_);
  EXPECT_FALSE(everReversed());
  EXPECT_FALSE(ever(ControlStatus::NoValidControl));
  EXPECT_GT(state_.pose.x, 3.0) << "turned round and made progress";
}

TEST_F(ClosedLoopTest, final_turn_reaches_the_goal_heading_without_hunting)
{
  run(stateAt(0, 0), { { 0.0, 0.0 }, { 0.05, 0.0 } }, M_PI / 2, 10.0);
  ASSERT_FALSE(statuses_.empty());
  EXPECT_EQ(statuses_.back(), ControlStatus::GoalReached);
  EXPECT_LT(std::abs(wrapAngle(state_.pose.theta - M_PI / 2)), params_.goal_yaw_tolerance + 0.05);
  int flips = 0;
  for (size_t i = 1; i < commands_.size(); ++i)
  {
    flips += commands_[i].omega * commands_[i - 1].omega < 0.0;
  }
  EXPECT_LE(flips, 1);
  EXPECT_LT(std::abs(state_.pose.x), 0.05) << "turned on the spot";
}

TEST(ControllerCore, stale_state_brakes_instead_of_planning)
{
  auto costmap = freeCostmap();
  ControllerParams params = defaultParams();
  ControllerCore core(params);
  ControlInput input;
  input.state = stateAt(0, 0, 0, 1.0);
  input.costmap = &costmap;
  input.route = straightRoute(4.0);
  input.ages.state = params.safety.max_state_age + 0.1;
  const ControlOutput output = core.compute(input);
  EXPECT_EQ(output.status, ControlStatus::InvalidInput);
  EXPECT_LT(output.command.v, 1.0);
  EXPECT_GE(output.command.v, 0.0);
}

TEST(ControllerCore, degenerate_route_is_invalid_input)
{
  auto costmap = freeCostmap();
  ControllerCore core(defaultParams());
  ControlInput input;
  input.state = stateAt(0, 0, 0, 0.5);
  input.costmap = &costmap;
  input.route = { { 0.0, 0.0 } };
  const ControlOutput output = core.compute(input);
  EXPECT_EQ(output.status, ControlStatus::InvalidInput);
  EXPECT_LT(output.command.v, 0.5);
}

TEST(ControllerCore, gives_up_only_after_repeated_failures_at_rest)
{
  auto costmap = freeCostmap();
  fill(costmap, -1.0, -1.0, 1.0, 1.0);
  ControllerParams params = defaultParams();
  params.max_fallback_cycles = 3;
  ControllerCore core(params);
  ControlInput input;
  input.state = stateAt(0, 0);
  input.costmap = &costmap;
  input.route = straightRoute(4.0);
  std::vector<ControlStatus> statuses;
  for (int i = 0; i < 5; ++i)
  {
    statuses.push_back(core.compute(input).status);
  }
  EXPECT_EQ(statuses[0], ControlStatus::Fallback);
  EXPECT_EQ(statuses[2], ControlStatus::Fallback);
  EXPECT_EQ(statuses[3], ControlStatus::NoValidControl);
}

TEST(ControllerParams, defaults_with_a_footprint_are_valid_and_bad_values_are_named)
{
  ControllerParams params = defaultParams();
  EXPECT_EQ(params.validate(), "");
  params.search.frontier_size = 0;
  EXPECT_NE(params.validate().find("search"), std::string::npos);
  params = defaultParams();
  params.footprint.resize(2);
  EXPECT_NE(params.validate().find("footprint"), std::string::npos);
  params = defaultParams();
  params.collision.resolution = params.safety.collision.resolution * 1.5;
  EXPECT_NE(params.validate().find("collision.resolution"), std::string::npos)
      << "a search coarser than the validator proposes paths the validator rejects";
}
