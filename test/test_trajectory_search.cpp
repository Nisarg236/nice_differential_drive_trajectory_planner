#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "test_helpers.hpp"

using namespace nice_differential_drive_trajectory_planner;
using namespace nice_differential_drive_trajectory_planner::test;

namespace
{
class SearchTest : public ::testing::Test
{
protected:
  SearchTest() : costmap_(freeCostmap())
  {
    params_ = defaultParams();
    params_.search.time_budget = 1.0;  // Deterministic: bounded by evaluation counts, not wall time.
  }

  void plan(const RobotState& state, double route_length = 8.0)
  {
    SpeedProfileParams speed = params_.speed_profile;
    speed.end_speed = -1.0;
    ASSERT_TRUE(route_.build(straightRoute(route_length), speed));
    model_ = std::make_unique<DynamicsModel>(params_.limits, params_.nominal_dynamics);
    checker_ = std::make_unique<CollisionChecker>(params_.footprint, params_.collision);
    checker_->setCostmap(&costmap_, 0);
    checker_->setObstacles(&obstacles_);
    search_ = std::make_unique<TrajectorySearch>(params_.search, params_.primitives, params_.scoring, params_.limits);

    SearchInput input;
    input.state = state;
    input.pending = { state.v, state.omega };
    const RouteProjection projection = route_.project(position(state.pose), 0.0, 1.0);
    input.s0 = projection.s;
    input.lateral0 = projection.lateral;
    input.route = &route_;
    input.model = model_.get();
    input.checker = checker_.get();
    search_->plan(input, candidates_, stats_);
  }

  /** Extreme lateral offset of a candidate from the y = 0 route. */
  static double maxLateral(const Candidate& candidate)
  {
    double extreme = 0.0;
    for (const auto& state : candidate.states)
    {
      extreme = std::abs(state.pose.y) > std::abs(extreme) ? state.pose.y : extreme;
    }
    return extreme;
  }

  bool collisionFree(const Candidate& candidate) const
  {
    CollisionChecker fine(params_.footprint, { 0.01, 0.1, true });
    fine.setCostmap(&costmap_, 0);
    fine.setObstacles(&obstacles_);
    return !fine
                .sweep(candidate.start.pose, 0.0, candidate.states.data(), candidate.states.size(),
                       params_.primitives.integration_dt)
                .collision();
  }

  ControllerParams params_;
  nav2_costmap_2d::Costmap2D costmap_;
  ObstaclePredictionModel obstacles_;
  Route route_;
  std::unique_ptr<DynamicsModel> model_;
  std::unique_ptr<CollisionChecker> checker_;
  std::unique_ptr<TrajectorySearch> search_;
  std::vector<Candidate> candidates_;
  SearchStats stats_;
};

ObstacleTrack movingObstacle(double x, double y, double vx, double vy)
{
  ObstacleTrack track;
  track.position = { x, y };
  track.velocity = { vx, vy };
  track.radius = 0.3;
  track.position_stddev = 0.02;
  track.velocity_stddev = 0.05;
  return track;
}
}  // namespace

TEST_F(SearchTest, straight_unobstructed_route_follows_at_speed)
{
  plan(stateAt(0, 0, 0, 1.0));
  ASSERT_FALSE(candidates_.empty());
  const Candidate& best = candidates_.front();
  EXPECT_EQ(best.maneuver, Maneuver::Follow);
  EXPECT_TRUE(best.reached_horizon || best.progress >= params_.search.horizon_distance - 0.1)
      << "ends at the time horizon or at the progress target";
  EXPECT_GT(best.commands.front().v, 1.0);
  EXPECT_NEAR(best.commands.front().omega, 0.0, 0.05);
  EXPECT_GT(best.progress, 2.5);
  EXPECT_LT(std::abs(maxLateral(best)), params_.scoring.filter_dist);
  EXPECT_FALSE(stats_.budget_exhausted);
}

TEST_F(SearchTest, children_continue_from_the_physical_end_state)
{
  plan(stateAt(0, 0, 0, 0.6, 0.1));
  ASSERT_FALSE(candidates_.empty());
  for (const Candidate& candidate : candidates_)
  {
    // Re-simulating the command sequence from the start must reproduce the candidate exactly.
    std::vector<RobotState> replay;
    RobotState state = candidate.start;
    Command pending{ candidate.start.v, candidate.start.omega };
    for (const Command& command : candidate.commands)
    {
      std::vector<RobotState> segment;
      model_->rollout(state, pending, 0.0, command, params_.primitives.duration, params_.primitives.integration_dt,
                      segment);
      replay.insert(replay.end(), segment.begin(), segment.end());
      state = segment.back();
      pending = command;
    }
    ASSERT_EQ(replay.size(), candidate.states.size());
    for (size_t i = 0; i < replay.size(); ++i)
    {
      EXPECT_EQ(replay[i].pose.x, candidate.states[i].pose.x);
      EXPECT_EQ(replay[i].pose.y, candidate.states[i].pose.y);
      EXPECT_EQ(replay[i].v, candidate.states[i].v);
    }
  }
}

TEST_F(SearchTest, static_obstacle_on_the_route_is_passed_on_either_side)
{
  fill(costmap_, 2.0, -0.15, 2.3, 0.15);
  params_.search.max_candidates = 40;
  params_.search.horizon_time = 4.0;
  plan(stateAt(0, 0, 0, 0.8));
  ASSERT_FALSE(candidates_.empty());
  for (const Candidate& candidate : candidates_)
  {
    EXPECT_TRUE(collisionFree(candidate));
  }
  bool left = false;
  bool right = false;
  for (const Candidate& candidate : candidates_)
  {
    if (candidate.states.back().pose.x > 2.6)
    {
      left = left || maxLateral(candidate) > params_.scoring.filter_dist;
      right = right || maxLateral(candidate) < -params_.scoring.filter_dist;
    }
  }
  EXPECT_TRUE(left);
  EXPECT_TRUE(right);

  const Candidate& best = candidates_.front();
  EXPECT_GT(best.states.back().pose.x, 2.6) << "the best candidate gets past the obstacle";
  EXPECT_GT(std::abs(maxLateral(best)), params_.scoring.filter_dist) << "leaves the preferred corridor";
  EXPECT_LT(std::abs(maxLateral(best)), params_.scoring.max_route_deviation);
}

TEST_F(SearchTest, passes_on_the_only_free_side)
{
  fill(costmap_, 2.0, -0.15, 2.3, 1.5);
  params_.search.horizon_time = 4.0;
  plan(stateAt(0, 0, 0, 0.8));
  ASSERT_FALSE(candidates_.empty());
  const Candidate& best = candidates_.front();
  EXPECT_TRUE(collisionFree(best));
  EXPECT_GT(best.states.back().pose.x, 2.6);
  EXPECT_LT(maxLateral(best), -params_.scoring.filter_dist);
}

TEST_F(SearchTest, fully_blocked_route_stops_before_the_wall)
{
  fill(costmap_, 2.5, -1.5, 2.6, 1.5);
  plan(stateAt(0, 0, 0, 1.2));
  ASSERT_FALSE(candidates_.empty());
  for (const Candidate& candidate : candidates_)
  {
    EXPECT_TRUE(collisionFree(candidate));
  }
  const Candidate& best = candidates_.front();
  EXPECT_LT(best.states.back().v, 0.05);
  EXPECT_LT(best.states.back().pose.x + params_.footprint.front().x, 2.5);
  EXPECT_GT(stats_.rejected_collision + stats_.rejected_terminal, 0u);
}

TEST_F(SearchTest, fast_approach_to_a_close_wall_returns_only_safe_candidates)
{
  fill(costmap_, 1.0, -1.5, 1.1, 1.5);
  plan(stateAt(0, 0, 0, 1.6));
  for (const Candidate& candidate : candidates_)
  {
    EXPECT_TRUE(collisionFree(candidate));
    EXPECT_LT(candidate.states.back().v, 0.05) << "anything still moving would hit the wall";
  }
}

TEST_F(SearchTest, head_on_obstacle_is_avoided)
{
  obstacles_.setTracks({ movingObstacle(5.0, 0.0, -0.8, 0.0) }, 0.0);
  params_.search.horizon_time = 3.2;
  plan(stateAt(0, 0, 0, 1.0));
  ASSERT_FALSE(candidates_.empty());
  const Candidate& best = candidates_.front();
  EXPECT_TRUE(collisionFree(best));
  EXPECT_GT(best.min_dynamic_clearance, params_.collision.dynamic_margin);
}

TEST_F(SearchTest, crossing_obstacle_is_never_driven_into)
{
  // Crosses the route at x = 2.5 about 2.5 s from now.
  obstacles_.setTracks({ movingObstacle(2.5, -2.5, 0.0, 1.0) }, 0.0);
  plan(stateAt(0, 0, 0, 1.0));
  ASSERT_FALSE(candidates_.empty());
  for (const Candidate& candidate : candidates_)
  {
    EXPECT_TRUE(collisionFree(candidate));
  }
}

TEST_F(SearchTest, uncertain_obstacle_keeps_a_wider_berth)
{
  ObstacleTrack certain = movingObstacle(3.0, 0.0, 0.0, 0.0);
  ObstacleTrack uncertain = certain;
  uncertain.position_stddev = 0.2;
  uncertain.velocity_stddev = 0.2;
  params_.search.horizon_time = 4.0;

  obstacles_.setTracks({ certain }, 0.0);
  plan(stateAt(0, 0, 0, 0.8));
  ASSERT_FALSE(candidates_.empty());
  const double certain_clearance = candidates_.front().min_dynamic_clearance;

  obstacles_.setTracks({ uncertain }, 0.0);
  plan(stateAt(0, 0, 0, 0.8));
  ASSERT_FALSE(candidates_.empty());
  CollisionChecker reference(params_.footprint, params_.collision);
  ObstaclePredictionModel nominal;
  nominal.setTracks({ certain }, 0.0);
  reference.setObstacles(&nominal);
  double true_clearance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < candidates_.front().states.size(); ++i)
  {
    true_clearance = std::min(true_clearance, reference.dynamicClearance(candidates_.front().states[i].pose,
                                                                         params_.primitives.integration_dt * (i + 1)));
  }
  EXPECT_GT(true_clearance, certain_clearance - 0.05);
}

TEST_F(SearchTest, frontier_keeps_several_maneuver_classes)
{
  params_.search.max_candidates = 60;
  plan(stateAt(0, 0, 0, 1.0));
  std::set<Maneuver> maneuvers;
  for (const Candidate& candidate : candidates_)
  {
    maneuvers.insert(candidate.maneuver);
  }
  EXPECT_GE(maneuvers.size(), 3u);
}

TEST_F(SearchTest, exhausted_budget_still_returns_fully_evaluated_candidates)
{
  params_.search.max_primitive_evaluations = 30;
  plan(stateAt(0, 0, 0, 1.0));
  EXPECT_TRUE(stats_.budget_exhausted);
  EXPECT_LE(stats_.evaluations, 30u);
  ASSERT_FALSE(candidates_.empty());
  for (const Candidate& candidate : candidates_)
  {
    EXPECT_FALSE(candidate.states.empty());
    EXPECT_TRUE(collisionFree(candidate));
  }
}

TEST_F(SearchTest, wall_clock_budget_is_respected)
{
  params_.search.time_budget = 0.002;
  params_.search.max_nodes = 100000;
  params_.search.max_primitive_evaluations = 100000;
  params_.search.frontier_size = 200;
  plan(stateAt(0, 0, 0, 1.0));
  EXPECT_TRUE(stats_.budget_exhausted);
  EXPECT_LT(stats_.elapsed, 0.05);
}

TEST_F(SearchTest, robot_boxed_in_has_no_moving_candidate)
{
  fill(costmap_, -0.6, 0.35, 1.0, 0.4);
  fill(costmap_, -0.6, -0.4, 1.0, -0.35);
  fill(costmap_, 0.4, -0.4, 0.45, 0.4);
  plan(stateAt(0, 0, 0, 0.0));
  for (const Candidate& candidate : candidates_)
  {
    EXPECT_TRUE(collisionFree(candidate));
    EXPECT_LT(candidate.states.back().pose.x, 0.1);
  }
}

TEST_F(SearchTest, robot_facing_away_from_the_route_turns_instead_of_stopping)
{
  // Regression: an early stop used to accrue running costs only until it stopped, so it beat a long turn.
  plan(stateAt(0, 0, M_PI));
  ASSERT_FALSE(candidates_.empty());
  EXPECT_EQ(candidates_.front().maneuver, Maneuver::Rotate);
  EXPECT_GT(std::abs(candidates_.front().commands.front().omega), 0.5);
}

TEST_F(SearchTest, stopping_short_is_charged_for_the_rest_of_the_horizon)
{
  plan(stateAt(0, 0, 0, 0.0));
  ASSERT_FALSE(candidates_.empty());
  EXPECT_NE(candidates_.front().maneuver, Maneuver::Stop) << "open route ahead";
  for (const Candidate& candidate : candidates_)
  {
    if (candidate.maneuver == Maneuver::Stop)
    {
      EXPECT_GT(candidate.cost[kVelocity], 0.5) << "stopped for the whole horizon against v_ref 1.6 m/s";
    }
  }
}

TEST_F(SearchTest, tree_holds_every_simulated_primitive_when_recording)
{
  fill(costmap_, 2.0, -0.15, 2.3, 0.15);
  plan(stateAt(0, 0, 0, 0.8));
  std::vector<TreePrimitive> tree;
  search_->treePrimitives(tree, 100000);
  EXPECT_EQ(tree.size(), stats_.nodes) << "rejected primitives are only kept when recording";

  search_->setRecordTree(true);
  SearchInput input;
  input.state = stateAt(0, 0, 0, 0.8);
  input.pending = { 0.8, 0.0 };
  input.route = &route_;
  input.model = model_.get();
  input.checker = checker_.get();
  search_->plan(input, candidates_, stats_);
  search_->treePrimitives(tree, 100000);
  const auto count = [&tree](TreePrimitive::Outcome outcome) {
    return static_cast<size_t>(std::count_if(tree.begin(), tree.end(),
                                             [outcome](const TreePrimitive& p) { return p.outcome == outcome; }));
  };
  EXPECT_EQ(count(TreePrimitive::Outcome::Kept), stats_.nodes);
  EXPECT_EQ(count(TreePrimitive::Outcome::Collision), stats_.rejected_collision);
  EXPECT_EQ(count(TreePrimitive::Outcome::OffRoute), stats_.rejected_route);
  EXPECT_GT(stats_.rejected_collision, 0u);
  for (const auto& primitive : tree)
  {
    EXPECT_EQ(primitive.states.size(), 9u) << "start state plus 8 simulated states";
  }
}

TEST_F(SearchTest, planning_is_deterministic)
{
  fill(costmap_, 2.0, -0.15, 2.3, 0.15);
  plan(stateAt(0, 0, 0, 0.8));
  const auto first = candidates_;
  plan(stateAt(0, 0, 0, 0.8));
  ASSERT_EQ(first.size(), candidates_.size());
  for (size_t i = 0; i < first.size(); ++i)
  {
    EXPECT_EQ(first[i].total, candidates_[i].total);
    EXPECT_EQ(first[i].commands.front().omega, candidates_[i].commands.front().omega);
  }
}
