#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "test_helpers.hpp"

using namespace nice_differential_drive_trajectory_planner;
using namespace nice_differential_drive_trajectory_planner::test;

namespace
{
constexpr double kHorizon = 2.0;
constexpr double kDt = 0.1;

struct Segment
{
  std::vector<RobotState> states;
  std::vector<double> s;
  std::vector<double> lateral;
};

/** count states at constant speed v along y = lateral. */
Segment cruise(double v, double lateral, int count = 20, double omega = 0.0)
{
  Segment segment;
  for (int k = 1; k <= count; ++k)
  {
    segment.states.push_back(stateAt(v * kDt * k, lateral, 0.0, v, omega));
    segment.s.push_back(v * kDt * k);
    segment.lateral.push_back(lateral);
  }
  return segment;
}

class ScorerTest : public ::testing::Test
{
protected:
  ScorerTest() : scorer_(ScoringParams{}, limits(), kHorizon)
  {
    SpeedProfileParams speed;
    speed.max_speed = 1.0;
    speed.end_speed = -1.0;
    route_.build(straightRoute(10.0), speed);
  }

  static RobotLimits limits()
  {
    RobotLimits limits;
    limits.max_v = 2.0;
    limits.max_omega = 2.0;
    limits.max_accel = 2.0;
    return limits;
  }

  CostVector score(const Segment& segment, const RobotState& before, const Command& parent = {},
                   const Command& command = {}, const SweepResult& sweep = {})
  {
    CostVector cost{};
    scorer_.addSegment(before, segment.states.data(), segment.s.data(), segment.lateral.data(),
                       segment.states.size(), kDt, route_, parent, command, sweep, cost);
    return cost;
  }

  TrajectoryScorer scorer_;
  Route route_;
};
}  // namespace

TEST_F(ScorerTest, on_route_at_reference_speed_costs_nothing)
{
  const CostVector cost = score(cruise(1.0, 0.0), stateAt(0, 0, 0, 1.0));
  for (size_t i = 0; i < kCostTermCount; ++i)
  {
    EXPECT_NEAR(cost[i], 0.0, 1e-12) << toString(static_cast<CostTerm>(i));
  }
}

TEST_F(ScorerTest, velocity_term_penalises_both_slow_and_fast_symmetrically)
{
  const double slow = score(cruise(0.5, 0.0), stateAt(0, 0, 0, 0.5))[kVelocity];
  const double fast = score(cruise(1.5, 0.0), stateAt(0, 0, 0, 1.5))[kVelocity];
  // 20 states of dt 0.1 over a 2 s horizon: ((0.5) / 2)^2 averaged over the whole horizon.
  EXPECT_NEAR(slow, 0.0625, 1e-12);
  EXPECT_NEAR(fast, 0.0625, 1e-12);
}

TEST_F(ScorerTest, route_term_is_quadratic_inside_and_a_barrier_outside)
{
  const ScoringParams& p = scorer_.params();
  EXPECT_NEAR(scorer_.routeCost(p.filter_dist / 2), 0.25, 1e-12);
  EXPECT_NEAR(scorer_.routeCost(-p.filter_dist), 1.0, 1e-12);
  const double midway = (p.filter_dist + p.max_route_deviation) / 2;
  EXPECT_NEAR(scorer_.routeCost(midway), 2.0, 1e-12);
  EXPECT_GT(scorer_.routeCost(p.max_route_deviation - 0.01), 50.0);
  EXPECT_NEAR(score(cruise(1.0, p.filter_dist / 2), stateAt(0, 0, 0, 1.0))[kRoute], 0.25, 1e-12);
}

TEST_F(ScorerTest, acceleration_term_counts_the_speed_change)
{
  Segment segment = cruise(1.0, 0.0, 1);
  const CostVector cost = score(segment, stateAt(0, 0, 0, 0.8));
  // a = 0.2 / 0.1 = 2 = max_accel, over 0.1 of a 2 s horizon.
  EXPECT_NEAR(cost[kAcceleration], 0.05, 1e-12);
}

TEST_F(ScorerTest, angular_term_ignores_the_yaw_rate_the_route_needs)
{
  Route arc;
  SpeedProfileParams speed;
  speed.end_speed = -1.0;
  std::vector<Point2D> points;
  for (int i = 0; i <= 100; ++i)
  {
    const double a = -M_PI / 2 + M_PI * i / 100;
    points.push_back({ 2.0 * std::cos(a), 2.0 + 2.0 * std::sin(a) });
  }
  ASSERT_TRUE(arc.build(points, speed));
  RobotState state = stateAt(0, 0, 0, 1.0, 0.5);
  CostVector cost{};
  const double s = M_PI;
  const double lateral = 0.0;
  scorer_.addSegment(state, &state, &s, &lateral, 1, kDt, arc, {}, {}, {}, cost);
  EXPECT_LT(cost[kAngular], 1e-4);

  const CostVector straight_turn = score(cruise(1.0, 0.0, 1, 0.5), stateAt(0, 0, 0, 1.0));
  EXPECT_NEAR(straight_turn[kAngular], 0.0625 * kDt / kHorizon, 1e-12);
}

TEST_F(ScorerTest, yaw_rate_sign_flip_adds_the_oscillation_penalty)
{
  const auto segment = cruise(1.0, 0.0, 1);
  const double flip = score(segment, stateAt(0, 0, 0, 1.0), { 1.0, 0.5 }, { 1.0, -0.5 })[kAngular];
  const double same = score(segment, stateAt(0, 0, 0, 1.0), { 1.0, 0.5 }, { 1.0, 0.4 })[kAngular];
  EXPECT_NEAR(flip - same, scorer_.params().yaw_flip_penalty, 1e-12);
}

TEST_F(ScorerTest, obstacle_term_rises_with_cost_and_proximity)
{
  SweepResult far;
  far.min_dynamic_clearance = 5.0;
  SweepResult near;
  near.min_dynamic_clearance = 0.1;
  near.max_static_cost = 252.0;
  const auto segment = cruise(1.0, 0.0);
  const double far_cost = score(segment, stateAt(0, 0, 0, 1.0), {}, {}, far)[kObstacle];
  const double near_cost = score(segment, stateAt(0, 0, 0, 1.0), {}, {}, near)[kObstacle];
  EXPECT_NEAR(far_cost, std::exp(-5.0 / 0.5), 1e-12);
  EXPECT_NEAR(near_cost, 1.0 + std::exp(-0.1 / 0.5), 1e-12);
}

TEST_F(ScorerTest, progress_shortfall_and_terminal_terms)
{
  EXPECT_NEAR(scorer_.progressShortfall(1.0, 4.0), 0.75, 1e-12);
  EXPECT_NEAR(scorer_.progressShortfall(5.0, 4.0), 0.0, 1e-12);
  EXPECT_NEAR(scorer_.progressShortfall(0.0, 0.0), 0.0, 1e-12);

  EXPECT_NEAR(scorer_.terminalCost(stateAt(1, 0, 0), kHorizon, 1.0, route_, false), 0.0, 1e-12);
  EXPECT_NEAR(scorer_.terminalCost(stateAt(1, 0, 0), kHorizon / 2, 1.0, route_, true), 0.5, 1e-12);
  EXPECT_NEAR(scorer_.terminalCost(stateAt(1, 0, 0), kHorizon / 2, 1.0, route_, false), 0.0, 1e-12)
      << "a sequence that finished early, e.g. at its progress target, is not unfinished";
  EXPECT_NEAR(scorer_.terminalCost(stateAt(1, 0, M_PI / 2), kHorizon, 1.0, route_, false), 0.25, 1e-12);
}

TEST_F(ScorerTest, holding_still_costs_the_same_as_cruising_at_that_speed)
{
  // Stopped for the whole horizon against a 1 m/s reference: ((0 - 1) / 2)^2 = 0.25.
  CostVector held{};
  scorer_.addHold(stateAt(1, 0.1, 0), 1.0, 0.1, 0.0, kHorizon, route_, held);
  const CostVector cruising = score(cruise(0.0, 0.1), stateAt(0, 0.1, 0, 0.0));
  EXPECT_NEAR(held[kVelocity], 0.25, 1e-12);
  EXPECT_NEAR(held[kVelocity], cruising[kVelocity], 1e-12);
  EXPECT_NEAR(held[kRoute], cruising[kRoute], 1e-12);
}

TEST_F(ScorerTest, ablation_removes_a_term_from_the_total)
{
  CostVector cost{};
  cost[kVelocity] = 1.0;
  cost[kRoute] = 1.0;
  ScoringParams params;
  const double both = TrajectoryScorer(params, limits(), kHorizon).total(cost);
  EXPECT_NEAR(both, params.weights[kVelocity] + params.weights[kRoute], 1e-12);
  params.enabled[kVelocity] = false;
  EXPECT_NEAR(TrajectoryScorer(params, limits(), kHorizon).total(cost), params.weights[kRoute], 1e-12);
}

TEST(ScoringParams, validation)
{
  ScoringParams params;
  EXPECT_TRUE(params.validate().empty());
  params.max_route_deviation = params.filter_dist;
  EXPECT_FALSE(params.validate().empty());
  params = ScoringParams{};
  params.weights[kObstacle] = -1.0;
  EXPECT_FALSE(params.validate().empty());
}
