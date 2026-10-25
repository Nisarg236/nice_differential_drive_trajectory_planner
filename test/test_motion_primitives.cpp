#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "test_helpers.hpp"

using namespace nice_differential_drive_trajectory_planner;
using namespace nice_differential_drive_trajectory_planner::test;

namespace
{
Route straight(double length = 6.0)
{
  Route route;
  SpeedProfileParams speed;
  speed.max_speed = 1.6;
  speed.end_speed = -1.0;
  route.build(straightRoute(length), speed);
  return route;
}

bool hasManeuver(const std::vector<Primitive>& primitives, Maneuver maneuver)
{
  return std::any_of(primitives.begin(), primitives.end(),
                     [maneuver](const Primitive& p) { return p.maneuver == maneuver; });
}
}  // namespace

TEST(MotionPrimitives, set_is_bounded_and_deterministic)
{
  const DynamicsModel model(defaultParams().limits, DynamicsParams{});
  PrimitiveParams params;
  params.max_successors = 10;
  const MotionPrimitiveGenerator generator(params);
  const Route route = straight();
  std::vector<Primitive> first;
  std::vector<Primitive> second;
  generator.generate(stateAt(0, 0, 0, 0.8), route, 0.0, true, model, first);
  generator.generate(stateAt(0, 0, 0, 0.8), route, 0.0, true, model, second);
  EXPECT_LE(first.size(), 10u);
  ASSERT_EQ(first.size(), second.size());
  for (size_t i = 0; i < first.size(); ++i)
  {
    EXPECT_EQ(first[i].command.v, second[i].command.v);
    EXPECT_EQ(first[i].command.omega, second[i].command.omega);
  }
}

TEST(MotionPrimitives, speeds_are_reachable_and_never_negative)
{
  const ControllerParams controller = defaultParams();
  const DynamicsModel model(controller.limits, DynamicsParams{});
  const PrimitiveParams params;
  const MotionPrimitiveGenerator generator(params);
  std::vector<Primitive> primitives;
  generator.generate(stateAt(0, 0, 0, 0.5), straight(), 0.0, true, model, primitives);
  const double v_low = std::max(0.0, 0.5 - controller.limits.max_brake * params.duration);
  const double v_high = 0.5 + controller.limits.max_accel * params.duration;
  for (const auto& primitive : primitives)
  {
    EXPECT_GE(primitive.command.v, 0.0);
    if (primitive.maneuver != Maneuver::Stop)
    {
      EXPECT_GE(primitive.command.v, v_low - 1e-9);
      EXPECT_LE(primitive.command.v, v_high + 1e-9);
    }
  }
}

TEST(MotionPrimitives, always_offers_stop_and_both_passing_sides)
{
  const DynamicsModel model(defaultParams().limits, DynamicsParams{});
  const MotionPrimitiveGenerator generator(PrimitiveParams{});
  std::vector<Primitive> primitives;
  generator.generate(stateAt(0, 0, 0, 1.0), straight(), 0.0, false, model, primitives);
  EXPECT_EQ(primitives.front().maneuver, Maneuver::Stop);
  EXPECT_TRUE(hasManeuver(primitives, Maneuver::Follow));
  EXPECT_TRUE(hasManeuver(primitives, Maneuver::PassLeft));
  EXPECT_TRUE(hasManeuver(primitives, Maneuver::PassRight));
}

TEST(MotionPrimitives, offers_rotation_in_place_only_when_misaligned)
{
  const DynamicsModel model(defaultParams().limits, DynamicsParams{});
  const MotionPrimitiveGenerator generator(PrimitiveParams{});
  std::vector<Primitive> primitives;
  generator.generate(stateAt(0, 0, 0.1), straight(), 0.0, false, model, primitives);
  EXPECT_FALSE(hasManeuver(primitives, Maneuver::Rotate));

  generator.generate(stateAt(0, 0, -2.0), straight(), 0.0, false, model, primitives);
  ASSERT_TRUE(hasManeuver(primitives, Maneuver::Rotate));
  const auto rotate = std::find_if(primitives.begin(), primitives.end(),
                                   [](const Primitive& p) { return p.maneuver == Maneuver::Rotate; });
  EXPECT_EQ(rotate->command.v, 0.0);
  EXPECT_GT(rotate->command.omega, 0.0) << "turns the short way towards the route heading";
}

TEST(MotionPrimitives, dense_sampling_adds_alternatives)
{
  const DynamicsModel model(defaultParams().limits, DynamicsParams{});
  PrimitiveParams params;
  params.max_successors = 64;
  const MotionPrimitiveGenerator generator(params);
  std::vector<Primitive> coarse;
  std::vector<Primitive> dense;
  generator.generate(stateAt(0, 0, 0, 0.8), straight(), 0.0, false, model, coarse);
  generator.generate(stateAt(0, 0, 0, 0.8), straight(), 0.0, true, model, dense);
  EXPECT_GT(dense.size(), coarse.size());
}

TEST(MotionPrimitives, offset_tracking_steers_towards_the_shifted_route)
{
  const MotionPrimitiveGenerator generator(PrimitiveParams{});
  const Route route = straight();
  EXPECT_NEAR(generator.trackingYawRate(stateAt(0, 0), route, 0.0, 0.0, 1.0), 0.0, 1e-9);
  EXPECT_GT(generator.trackingYawRate(stateAt(0, 0), route, 0.0, 0.5, 1.0), 0.0);
  EXPECT_LT(generator.trackingYawRate(stateAt(0, 0), route, 0.0, -0.5, 1.0), 0.0);
  EXPECT_LT(generator.trackingYawRate(stateAt(0, 0.4), route, 0.0, 0.0, 1.0), 0.0) << "returns from the left";
  EXPECT_EQ(generator.trackingYawRate(stateAt(0, 0.4), route, 0.0, 0.0, 0.0), 0.0);
}

TEST(MotionPrimitives, validation_rejects_bad_durations)
{
  PrimitiveParams params;
  EXPECT_TRUE(params.validate().empty());
  params.integration_dt = 0.03;
  EXPECT_FALSE(params.validate().empty()) << "0.4 is not a multiple of 0.03";
  params.integration_dt = 0.05;
  params.lateral_offsets.clear();
  EXPECT_FALSE(params.validate().empty());
}
