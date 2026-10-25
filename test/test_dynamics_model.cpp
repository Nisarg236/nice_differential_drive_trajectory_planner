#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "nice_differential_drive_trajectory_planner/dynamics_model.hpp"

using nice_differential_drive_trajectory_planner::Command;
using nice_differential_drive_trajectory_planner::DynamicsModel;
using nice_differential_drive_trajectory_planner::DynamicsParams;
using nice_differential_drive_trajectory_planner::Pose2D;
using nice_differential_drive_trajectory_planner::RobotLimits;
using nice_differential_drive_trajectory_planner::RobotState;

namespace
{
constexpr double kDt = 0.02;
constexpr double kTolerance = 1e-9;

RobotState moving(double v, double omega)
{
  RobotState state;
  state.v = v;
  state.omega = omega;
  return state;
}

std::vector<RobotState> simulate(const DynamicsModel& model, const RobotState& start, const Command& command,
                                 double duration, double dt = kDt)
{
  std::vector<RobotState> out;
  model.rollout(start, command, 0.0, command, duration, dt, out);
  return out;
}
}  // namespace

TEST(DynamicsModel, straight_line_at_constant_speed_matches_v_times_t)
{
  const DynamicsModel model(RobotLimits{}, DynamicsParams{});
  const auto out = simulate(model, moving(1.0, 0.0), { 1.0, 0.0 }, 2.0);
  ASSERT_EQ(out.size(), 100u);
  EXPECT_NEAR(out.back().pose.x, 2.0, kTolerance);
  EXPECT_NEAR(out.back().pose.y, 0.0, kTolerance);
  EXPECT_NEAR(out.back().pose.theta, 0.0, kTolerance);
}

TEST(DynamicsModel, constant_curvature_stays_on_the_circle)
{
  const DynamicsModel model(RobotLimits{}, DynamicsParams{});
  const double v = 0.5;
  const double omega = 0.5;
  const double radius = v / omega;
  const double duration = 4.0;
  const auto out = simulate(model, moving(v, omega), { v, omega }, duration);
  for (const auto& state : out)
  {
    EXPECT_NEAR(std::hypot(state.pose.x, state.pose.y - radius), radius, kTolerance);
  }
  EXPECT_NEAR(out.back().pose.x, radius * std::sin(omega * duration), kTolerance);
  EXPECT_NEAR(out.back().pose.y, radius * (1.0 - std::cos(omega * duration)), kTolerance);
}

TEST(DynamicsModel, rotation_in_place_does_not_translate)
{
  const DynamicsModel model(RobotLimits{}, DynamicsParams{});
  const auto out = simulate(model, moving(0.0, 1.0), { 0.0, 1.0 }, 1.0);
  EXPECT_NEAR(out.back().pose.x, 0.0, kTolerance);
  EXPECT_NEAR(out.back().pose.y, 0.0, kTolerance);
  EXPECT_NEAR(out.back().pose.theta, 1.0, kTolerance);
}

TEST(DynamicsModel, zero_command_at_rest_stays_put)
{
  const DynamicsModel model(RobotLimits{}, DynamicsParams{});
  const auto out = simulate(model, RobotState{}, {}, 1.0);
  EXPECT_EQ(out.back().pose.x, 0.0);
  EXPECT_EQ(out.back().pose.y, 0.0);
  EXPECT_EQ(out.back().pose.theta, 0.0);
  EXPECT_EQ(out.back().v, 0.0);
}

TEST(DynamicsModel, acceleration_and_braking_respect_their_limits)
{
  RobotLimits limits;
  limits.max_accel = 1.0;
  limits.max_brake = 2.0;
  const DynamicsModel model(limits, DynamicsParams{});

  auto out = simulate(model, RobotState{}, { 1.5, 0.0 }, 1.0);
  EXPECT_NEAR(out.back().v, 1.0, kTolerance);
  EXPECT_NEAR(out.back().pose.x, 0.5, 1e-6);

  out = simulate(model, moving(1.5, 0.0), { 0.0, 0.0 }, 0.5);
  EXPECT_NEAR(out.back().v, 0.5, kTolerance);
  out = simulate(model, moving(1.5, 0.0), { 0.0, 0.0 }, 1.0);
  EXPECT_NEAR(out.back().v, 0.0, kTolerance);
  EXPECT_NEAR(out.back().pose.x, 1.5 * 1.5 / (2.0 * 2.0), 1e-6);
}

TEST(DynamicsModel, angular_acceleration_is_limited_both_ways)
{
  RobotLimits limits;
  limits.max_angular_accel = 2.0;
  const DynamicsModel model(limits, DynamicsParams{});
  auto out = simulate(model, RobotState{}, { 0.0, 2.0 }, 0.5);
  EXPECT_NEAR(out.back().omega, 1.0, kTolerance);
  out = simulate(model, moving(0.0, 1.0), { 0.0, -2.0 }, 0.5);
  EXPECT_NEAR(out.back().omega, 0.0, kTolerance);
}

TEST(DynamicsModel, wheel_speed_limit_scales_the_command_keeping_curvature)
{
  RobotLimits limits;
  limits.max_v = 10.0;
  limits.max_omega = 10.0;
  limits.wheel_radius = 0.1;
  limits.wheel_separation = 0.5;
  limits.max_wheel_speed = 10.0;
  const DynamicsModel model(limits, DynamicsParams{});

  const Command saturated = model.saturate({ 1.0, 2.0 });
  const double half_track = limits.wheel_separation / 2.0;
  const double right = (saturated.v + saturated.omega * half_track) / limits.wheel_radius;
  EXPECT_NEAR(right, limits.max_wheel_speed, kTolerance);
  EXPECT_NEAR(saturated.omega / saturated.v, 2.0, kTolerance);
}

TEST(DynamicsModel, velocity_limits_scale_the_command_keeping_curvature)
{
  const RobotLimits limits;
  const DynamicsModel model(limits, DynamicsParams{});
  const Command saturated = model.saturate({ 2.0 * limits.max_v, 0.2 });
  EXPECT_NEAR(saturated.v, limits.max_v, kTolerance);
  EXPECT_NEAR(saturated.omega, 0.1, kTolerance);
}

TEST(DynamicsModel, never_reverses)
{
  const DynamicsModel model(RobotLimits{}, DynamicsParams{});
  EXPECT_EQ(model.saturate({ -1.0, 0.5 }).v, 0.0);
  EXPECT_EQ(model.saturate({ -1.0, 0.5 }).omega, 0.5);
  const auto out = simulate(model, moving(0.3, 0.0), { -1.0, 0.0 }, 1.0);
  for (const auto& state : out)
  {
    EXPECT_GE(state.v, 0.0);
  }
  EXPECT_GE(out.back().pose.x, 0.0);

  DynamicsParams asymmetric;
  asymmetric.left_wheel_scale = 2.0;
  asymmetric.right_wheel_scale = 0.1;
  const DynamicsModel skewed(RobotLimits{}, asymmetric);
  EXPECT_GE(skewed.steadyState({ 0.0, 1.0 }).v, 0.0);
}

TEST(DynamicsModel, integration_is_independent_of_the_step_for_constant_twist)
{
  const DynamicsModel model(RobotLimits{}, DynamicsParams{});
  const auto coarse = simulate(model, moving(0.8, 0.4), { 0.8, 0.4 }, 2.0, 0.5);
  const auto fine = simulate(model, moving(0.8, 0.4), { 0.8, 0.4 }, 2.0, 0.001);
  EXPECT_NEAR(coarse.back().pose.x, fine.back().pose.x, 1e-9);
  EXPECT_NEAR(coarse.back().pose.y, fine.back().pose.y, 1e-9);
}

TEST(DynamicsModel, integration_converges_under_acceleration)
{
  DynamicsParams params;
  params.linear_time_constant = 0.2;
  params.angular_time_constant = 0.1;
  const DynamicsModel model(RobotLimits{}, params);
  const auto coarse = simulate(model, RobotState{}, { 1.0, 0.8 }, 2.0, 0.02);
  const auto fine = simulate(model, RobotState{}, { 1.0, 0.8 }, 2.0, 0.001);
  EXPECT_NEAR(coarse.back().pose.x, fine.back().pose.x, 5e-3);
  EXPECT_NEAR(coarse.back().pose.y, fine.back().pose.y, 5e-3);
  EXPECT_NEAR(coarse.back().pose.theta, fine.back().pose.theta, 5e-3);
}

TEST(DynamicsModel, first_order_lag_reaches_63_percent_after_one_time_constant)
{
  RobotLimits limits;
  limits.max_accel = 100.0;
  DynamicsParams params;
  params.linear_time_constant = 0.25;
  const DynamicsModel model(limits, params);
  const auto out = simulate(model, RobotState{}, { 1.0, 0.0 }, 0.25, 0.01);
  EXPECT_NEAR(out.back().v, 1.0 - std::exp(-1.0), 1e-9);
}

TEST(DynamicsModel, delay_holds_the_pending_command)
{
  const DynamicsModel model(RobotLimits{}, DynamicsParams{});
  std::vector<RobotState> out;
  model.rollout(moving(1.0, 0.0), { 1.0, 0.0 }, 0.1, { 0.0, 0.0 }, 0.6, kDt, out);
  EXPECT_NEAR(out[4].v, 1.0, kTolerance);
  EXPECT_NEAR(out[4].pose.x, 0.1, kTolerance);
  EXPECT_NEAR(out.back().v, 0.0, kTolerance);
  EXPECT_NEAR(out.back().pose.x, 0.1 + 1.0 / (2.0 * RobotLimits{}.max_brake), 1e-6);
}

TEST(DynamicsModel, delay_switching_mid_step_matches_a_fine_grid)
{
  const DynamicsModel model(RobotLimits{}, DynamicsParams{});
  std::vector<RobotState> coarse;
  std::vector<RobotState> fine;
  model.rollout(RobotState{}, { 0.0, 0.0 }, 0.13, { 1.0, 0.5 }, 1.0, 0.1, coarse);
  model.rollout(RobotState{}, { 0.0, 0.0 }, 0.13, { 1.0, 0.5 }, 1.0, 0.01, fine);
  EXPECT_NEAR(coarse.back().v, fine.back().v, 1e-9);
  EXPECT_NEAR(coarse.back().pose.x, fine.back().pose.x, 2e-3);
}

TEST(DynamicsModel, wheel_scales_and_bias_shape_the_response)
{
  DynamicsParams params;
  params.left_wheel_scale = 0.9;
  params.right_wheel_scale = 1.1;
  params.angular_bias = 0.05;
  const RobotLimits limits;
  const DynamicsModel model(limits, params);
  const Command response = model.steadyState({ 1.0, 0.0 });
  EXPECT_NEAR(response.v, 1.0, kTolerance);
  EXPECT_NEAR(response.omega, 0.2 / limits.wheel_separation + 0.05, kTolerance);
  EXPECT_EQ(model.steadyState({ 0.0, 0.0 }).omega, 0.0);
}

TEST(DynamicsModel, learned_scales_never_exceed_configured_accelerations)
{
  DynamicsParams params;
  params.accel_scale = 3.0;
  params.brake_scale = 0.5;
  const RobotLimits limits;
  const DynamicsModel model(limits, params);
  EXPECT_EQ(model.effectiveAccel(), limits.max_accel);
  EXPECT_EQ(model.effectiveBrake(), 0.5 * limits.max_brake);
}

TEST(DynamicsModel, repeated_simulation_is_bitwise_deterministic)
{
  DynamicsParams params;
  params.linear_time_constant = 0.15;
  params.delay = 0.05;
  const DynamicsModel model(RobotLimits{}, params);
  std::vector<RobotState> first;
  std::vector<RobotState> second;
  model.rollout(moving(0.4, -0.2), { 0.4, -0.2 }, 0.05, { 1.2, 0.7 }, 1.5, kDt, first);
  model.rollout(moving(0.4, -0.2), { 0.4, -0.2 }, 0.05, { 1.2, 0.7 }, 1.5, kDt, second);
  ASSERT_EQ(first.size(), second.size());
  for (size_t i = 0; i < first.size(); ++i)
  {
    EXPECT_EQ(first[i].pose.x, second[i].pose.x);
    EXPECT_EQ(first[i].pose.y, second[i].pose.y);
    EXPECT_EQ(first[i].pose.theta, second[i].pose.theta);
    EXPECT_EQ(first[i].v, second[i].v);
  }
}

TEST(DynamicsModel, validation_rejects_invalid_values)
{
  RobotLimits limits;
  EXPECT_TRUE(limits.validate().empty());
  limits.max_brake = 0.0;
  EXPECT_FALSE(limits.validate().empty());
  limits = RobotLimits{};
  limits.max_v = limits.max_wheel_speed * limits.wheel_radius + 0.01;
  EXPECT_FALSE(limits.validate().empty()) << "straight-line speed the wheels cannot reach";

  DynamicsParams params;
  EXPECT_TRUE(params.validate().empty());
  params.delay = -0.1;
  EXPECT_FALSE(params.validate().empty());
  params.delay = 0.0;
  params.linear_time_constant = std::nan("");
  EXPECT_FALSE(params.validate().empty());
}
