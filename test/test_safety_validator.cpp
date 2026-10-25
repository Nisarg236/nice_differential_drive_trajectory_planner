#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "test_helpers.hpp"

using namespace nice_differential_drive_trajectory_planner;
using namespace nice_differential_drive_trajectory_planner::test;

namespace
{
constexpr double kDt = 0.05;

class SafetyTest : public ::testing::Test
{
protected:
  SafetyTest() : params_(defaultParams()), costmap_(freeCostmap())
  {
    params_.safety.control_period = 0.05;
  }

  SafetyValidator validator()
  {
    SafetyValidator v(params_.limits, params_.footprint, params_.safety);
    v.setCostmap(&costmap_, 0);
    v.setObstacles(&obstacles_);
    return v;
  }

  /** Candidate holding command from start for duration, simulated with the nominal model. */
  Candidate hold(const RobotState& start, const Command& command, double duration = 1.0)
  {
    Candidate candidate;
    candidate.start = start;
    candidate.commands = { command };
    DynamicsModel(params_.limits, DynamicsParams{})
        .rollout(start, command, 0.0, command, duration, kDt, candidate.states);
    return candidate;
  }

  ControllerParams params_;
  nav2_costmap_2d::Costmap2D costmap_;
  ObstaclePredictionModel obstacles_;
};
}  // namespace

TEST_F(SafetyTest, feasible_candidate_in_open_space_passes)
{
  const RobotState start = stateAt(0, 0, 0, 0.8);
  EXPECT_EQ(validator().validate(hold(start, { 0.8, 0.2 }), start, DynamicsParams{}, kDt), SafetyVerdict::Ok);
}

TEST_F(SafetyTest, rejects_a_candidate_that_does_not_start_at_the_robot)
{
  const RobotState start = stateAt(0, 0, 0, 0.8);
  EXPECT_EQ(validator().validate(hold(start, { 0.8, 0.0 }), stateAt(0.1, 0, 0, 0.8), DynamicsParams{}, kDt),
            SafetyVerdict::Discontinuous);
  EXPECT_EQ(validator().validate(hold(start, { 0.8, 0.0 }), stateAt(0, 0, 0.1, 0.8), DynamicsParams{}, kDt),
            SafetyVerdict::Discontinuous);
}

TEST_F(SafetyTest, rejects_a_jump_inside_the_trajectory)
{
  const RobotState start = stateAt(0, 0, 0, 0.8);
  Candidate candidate = hold(start, { 0.8, 0.0 });
  candidate.states[5].pose.x += 2.0;
  EXPECT_EQ(validator().validate(candidate, start, DynamicsParams{}, kDt), SafetyVerdict::Discontinuous);
}

TEST_F(SafetyTest, rejects_reverse_motion)
{
  const RobotState start = stateAt(0, 0, 0, 0.0);
  Candidate candidate = hold(start, { 0.0, 0.0 });
  candidate.states[3].v = -0.1;
  EXPECT_EQ(validator().validate(candidate, start, DynamicsParams{}, kDt), SafetyVerdict::Reverse);

  candidate = hold(start, { 0.0, 0.0 });
  candidate.commands.front().v = -0.2;
  EXPECT_EQ(validator().validate(candidate, start, DynamicsParams{}, kDt), SafetyVerdict::Reverse);
}

TEST_F(SafetyTest, rejects_velocity_acceleration_and_wheel_limit_violations)
{
  const RobotState start = stateAt(0, 0, 0, 1.0);
  Candidate too_fast = hold(start, { 1.0, 0.0 });
  too_fast.states[4].v = params_.limits.max_v * 1.2;
  too_fast.states[5].v = params_.limits.max_v * 1.2;
  const SafetyVerdict fast = validator().validate(too_fast, start, DynamicsParams{}, kDt);
  EXPECT_TRUE(fast == SafetyVerdict::VelocityLimit || fast == SafetyVerdict::AccelerationLimit) << toString(fast);

  Candidate jerky = hold(start, { 1.0, 0.0 });
  jerky.states[0].v = 1.0 + params_.limits.max_accel * kDt * 2.0;
  EXPECT_EQ(validator().validate(jerky, start, DynamicsParams{}, kDt), SafetyVerdict::AccelerationLimit);

  // Within the body limits, but the outer wheel spins too fast.
  params_.limits.max_wheel_speed = 20.0;
  const RobotState turning = stateAt(0, 0, 0, 1.0, 2.0);
  Candidate wheels = hold(turning, { 1.0, 2.0 }, 0.3);
  for (auto& state : wheels.states)
  {
    state.v = 1.0;
    state.omega = 2.0;
  }
  EXPECT_EQ(validator().validate(wheels, turning, DynamicsParams{}, kDt), SafetyVerdict::WheelLimit);
}

TEST_F(SafetyTest, rejects_a_static_collision_between_samples)
{
  fill(costmap_, 0.9, -1.0, 0.9, 1.0);
  const RobotState start = stateAt(0, 0, 0, 1.0);
  Candidate candidate = hold(start, { 1.0, 0.0 }, 1.0);
  // Keep only the first and last state: the wall lies between them.
  candidate.states = { candidate.states.front(), candidate.states.back() };
  candidate.states.front().pose.x = 0.05;
  EXPECT_EQ(validator().validate(candidate, start, DynamicsParams{}, 0.5), SafetyVerdict::StaticCollision);
}

TEST_F(SafetyTest, rejects_a_predicted_obstacle_encounter)
{
  ObstacleTrack track;
  track.position = { 3.0, 0.0 };
  track.velocity = { -1.0, 0.0 };
  track.radius = 0.3;
  obstacles_.setTracks({ track }, 0.0);
  const RobotState start = stateAt(0, 0, 0, 1.0);
  EXPECT_EQ(validator().validate(hold(start, { 1.0, 0.0 }, 2.0), start, DynamicsParams{}, kDt),
            SafetyVerdict::DynamicCollision);
}

TEST_F(SafetyTest, rejects_a_first_move_that_cannot_stop_before_a_wall)
{
  // At 1.5 m/s the conservative stop (80 % of 2 m/s^2) needs about 0.7 m; the wall is 0.5 m past the bumper.
  fill(costmap_, 0.82, -1.0, 0.85, 1.0);
  const RobotState start = stateAt(0, 0, 0, 1.5);
  Candidate candidate = hold(start, { 0.0, 0.0 }, 0.05);
  EXPECT_EQ(validator().validate(candidate, start, DynamicsParams{}, kDt), SafetyVerdict::CannotStop);

  const RobotState slow = stateAt(0, 0, 0, 0.3);
  EXPECT_EQ(validator().validate(hold(slow, { 0.0, 0.0 }, 0.05), slow, DynamicsParams{}, kDt), SafetyVerdict::Ok);
}

TEST_F(SafetyTest, learned_parameters_never_loosen_the_stop_check)
{
  // A stop that only fits with the full configured braking, not the conservative 80 %.
  fill(costmap_, 1.05, -1.0, 1.1, 1.0);
  const RobotState start = stateAt(0, 0, 0, 1.6);
  const Candidate candidate = hold(start, { 0.0, 0.0 }, 0.05);
  DynamicsParams optimistic;
  optimistic.brake_scale = 5.0;
  EXPECT_EQ(validator().validate(candidate, start, optimistic, kDt), SafetyVerdict::CannotStop);

  // A longer learned delay makes the check stricter: the robot keeps cruising for longer before it brakes.
  const RobotState slow = stateAt(0, 0, 0, 1.0);
  const Candidate gentle = hold(slow, { 1.0, 0.0 }, 0.05);
  EXPECT_EQ(validator().validate(gentle, slow, DynamicsParams{}, kDt), SafetyVerdict::Ok);
  DynamicsParams laggy;
  laggy.delay = 0.4;
  EXPECT_EQ(validator().validate(gentle, slow, laggy, kDt), SafetyVerdict::CannotStop);
}

TEST_F(SafetyTest, braking_command_ramps_down_within_limits)
{
  const SafetyValidator v = validator();
  const Command fast = v.brakingCommand(stateAt(0, 0, 0, 1.0, 1.0));
  EXPECT_NEAR(fast.v, 1.0 - params_.limits.max_brake * params_.safety.control_period, 1e-9);
  EXPECT_NEAR(fast.omega, 1.0 - params_.limits.max_angular_accel * params_.safety.control_period, 1e-9);

  const Command almost = v.brakingCommand(stateAt(0, 0, 0, 0.05, -0.1));
  EXPECT_EQ(almost.v, 0.0);
  EXPECT_EQ(almost.omega, 0.0);
}

TEST_F(SafetyTest, stale_inputs_are_detected)
{
  const SafetyValidator v = validator();
  EXPECT_TRUE(v.inputsFresh({ 0.0, 0.0, 10.0 }));
  EXPECT_FALSE(v.inputsFresh({ params_.safety.max_state_age + 0.1, 0.0, 0.0 }));
  EXPECT_FALSE(v.inputsFresh({ 0.0, params_.safety.max_costmap_age + 0.1, 0.0 }));
  params_.safety.require_obstacle_tracks = true;
  EXPECT_FALSE(validator().inputsFresh({ 0.0, 0.0, params_.safety.max_obstacle_age + 0.1 }));
}

TEST_F(SafetyTest, current_pose_in_collision_is_reported)
{
  fill(costmap_, -0.05, -0.05, 0.05, 0.05);
  EXPECT_TRUE(validator().currentPoseInCollision(stateAt(0, 0)));
  EXPECT_FALSE(validator().currentPoseInCollision(stateAt(2, 2)));
}
