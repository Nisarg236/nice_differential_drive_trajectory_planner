#include "nice_differential_drive_trajectory_planner/safety_validator.hpp"

#include <algorithm>
#include <cmath>

namespace nice_differential_drive_trajectory_planner
{
namespace
{
constexpr double kStoppedSpeed = 0.01;
}  // namespace

std::string SafetyParams::validate() const
{
  const std::string collision_error = collision.validate();
  if (!collision_error.empty())
  {
    return "safety " + collision_error;
  }
  if (!(brake_factor > 0.0) || brake_factor > 1.0)
  {
    return "brake_factor must be within (0, 1]";
  }
  if (!(min_delay >= 0.0) || !(control_period > 0.0))
  {
    return "min_delay must not be negative and control_period must be positive";
  }
  if (!(limit_tolerance >= 1.0))
  {
    return "limit_tolerance must be at least 1";
  }
  if (!(start_position_tolerance > 0.0) || !(start_heading_tolerance > 0.0))
  {
    return "start tolerances must be positive";
  }
  if (!(max_state_age > 0.0) || !(max_costmap_age > 0.0) || !(max_obstacle_age > 0.0))
  {
    return "input ages must be positive";
  }
  return {};
}

const char* toString(SafetyVerdict verdict)
{
  switch (verdict)
  {
    case SafetyVerdict::Ok:
      return "ok";
    case SafetyVerdict::StaleInput:
      return "stale_input";
    case SafetyVerdict::Discontinuous:
      return "discontinuous";
    case SafetyVerdict::Reverse:
      return "reverse";
    case SafetyVerdict::VelocityLimit:
      return "velocity_limit";
    case SafetyVerdict::AccelerationLimit:
      return "acceleration_limit";
    case SafetyVerdict::WheelLimit:
      return "wheel_limit";
    case SafetyVerdict::StaticCollision:
      return "static_collision";
    case SafetyVerdict::DynamicCollision:
      return "dynamic_collision";
    case SafetyVerdict::CannotStop:
      return "cannot_stop";
  }
  return "unknown";
}

SafetyValidator::SafetyValidator(const RobotLimits& limits, const Footprint& footprint, const SafetyParams& params)
  : limits_(limits), params_(params), checker_(footprint, params.collision)
{
}

void SafetyValidator::setCostmap(const nav2_costmap_2d::Costmap2D* costmap, unsigned char circumscribed_cost)
{
  checker_.setCostmap(costmap, circumscribed_cost);
}

void SafetyValidator::setObstacles(const ObstaclePredictionModel* obstacles)
{
  checker_.setObstacles(obstacles);
}

bool SafetyValidator::inputsFresh(const InputAges& ages) const
{
  return ages.state <= params_.max_state_age && ages.costmap <= params_.max_costmap_age &&
         (!params_.require_obstacle_tracks || ages.obstacles <= params_.max_obstacle_age);
}

DynamicsModel SafetyValidator::stopModel(const DynamicsParams& learned) const
{
  DynamicsParams conservative = learned;
  conservative.brake_scale = std::min(params_.brake_factor, learned.brake_scale);
  conservative.delay = std::max(params_.min_delay, learned.delay);
  return DynamicsModel(limits_, conservative);
}

bool SafetyValidator::currentPoseInCollision(const RobotState& current) const
{
  return checker_.staticCollision(current.pose) || checker_.dynamicClearance(current.pose, 0.0) < 0.0;
}

SafetyVerdict SafetyValidator::validate(const Candidate& candidate, const RobotState& current,
                                        const DynamicsParams& learned, double integration_dt) const
{
  if (candidate.states.empty() || candidate.commands.empty())
  {
    return SafetyVerdict::Discontinuous;
  }
  if (std::hypot(candidate.start.pose.x - current.pose.x, candidate.start.pose.y - current.pose.y) >
          params_.start_position_tolerance ||
      std::abs(wrapAngle(candidate.start.pose.theta - current.pose.theta)) > params_.start_heading_tolerance)
  {
    return SafetyVerdict::Discontinuous;
  }

  const double tolerance = params_.limit_tolerance;
  const double half_track = limits_.wheel_separation / 2.0;
  const double max_jump = std::max(limits_.max_v, limits_.max_omega * half_track) * integration_dt * 4.0;
  RobotState previous = candidate.start;
  for (const RobotState& state : candidate.states)
  {
    if (state.v < 0.0)
    {
      return SafetyVerdict::Reverse;
    }
    if (state.v > limits_.max_v * tolerance || std::abs(state.omega) > limits_.max_omega * tolerance)
    {
      return SafetyVerdict::VelocityLimit;
    }
    const double accel = (state.v - previous.v) / integration_dt;
    if (accel > limits_.max_accel * tolerance || -accel > limits_.max_brake * tolerance ||
        std::abs(state.omega - previous.omega) / integration_dt > limits_.max_angular_accel * tolerance)
    {
      return SafetyVerdict::AccelerationLimit;
    }
    const double wheel =
        std::max(std::abs(state.v + state.omega * half_track), std::abs(state.v - state.omega * half_track)) /
        limits_.wheel_radius;
    if (wheel > limits_.max_wheel_speed * tolerance)
    {
      return SafetyVerdict::WheelLimit;
    }
    if (std::hypot(state.pose.x - previous.pose.x, state.pose.y - previous.pose.y) > max_jump)
    {
      return SafetyVerdict::Discontinuous;
    }
    previous = state;
  }
  for (const Command& command : candidate.commands)
  {
    if (command.v < 0.0)
    {
      return SafetyVerdict::Reverse;
    }
  }

  const SweepResult sweep =
      checker_.sweep(candidate.start.pose, 0.0, candidate.states.data(), candidate.states.size(), integration_dt);
  if (sweep.static_collision)
  {
    return SafetyVerdict::StaticCollision;
  }
  if (sweep.dynamic_collision)
  {
    return SafetyVerdict::DynamicCollision;
  }
  if (!canStopAfter(current, candidate.commands.front(), learned, integration_dt))
  {
    return SafetyVerdict::CannotStop;
  }
  return SafetyVerdict::Ok;
}

bool SafetyValidator::canStopAfter(const RobotState& current, const Command& first, const DynamicsParams& learned,
                                   double integration_dt) const
{
  std::vector<RobotState> states;
  stoppingStates(current, first, learned, integration_dt, states);
  if (states.empty() || states.back().v > kStoppedSpeed)
  {
    return false;
  }
  return !checker_.sweep(current.pose, 0.0, states.data(), states.size(), integration_dt).collision();
}

void SafetyValidator::stoppingStates(const RobotState& current, const Command& first, const DynamicsParams& learned,
                                     double integration_dt, std::vector<RobotState>& out) const
{
  const DynamicsModel model = stopModel(learned);
  const double hold = model.params().delay + params_.control_period;
  const double peak = std::min(limits_.max_v, current.v + model.effectiveAccel() * hold);
  const double duration = integration_dt * std::ceil((hold + peak / model.effectiveBrake()) / integration_dt + 1.0);
  out.clear();
  model.rollout(current, first, hold, Command{}, duration, integration_dt, out);
}

Command SafetyValidator::brakingCommand(const RobotState& current) const
{
  const double dv = limits_.max_brake * params_.control_period;
  const double domega = limits_.max_angular_accel * params_.control_period;
  Command command;
  command.v = std::max(0.0, current.v - dv);
  command.omega = std::abs(current.omega) <= domega ? 0.0 : current.omega - std::copysign(domega, current.omega);
  return command;
}

}  // namespace nice_differential_drive_trajectory_planner
