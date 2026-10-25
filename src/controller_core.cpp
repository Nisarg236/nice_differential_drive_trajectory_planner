#include "nice_differential_drive_trajectory_planner/controller_core.hpp"

#include <algorithm>
#include <cmath>

namespace nice_differential_drive_trajectory_planner
{
namespace
{
constexpr double kStoppedSpeed = 0.01;
constexpr double kTurnCheckTime = 0.6;

bool finite(const RobotState& state)
{
  return std::isfinite(state.pose.x) && std::isfinite(state.pose.y) && std::isfinite(state.pose.theta) &&
         std::isfinite(state.v) && std::isfinite(state.omega);
}

std::string prefixed(const std::string& prefix, const std::string& error)
{
  return error.empty() ? error : prefix + error;
}
}  // namespace

std::string ControllerParams::validate() const
{
  for (const std::string& error :
       { prefixed("limits: ", limits.validate()), prefixed("dynamics: ", nominal_dynamics.validate()),
         prefixed("speed_profile: ", speed_profile.validate()), prefixed("primitives: ", primitives.validate()),
         prefixed("search: ", search.validate(primitives)), prefixed("scoring: ", scoring.validate()),
         prefixed("collision: ", collision.validate()), prefixed("safety: ", safety.validate()),
         prefixed("obstacles: ", obstacles.validate()) })
  {
    if (!error.empty())
    {
      return error;
    }
  }
  if (footprint.size() < 3)
  {
    return "footprint needs at least three points";
  }
  if (collision.resolution > safety.collision.resolution)
  {
    return "collision.resolution must not be coarser than safety.resolution";
  }
  if (nominal_dynamics.delay > primitives.duration)
  {
    return "dynamics delay must not exceed the primitive duration";
  }
  if (!(route_start_window > 0.0) || !(goal_yaw_gain > 0.0) || !(goal_yaw_tolerance > 0.0))
  {
    return "route_start_window, goal_yaw_gain and goal_yaw_tolerance must be positive";
  }
  if (max_fallback_cycles < 1)
  {
    return "max_fallback_cycles must be at least 1";
  }
  return {};
}

const char* toString(ControlStatus status)
{
  switch (status)
  {
    case ControlStatus::Tracking:
      return "tracking";
    case ControlStatus::TurningToGoal:
      return "turning_to_goal";
    case ControlStatus::GoalReached:
      return "goal_reached";
    case ControlStatus::Fallback:
      return "fallback";
    case ControlStatus::NoValidControl:
      return "no_valid_control";
    case ControlStatus::InvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

ControllerCore::ControllerCore(const ControllerParams& params)
  : params_(params)
  , snapshots_(std::make_shared<const ModelSnapshot>(ModelSnapshot{ params.nominal_dynamics, 0 }))
  , obstacles_(params.obstacles)
  , checker_(params.footprint, params.collision)
  , search_(params.search, params.primitives, params.scoring, params.limits)
  , validator_(params.limits, params.footprint, params.safety)
{
}

void ControllerCore::reset()
{
  last_command_ = Command{};
  has_previous_maneuver_ = false;
  fallback_cycles_ = 0;
  route_.clear();
  obstacles_.clear();
}

ControlOutput ControllerCore::fallback(ControlOutput output, const RobotState& state, const std::string& reason)
{
  output.command = validator_.brakingCommand(state);
  output.reason = reason;
  ++fallback_cycles_;
  const bool stopped = state.v < kStoppedSpeed && std::abs(state.omega) < kStoppedSpeed;
  if (stopped && fallback_cycles_ > params_.max_fallback_cycles)
  {
    output.status = ControlStatus::NoValidControl;
  }
  else if (output.status != ControlStatus::InvalidInput)
  {
    output.status = ControlStatus::Fallback;
  }
  last_command_ = output.command;
  has_previous_maneuver_ = false;
  return output;
}

ControlOutput ControllerCore::turnToGoal(ControlOutput output, const ControlInput& input,
                                         const DynamicsParams& learned)
{
  const RobotState& state = input.state;
  const double error = wrapAngle(input.goal_heading - state.pose.theta);
  if (std::abs(error) < params_.goal_yaw_tolerance && state.v < kStoppedSpeed)
  {
    output.status = ControlStatus::GoalReached;
    output.command = Command{};
    last_command_ = output.command;
    fallback_cycles_ = 0;
    return output;
  }

  const double period = params_.safety.control_period;
  const double angular_step = params_.limits.max_angular_accel * period;
  // Half the angular deceleration leaves room for the delay between command and measured yaw rate.
  const double braking_rate = std::sqrt(params_.limits.max_angular_accel * std::abs(error));
  Command command;
  command.v = std::max(0.0, state.v - params_.limits.max_brake * period);
  command.omega = std::copysign(
      std::min({ params_.goal_yaw_gain * std::abs(error), braking_rate, params_.primitives.rotate_speed }), error);
  command.omega = std::clamp(command.omega, state.omega - angular_step, state.omega + angular_step);

  const DynamicsModel model(params_.limits, learned);
  const double dt = params_.primitives.integration_dt;
  Candidate turn;
  turn.start = state;
  turn.commands = { command };
  model.rollout(state, last_command_, learned.delay, command, dt * std::ceil(kTurnCheckTime / dt), dt, turn.states);
  const SafetyVerdict verdict = validator_.validate(turn, state, learned, dt);
  if (verdict != SafetyVerdict::Ok)
  {
    output.first_rejection = verdict;
    return fallback(output, state, std::string("goal turn rejected: ") + toString(verdict));
  }
  output.status = ControlStatus::TurningToGoal;
  output.command = model.saturate(command);
  last_command_ = output.command;
  fallback_cycles_ = 0;
  return output;
}

ControlOutput ControllerCore::compute(const ControlInput& input)
{
  ControlOutput output;
  candidates_.clear();
  verdicts_.clear();
  const ModelSnapshotConstPtr snapshot = snapshots_.current();
  output.model_version = snapshot->version;
  output.goal_position_reached = input.goal_position_reached;
  output.goal_heading = input.goal_heading;
  const RobotState& state = input.state;

  if (!finite(state))
  {
    output.status = ControlStatus::InvalidInput;
    return fallback(output, RobotState{}, "robot state is not finite");
  }
  if (!validator_.inputsFresh(input.ages))
  {
    output.status = ControlStatus::InvalidInput;
    return fallback(output, state, "stale robot state, costmap or obstacle tracks");
  }

  checker_.setCostmap(input.costmap, input.circumscribed_cost);
  validator_.setCostmap(input.costmap, input.circumscribed_cost);
  static const std::vector<ObstacleTrack> no_tracks;
  obstacles_.setTracks(input.tracks != nullptr ? *input.tracks : no_tracks, input.now);
  checker_.setObstacles(&obstacles_);
  validator_.setObstacles(&obstacles_);

  if (validator_.currentPoseInCollision(state))
  {
    return fallback(output, state, "robot footprint is in collision");
  }
  if (input.goal_position_reached)
  {
    return turnToGoal(output, input, snapshot->params);
  }

  SpeedProfileParams speed = params_.speed_profile;
  speed.max_speed = std::min(speed.max_speed, params_.limits.max_v);
  if (input.speed_limit > 0.0)
  {
    speed.max_speed = std::min(speed.max_speed, input.speed_limit);
  }
  speed.end_speed = input.route_ends_at_goal ? 0.0 : -1.0;
  if (!route_.build(input.route, speed, input.speed_caps))
  {
    output.status = ControlStatus::InvalidInput;
    return fallback(output, state, "route has fewer than two distinct points");
  }

  const RouteProjection projection = route_.project(position(state.pose), 0.0, params_.route_start_window);
  if (!projection.valid || projection.distance > params_.scoring.max_route_deviation)
  {
    return fallback(output, state, "robot is outside max_route_deviation");
  }
  output.route_progress = projection.s;
  output.route_deviation = projection.lateral;
  output.reference_speed = route_.referenceSpeed(projection.s);

  const DynamicsModel model(params_.limits, snapshot->params);
  SearchInput search_input;
  search_input.state = state;
  search_input.pending = last_command_;
  search_input.s0 = projection.s;
  search_input.lateral0 = projection.lateral;
  search_input.route = &route_;
  search_input.model = &model;
  search_input.checker = &checker_;
  search_input.has_previous_maneuver = has_previous_maneuver_;
  search_input.previous_maneuver = previous_maneuver_;
  search_.plan(search_input, candidates_, output.stats);
  output.candidates = candidates_.size();

  for (const Candidate& candidate : candidates_)
  {
    const SafetyVerdict verdict =
        validator_.validate(candidate, state, snapshot->params, params_.primitives.integration_dt);
    verdicts_.push_back(verdict);
    if (verdict != SafetyVerdict::Ok)
    {
      if (output.candidates_rejected++ == 0)
      {
        output.first_rejection = verdict;
      }
      continue;
    }
    output.status = ControlStatus::Tracking;
    output.command = model.saturate(candidate.commands.front());
    output.has_selection = true;
    output.selected = candidate;
    last_command_ = output.command;
    previous_maneuver_ = candidate.maneuver;
    has_previous_maneuver_ = true;
    fallback_cycles_ = 0;
    return output;
  }
  return fallback(output, state, candidates_.empty() ? "search found no feasible candidate" :
                                                       std::string("every candidate failed validation, first: ") +
                                                           toString(output.first_rejection));
}

}  // namespace nice_differential_drive_trajectory_planner
