#include "nice_differential_drive_trajectory_planner/trajectory_scorer.hpp"

#include <algorithm>
#include <cmath>

namespace nice_differential_drive_trajectory_planner
{
namespace
{
constexpr double kMaxRiskCost = 252.0;
constexpr double kMinTarget = 1e-3;
}  // namespace

const char* toString(CostTerm term)
{
  switch (term)
  {
    case kProgress:
      return "progress";
    case kRoute:
      return "route";
    case kVelocity:
      return "velocity";
    case kAcceleration:
      return "acceleration";
    case kAngular:
      return "angular";
    case kObstacle:
      return "obstacle";
    case kTerminal:
      return "terminal";
    case kCommitment:
      return "commitment";
    case kCostTermCount:
      break;
  }
  return "unknown";
}

std::string ScoringParams::validate() const
{
  for (double weight : weights)
  {
    if (!(weight >= 0.0) || !std::isfinite(weight))
    {
      return "cost weights must be finite and non-negative";
    }
  }
  if (!(filter_dist > 0.0) || !(max_route_deviation > filter_dist))
  {
    return "need 0 < filter_dist < max_route_deviation";
  }
  if (!(obstacle_distance_scale > 0.0))
  {
    return "obstacle_distance_scale must be positive";
  }
  if (!(yaw_flip_penalty >= 0.0) || !(yaw_flip_threshold >= 0.0))
  {
    return "yaw_flip_penalty and yaw_flip_threshold must not be negative";
  }
  return {};
}

TrajectoryScorer::TrajectoryScorer(const ScoringParams& params, const RobotLimits& limits, double horizon_time)
  : params_(params), limits_(limits), horizon_time_(horizon_time)
{
}

double TrajectoryScorer::routeCost(double distance) const
{
  const double d = std::abs(distance);
  if (d <= params_.filter_dist)
  {
    const double ratio = d / params_.filter_dist;
    return ratio * ratio;
  }
  const double remaining = std::max(1e-3, params_.max_route_deviation - d);
  return 1.0 + (d - params_.filter_dist) / remaining;
}

void TrajectoryScorer::addSegment(const RobotState& before, const RobotState* states, const double* s,
                                  const double* lateral, size_t count, double dt, const Route& route,
                                  const Command& parent_command, const Command& command, const SweepResult& sweep,
                                  CostVector& cost) const
{
  const double weight = dt / horizon_time_;
  double previous_v = before.v;
  for (size_t k = 0; k < count; ++k)
  {
    const RobotState& state = states[k];
    const RouteSample reference = route.sample(s[k]);

    cost[kRoute] += routeCost(lateral[k]) * weight;

    const double speed_error = (state.v - reference.v_ref) / limits_.max_v;
    cost[kVelocity] += speed_error * speed_error * weight;

    const double accel = (state.v - previous_v) / dt / limits_.max_accel;
    cost[kAcceleration] += accel * accel * weight;
    previous_v = state.v;

    const double yaw_error = (state.omega - state.v * reference.curvature) / limits_.max_omega;
    cost[kAngular] += yaw_error * yaw_error * weight;
  }

  if (std::abs(parent_command.omega) > params_.yaw_flip_threshold &&
      std::abs(command.omega) > params_.yaw_flip_threshold && parent_command.omega * command.omega < 0.0)
  {
    cost[kAngular] += params_.yaw_flip_penalty;
  }

  const double duration = dt * static_cast<double>(count) / horizon_time_;
  const double static_risk = sweep.max_static_cost / kMaxRiskCost;
  cost[kObstacle] += static_risk * static_risk * duration;
  if (std::isfinite(sweep.min_dynamic_clearance))
  {
    cost[kObstacle] += std::exp(-std::max(0.0, sweep.min_dynamic_clearance) / params_.obstacle_distance_scale) *
                       duration;
  }
}

double TrajectoryScorer::progressShortfall(double progress, double target) const
{
  if (target < kMinTarget)
  {
    return 0.0;
  }
  return std::clamp((target - progress) / target, 0.0, 1.0);
}

void TrajectoryScorer::addHold(const RobotState& final_state, double s, double lateral, double static_cost,
                               double duration, const Route& route, CostVector& cost) const
{
  const double weight = duration / horizon_time_;
  const double speed_error = (final_state.v - route.referenceSpeed(s)) / limits_.max_v;
  const double static_risk = static_cost / kMaxRiskCost;
  cost[kVelocity] += speed_error * speed_error * weight;
  cost[kRoute] += routeCost(lateral) * weight;
  cost[kObstacle] += static_risk * static_risk * weight;
}

double TrajectoryScorer::terminalCost(const RobotState& final_state, double time, double s, const Route& route,
                                      bool cut_short) const
{
  const double unfinished = cut_short ? std::clamp((horizon_time_ - time) / horizon_time_, 0.0, 1.0) : 0.0;
  const double heading_error = wrapAngle(route.sample(s).heading - final_state.pose.theta) / M_PI;
  return unfinished + heading_error * heading_error;
}

double TrajectoryScorer::total(const CostVector& cost) const
{
  double sum = 0.0;
  for (size_t i = 0; i < kCostTermCount; ++i)
  {
    if (params_.enabled[i])
    {
      sum += params_.weights[i] * cost[i];
    }
  }
  return sum;
}

}  // namespace nice_differential_drive_trajectory_planner
