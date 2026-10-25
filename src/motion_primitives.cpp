#include "nice_differential_drive_trajectory_planner/motion_primitives.hpp"

#include <algorithm>
#include <cmath>

namespace nice_differential_drive_trajectory_planner
{
namespace
{
constexpr double kSpeedTolerance = 0.02;
constexpr double kYawRateTolerance = 0.02;
constexpr double kMinTrackingSpeed = 0.02;

void addUnique(std::vector<double>& values, double value, double tolerance)
{
  for (double existing : values)
  {
    if (std::abs(existing - value) < tolerance)
    {
      return;
    }
  }
  values.push_back(value);
}

bool contains(const std::vector<Primitive>& primitives, const Command& command)
{
  for (const auto& primitive : primitives)
  {
    if (std::abs(primitive.command.v - command.v) < kSpeedTolerance &&
        std::abs(primitive.command.omega - command.omega) < kYawRateTolerance)
    {
      return true;
    }
  }
  return false;
}
}  // namespace

const char* toString(Maneuver maneuver)
{
  switch (maneuver)
  {
    case Maneuver::Follow:
      return "follow";
    case Maneuver::PassLeft:
      return "pass_left";
    case Maneuver::PassRight:
      return "pass_right";
    case Maneuver::Slow:
      return "slow";
    case Maneuver::Stop:
      return "stop";
    case Maneuver::Rotate:
      return "rotate";
  }
  return "unknown";
}

std::string PrimitiveParams::validate() const
{
  if (!(duration > 0.0) || !(integration_dt > 0.0) || integration_dt > duration)
  {
    return "primitive duration and integration_dt must be positive with integration_dt <= duration";
  }
  const double steps = duration / integration_dt;
  if (std::abs(steps - std::round(steps)) > 1e-6)
  {
    return "primitive duration must be a multiple of integration_dt";
  }
  if (lateral_offsets.empty())
  {
    return "lateral_offsets must not be empty";
  }
  if (speed_levels < 0 || angular_levels < 0 || dense_angular_levels < 0)
  {
    return "sample levels must not be negative";
  }
  if (!(lookahead_time > 0.0) || !(min_lookahead > 0.0))
  {
    return "lookahead_time and min_lookahead must be positive";
  }
  if (!(rotate_speed > 0.0) || !(rotate_heading_threshold > 0.0))
  {
    return "rotate_speed and rotate_heading_threshold must be positive";
  }
  if (max_successors < 2)
  {
    return "max_successors must be at least 2";
  }
  return {};
}

MotionPrimitiveGenerator::MotionPrimitiveGenerator(const PrimitiveParams& params) : params_(params)
{
}

Point2D MotionPrimitiveGenerator::trackingTarget(const Route& route, double s, double offset, double speed) const
{
  const double lookahead = std::max(params_.min_lookahead, speed * params_.lookahead_time);
  const RouteSample target = route.sample(s + lookahead);
  return { target.position.x - offset * std::sin(target.heading),
           target.position.y + offset * std::cos(target.heading) };
}

double MotionPrimitiveGenerator::trackingYawRate(const RobotState& state, const Route& route, double s,
                                                 double offset, double speed) const
{
  if (speed < kMinTrackingSpeed)
  {
    return 0.0;
  }
  const Point2D point = trackingTarget(route, s, offset, speed);
  const double dx = point.x - state.pose.x;
  const double dy = point.y - state.pose.y;
  const double range = std::hypot(dx, dy);
  if (range < 1e-6)
  {
    return 0.0;
  }
  const double alpha = wrapAngle(std::atan2(dy, dx) - state.pose.theta);
  return speed * 2.0 * std::sin(alpha) / range;
}

void MotionPrimitiveGenerator::generate(const RobotState& state, const Route& route, double s, bool dense,
                                        const DynamicsModel& model, std::vector<Primitive>& out) const
{
  out.clear();
  const RobotLimits& limits = model.limits();
  const double v_low = std::max(0.0, state.v - model.effectiveBrake() * params_.duration);
  const double v_high = std::min(limits.max_v, state.v + model.effectiveAccel() * params_.duration);
  const double v_ref = route.referenceSpeed(s);

  auto push = [&](const Command& command, Maneuver maneuver) {
    if (out.size() < params_.max_successors && !contains(out, command))
    {
      out.push_back({ command, maneuver });
    }
  };

  push({ 0.0, 0.0 }, Maneuver::Stop);

  const double heading_error = wrapAngle(route.sample(s).heading - state.pose.theta);
  if (std::abs(heading_error) > params_.rotate_heading_threshold)
  {
    const double towards = std::copysign(params_.rotate_speed, heading_error);
    push({ 0.0, towards }, Maneuver::Rotate);
    if (dense)
    {
      push({ 0.0, -towards }, Maneuver::Rotate);
    }
  }

  std::vector<double> speeds;
  addUnique(speeds, std::clamp(v_ref, v_low, v_high), kSpeedTolerance);
  addUnique(speeds, std::clamp(state.v, v_low, v_high), kSpeedTolerance);
  addUnique(speeds, v_low, kSpeedTolerance);
  if (dense || v_high < v_ref)
  {
    addUnique(speeds, v_high, kSpeedTolerance);
  }
  for (int level = 1; dense && level <= params_.speed_levels; ++level)
  {
    addUnique(speeds, v_low + (v_high - v_low) * level / (params_.speed_levels + 1), kSpeedTolerance);
  }

  for (double offset : params_.lateral_offsets)
  {
    for (double speed : speeds)
    {
      const Maneuver maneuver = offset > 0.0 ? Maneuver::PassLeft :
                                offset < 0.0 ? Maneuver::PassRight :
                                (speed < 0.5 * v_ref && speed < state.v) ? Maneuver::Slow :
                                                                           Maneuver::Follow;
      push({ speed, trackingYawRate(state, route, s, offset, speed) }, maneuver);
    }
  }

  const int levels = dense ? params_.dense_angular_levels : params_.angular_levels;
  const double hold = std::clamp(state.v, v_low, v_high);
  for (int level = 1; level <= levels; ++level)
  {
    const double omega = limits.max_omega * level / (levels + 1);
    push({ hold, omega }, Maneuver::PassLeft);
    push({ hold, -omega }, Maneuver::PassRight);
  }
}

}  // namespace nice_differential_drive_trajectory_planner
