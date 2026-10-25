#include "nice_differential_drive_trajectory_planner/dynamics_model.hpp"

#include <algorithm>
#include <cmath>

namespace nice_differential_drive_trajectory_planner
{
namespace
{
constexpr double kMotionEpsilon = 1e-6;

bool positiveFinite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

/** First-order lag towards target over dt, with the change per step limited by the rise and fall rates. */
double approach(double value, double target, double time_constant, double rise_rate, double fall_rate, double dt)
{
  const double lagged = time_constant > 0.0 ? target + (value - target) * std::exp(-dt / time_constant) : target;
  return std::clamp(lagged, value - fall_rate * dt, value + rise_rate * dt);
}

/** Mean of a value that ramps at a constant rate from start and holds once it reaches end within dt. */
double rampMean(double start, double end, double rate, double dt)
{
  const double ramp_time = rate > 0.0 ? std::min(dt, std::abs(end - start) / rate) : dt;
  return ((start + end) / 2.0 * ramp_time + end * (dt - ramp_time)) / dt;
}
}  // namespace

std::string RobotLimits::validate() const
{
  if (!positiveFinite(max_v))
  {
    return "max_v must be positive";
  }
  if (!positiveFinite(max_omega))
  {
    return "max_omega must be positive";
  }
  if (!positiveFinite(max_accel) || !positiveFinite(max_brake))
  {
    return "max_accel and max_brake must be positive";
  }
  if (!positiveFinite(max_angular_accel))
  {
    return "max_angular_accel must be positive";
  }
  if (!positiveFinite(wheel_radius) || !positiveFinite(wheel_separation) || !positiveFinite(max_wheel_speed))
  {
    return "wheel_radius, wheel_separation and max_wheel_speed must be positive";
  }
  if (max_v > max_wheel_speed * wheel_radius)
  {
    return "max_v is faster than the wheels can drive (max_wheel_speed * wheel_radius)";
  }
  return {};
}

std::string DynamicsParams::validate() const
{
  if (!std::isfinite(linear_time_constant) || linear_time_constant < 0.0 || !std::isfinite(angular_time_constant) ||
      angular_time_constant < 0.0)
  {
    return "time constants must be finite and non-negative";
  }
  if (!positiveFinite(accel_scale) || !positiveFinite(brake_scale) || !positiveFinite(angular_accel_scale))
  {
    return "acceleration scales must be positive";
  }
  if (!std::isfinite(delay) || delay < 0.0)
  {
    return "delay must be finite and non-negative";
  }
  if (!positiveFinite(left_wheel_scale) || !positiveFinite(right_wheel_scale))
  {
    return "wheel scales must be positive";
  }
  if (!std::isfinite(angular_bias))
  {
    return "angular_bias must be finite";
  }
  return {};
}

DynamicsModel::DynamicsModel(const RobotLimits& limits, const DynamicsParams& params)
  : limits_(limits), params_(params)
{
}

Command DynamicsModel::saturate(const Command& command) const
{
  const double v = std::max(0.0, command.v);
  const double omega = command.omega;
  const double half_track = limits_.wheel_separation / 2.0;
  const double max_wheel = std::max(std::abs(v + omega * half_track), std::abs(v - omega * half_track)) /
                           limits_.wheel_radius;

  double scale = 1.0;
  if (v > limits_.max_v)
  {
    scale = std::min(scale, limits_.max_v / v);
  }
  if (std::abs(omega) > limits_.max_omega)
  {
    scale = std::min(scale, limits_.max_omega / std::abs(omega));
  }
  if (max_wheel > limits_.max_wheel_speed)
  {
    scale = std::min(scale, limits_.max_wheel_speed / max_wheel);
  }
  return { v * scale, omega * scale };
}

Command DynamicsModel::steadyState(const Command& command) const
{
  const double sum = params_.right_wheel_scale + params_.left_wheel_scale;
  const double diff = params_.right_wheel_scale - params_.left_wheel_scale;
  const double b = limits_.wheel_separation;

  Command response;
  response.v = command.v * sum / 2.0 + command.omega * b * diff / 4.0;
  response.omega = command.v * diff / b + command.omega * sum / 2.0;
  if (std::abs(command.v) > kMotionEpsilon || std::abs(command.omega) > kMotionEpsilon)
  {
    response.omega += params_.angular_bias;
  }
  response.v = std::max(0.0, response.v);
  return response;
}

void DynamicsModel::step(RobotState& state, const Command& acting, double dt) const
{
  if (dt <= 0.0)
  {
    return;
  }
  const Command target = steadyState(saturate(acting));
  const double v0 = state.v;
  const double omega0 = state.omega;

  const double accel = effectiveAccel();
  const double brake = effectiveBrake();
  const double angular_accel = effectiveAngularAccel();
  state.v = std::max(0.0, approach(v0, target.v, params_.linear_time_constant, accel, brake, dt));
  state.omega = approach(omega0, target.omega, params_.angular_time_constant, angular_accel, angular_accel, dt);

  const double v_mean = params_.linear_time_constant > 0.0 ?
                            (v0 + state.v) / 2.0 :
                            rampMean(v0, state.v, state.v >= v0 ? accel : brake, dt);
  const double omega_mean = params_.angular_time_constant > 0.0 ? (omega0 + state.omega) / 2.0 :
                                                                  rampMean(omega0, state.omega, angular_accel, dt);
  state.pose = integrate(state.pose, v_mean, omega_mean, dt);
}

void DynamicsModel::rollout(const RobotState& start, const Command& pending, double pending_time,
                            const Command& command, double duration, double dt, std::vector<RobotState>& out) const
{
  const long steps = std::lround(duration / dt);
  RobotState state = start;
  for (long k = 0; k < steps; ++k)
  {
    const double t0 = static_cast<double>(k) * dt;
    const double t1 = t0 + dt;
    if (pending_time >= t1)
    {
      step(state, pending, dt);
    }
    else if (pending_time <= t0)
    {
      step(state, command, dt);
    }
    else
    {
      step(state, pending, pending_time - t0);
      step(state, command, t1 - pending_time);
    }
    out.push_back(state);
  }
}

Pose2D DynamicsModel::integrate(const Pose2D& pose, double v, double omega, double dt)
{
  const double dtheta = omega * dt;
  Pose2D result;
  result.theta = wrapAngle(pose.theta + dtheta);
  if (std::abs(dtheta) < 1e-9)
  {
    result.x = pose.x + v * dt * std::cos(pose.theta + dtheta / 2.0);
    result.y = pose.y + v * dt * std::sin(pose.theta + dtheta / 2.0);
  }
  else
  {
    const double radius = v / omega;
    result.x = pose.x + radius * (std::sin(pose.theta + dtheta) - std::sin(pose.theta));
    result.y = pose.y - radius * (std::cos(pose.theta + dtheta) - std::cos(pose.theta));
  }
  return result;
}

double DynamicsModel::effectiveAccel() const
{
  return limits_.max_accel * std::min(1.0, params_.accel_scale);
}

double DynamicsModel::effectiveBrake() const
{
  return limits_.max_brake * std::min(1.0, params_.brake_scale);
}

double DynamicsModel::effectiveAngularAccel() const
{
  return limits_.max_angular_accel * std::min(1.0, params_.angular_accel_scale);
}

}  // namespace nice_differential_drive_trajectory_planner
