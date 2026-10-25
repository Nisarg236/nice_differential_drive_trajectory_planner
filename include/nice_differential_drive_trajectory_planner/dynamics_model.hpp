#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "nice_differential_drive_trajectory_planner/types.hpp"

namespace nice_differential_drive_trajectory_planner
{
/** Configured physical limits. Nothing learned online may raise them. */
struct RobotLimits
{
  double max_v = 1.0;               ///< [m/s]
  double max_omega = 2.0;           ///< [rad/s]
  double max_accel = 2.0;           ///< [m/s^2]
  double max_brake = 2.0;           ///< [m/s^2], positive
  double max_angular_accel = 5.0;   ///< [rad/s^2]
  double wheel_radius = 0.05;       ///< [m]
  double wheel_separation = 0.3;    ///< [m]
  double max_wheel_speed = 30.0;    ///< [rad/s]

  /** Empty when valid, otherwise the first violated constraint. */
  std::string validate() const;
};

/** Actuator response the planner predicts with; the online estimator adjusts these within its own bounds. */
struct DynamicsParams
{
  double linear_time_constant = 0.0;   ///< First-order lag of v [s]; 0 tracks within the accel limits only.
  double angular_time_constant = 0.0;  ///< First-order lag of omega [s].
  double accel_scale = 1.0;            ///< Achieved fraction of max_accel; capped at 1.
  double brake_scale = 1.0;            ///< Achieved fraction of max_brake; capped at 1.
  double angular_accel_scale = 1.0;    ///< Achieved fraction of max_angular_accel; capped at 1.
  double delay = 0.0;                  ///< Command-to-motion delay [s].
  double left_wheel_scale = 1.0;       ///< Achieved / commanded left wheel speed.
  double right_wheel_scale = 1.0;      ///< Achieved / commanded right wheel speed.
  double angular_bias = 0.0;           ///< Constant yaw-rate offset [rad/s].

  std::string validate() const;
};

/** Immutable model a planning cycle predicts with; version identifies it in diagnostics. */
struct ModelSnapshot
{
  DynamicsParams params;
  uint64_t version = 0;
};

using ModelSnapshotConstPtr = std::shared_ptr<const ModelSnapshot>;

/** Hands validated snapshots from the estimator to the planner without either side blocking. */
class ModelSnapshotStore
{
public:
  explicit ModelSnapshotStore(ModelSnapshotConstPtr initial) : current_(std::move(initial))
  {
  }
  void publish(ModelSnapshotConstPtr snapshot)
  {
    current_.store(std::move(snapshot));
  }
  ModelSnapshotConstPtr current() const
  {
    return current_.load();
  }

private:
  std::atomic<ModelSnapshotConstPtr> current_;
};

/**
 * Differential-drive forward simulator: wheel-speed saturation, wheel scales and bias, first-order lag clipped by
 * the acceleration limits, command delay and exact constant-twist pose integration. Translation never reverses.
 */
class DynamicsModel
{
public:
  DynamicsModel(const RobotLimits& limits, const DynamicsParams& params);

  /** Clips a command to the velocity and wheel-speed limits, keeping its curvature; v below 0 becomes 0. */
  Command saturate(const Command& command) const;

  /** Body velocity the drive settles at for a saturated command. */
  Command steadyState(const Command& command) const;

  /** Advances the state by dt under the command acting on the wheels now (delay already applied). */
  void step(RobotState& state, const Command& acting, double dt) const;

  /**
   * Simulates duration seconds from start, appending one state per dt; duration must be a multiple of dt.
   * For the first pending_time seconds the pending command acts, then command, which models the delay.
   */
  void rollout(const RobotState& start, const Command& pending, double pending_time, const Command& command,
               double duration, double dt, std::vector<RobotState>& out) const;

  /** Pose after moving with constant twist (v, omega) for dt. */
  static Pose2D integrate(const Pose2D& pose, double v, double omega, double dt);

  double effectiveAccel() const;
  double effectiveBrake() const;
  double effectiveAngularAccel() const;

  const RobotLimits& limits() const
  {
    return limits_;
  }
  const DynamicsParams& params() const
  {
    return params_;
  }

private:
  RobotLimits limits_;
  DynamicsParams params_;
};

}  // namespace nice_differential_drive_trajectory_planner
