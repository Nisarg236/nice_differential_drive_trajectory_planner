#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "nice_differential_drive_trajectory_planner/dynamics_model.hpp"
#include "nice_differential_drive_trajectory_planner/route.hpp"
#include "nice_differential_drive_trajectory_planner/types.hpp"

namespace nice_differential_drive_trajectory_planner
{
/** Strategic choice a primitive represents; the search keeps alternatives of every class alive. */
enum class Maneuver : uint8_t
{
  Follow,
  PassLeft,
  PassRight,
  Slow,
  Stop,
  Rotate,
};
constexpr size_t kManeuverCount = 6;

const char* toString(Maneuver maneuver);

struct Primitive
{
  Command command;
  Maneuver maneuver = Maneuver::Follow;
};

struct PrimitiveParams
{
  double duration = 0.4;        ///< [s]
  double integration_dt = 0.05; ///< [s]; duration must be a multiple of it.
  std::vector<double> lateral_offsets{ 0.0, 0.3, -0.3, 0.6, -0.6 };  ///< Route offsets to track, positive left [m].
  int speed_levels = 2;           ///< Extra speed targets between braking and the fastest reachable speed.
  int angular_levels = 1;         ///< Fixed yaw-rate samples per side.
  int dense_angular_levels = 3;   ///< Fixed yaw-rate samples per side near obstacles.
  double lookahead_time = 1.0;    ///< Offset tracking lookahead per unit speed [s].
  double min_lookahead = 0.6;     ///< [m]
  double rotate_heading_threshold = 0.8;  ///< Heading error above which rotation in place is offered [rad].
  double rotate_speed = 1.5;              ///< [rad/s]
  size_t max_successors = 24;

  std::string validate() const;
};

/**
 * Builds a bounded, deterministic set of controls from a search node: stop, rotate in place when badly misaligned,
 * pure-pursuit tracking of laterally offset copies of the route at several reachable speeds, and fixed yaw rates.
 * Commands go through the dynamics model, so only reachable motion is ever predicted.
 */
class MotionPrimitiveGenerator
{
public:
  explicit MotionPrimitiveGenerator(const PrimitiveParams& params);

  /**
   * @param[in]  state  Node state.
   * @param[in]  route  Route the offsets refer to.
   * @param[in]  s      Node route progress [m].
   * @param[in]  dense  Sample more finely (near obstacles or tight turns).
   * @param[in]  model  Gives the reachable speeds over one primitive.
   * @param[out] out    Cleared, then filled most important first.
   */
  void generate(const RobotState& state, const Route& route, double s, bool dense, const DynamicsModel& model,
                std::vector<Primitive>& out) const;

  /** Point on the route shifted by offset that offset tracking at speed steers towards. */
  Point2D trackingTarget(const Route& route, double s, double offset, double speed) const;

  /** Yaw rate that steers towards the route shifted by offset, at speed. */
  double trackingYawRate(const RobotState& state, const Route& route, double s, double offset, double speed) const;

  const PrimitiveParams& params() const
  {
    return params_;
  }

private:
  PrimitiveParams params_;
};

}  // namespace nice_differential_drive_trajectory_planner
