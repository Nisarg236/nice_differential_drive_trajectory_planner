#pragma once

#include <array>
#include <cstddef>
#include <string>

#include "nice_differential_drive_trajectory_planner/collision_checker.hpp"
#include "nice_differential_drive_trajectory_planner/dynamics_model.hpp"
#include "nice_differential_drive_trajectory_planner/route.hpp"

namespace nice_differential_drive_trajectory_planner
{
/** Every term is dimensionless and time-averaged over the horizon, so a weight of 1 means the same for each. */
enum CostTerm : size_t
{
  kProgress,      ///< Shortfall of in-corridor route progress against the horizon target.
  kRoute,         ///< (d / filter_dist)^2 inside the preferred corridor, a barrier towards max_route_deviation.
  kVelocity,      ///< ((v - v_ref(s)) / max_v)^2, penalising both overspeed and needless slowing.
  kAcceleration,  ///< (a / max_accel)^2.
  kAngular,       ///< ((omega - v * curvature(s)) / max_omega)^2 plus a penalty per yaw-rate sign flip.
  kObstacle,      ///< (center costmap cost / 252)^2 plus exp(-dynamic clearance / obstacle_distance_scale).
  kTerminal,      ///< (terminal heading error / pi)^2, plus the unexplored fraction of a sequence the budget cut short.
  kCommitment,    ///< 1 when the first maneuver differs from the one executed last cycle.
  kCostTermCount
};

const char* toString(CostTerm term);

using CostVector = std::array<double, kCostTermCount>;

struct ScoringParams
{
  CostVector weights{ 4.0, 1.0, 2.0, 0.2, 0.5, 2.0, 1.0, 0.3 };
  std::array<bool, kCostTermCount> enabled{ true, true, true, true, true, true, true, true };  ///< Ablation.
  double filter_dist = 0.3;               ///< Preferred corridor half-width [m].
  double max_route_deviation = 1.0;       ///< Outer corridor half-width, a hard limit [m].
  double obstacle_distance_scale = 0.5;   ///< [m]
  double yaw_flip_penalty = 0.1;          ///< Added to kAngular per sign flip of the commanded yaw rate.
  double yaw_flip_threshold = 0.1;        ///< Yaw rates below this never count as flips [rad/s].

  std::string validate() const;
};

/** Running and terminal cost terms of one candidate. */
class TrajectoryScorer
{
public:
  TrajectoryScorer(const ScoringParams& params, const RobotLimits& limits, double horizon_time);

  /**
   * Adds the running terms of one primitive.
   * @param[in]     before          State the primitive starts from.
   * @param[in]     states          count simulated states, dt apart.
   * @param[in]     s, lateral      Route projection of every state.
   * @param[in]     parent_command  Command of the previous primitive, for yaw-rate flips.
   * @param[in]     command         Command of this primitive.
   * @param[in]     sweep           Collision sweep of the primitive.
   * @param[in,out] cost            Accumulated terms.
   */
  void addSegment(const RobotState& before, const RobotState* states, const double* s, const double* lateral,
                  size_t count, double dt, const Route& route, const Command& parent_command, const Command& command,
                  const SweepResult& sweep, CostVector& cost) const;

  /** kProgress for in-corridor progress against target progress [m]; 0 when there is nothing left to cover. */
  double progressShortfall(double progress, double target) const;

  /**
   * Running terms for staying at final_state for duration seconds. A sequence that stops before the horizon is charged
   * this for the rest of it, so stopping early is never cheaper than an equally slow sequence that keeps going.
   */
  void addHold(const RobotState& final_state, double s, double lateral, double static_cost, double duration,
               const Route& route, CostVector& cost) const;

  /** kTerminal for a candidate ending at time; cut_short marks a sequence the search stopped expanding early. */
  double terminalCost(const RobotState& final_state, double time, double s, const Route& route, bool cut_short) const;

  double routeCost(double distance) const;

  /** Weighted sum over the enabled terms. */
  double total(const CostVector& cost) const;

  const ScoringParams& params() const
  {
    return params_;
  }

private:
  ScoringParams params_;
  RobotLimits limits_;
  double horizon_time_;
};

}  // namespace nice_differential_drive_trajectory_planner
