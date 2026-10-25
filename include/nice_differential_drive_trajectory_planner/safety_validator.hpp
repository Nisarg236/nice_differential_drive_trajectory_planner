#pragma once

#include <string>
#include <vector>

#include "nice_differential_drive_trajectory_planner/collision_checker.hpp"
#include "nice_differential_drive_trajectory_planner/dynamics_model.hpp"
#include "nice_differential_drive_trajectory_planner/trajectory_search.hpp"

namespace nice_differential_drive_trajectory_planner
{
struct SafetyParams
{
  CollisionCheckParams collision{ 0.02, 0.1, true };
  double brake_factor = 0.8;             ///< Fraction of max_brake assumed when checking that the robot can stop.
  double min_delay = 0.0;                ///< Delay assumed when checking stops, if the model's is smaller [s].
  double control_period = 1.0 / 15.0;   ///< The first command acts at least this long before a new one [s].
  double limit_tolerance = 1.05;         ///< Factor on velocity and acceleration limits for numerical slack.
  double start_position_tolerance = 0.02;  ///< [m]
  double start_heading_tolerance = 0.02;   ///< [rad]
  double max_state_age = 0.3;     ///< [s]
  double max_costmap_age = 0.5;   ///< [s]
  double max_obstacle_age = 0.5;  ///< [s]; only checked when obstacle tracks are required.
  bool require_obstacle_tracks = false;

  std::string validate() const;
};

enum class SafetyVerdict
{
  Ok,
  StaleInput,
  Discontinuous,
  Reverse,
  VelocityLimit,
  AccelerationLimit,
  WheelLimit,
  StaticCollision,
  DynamicCollision,
  CannotStop,
};

const char* toString(SafetyVerdict verdict);

struct InputAges
{
  double state = 0.0;     ///< [s]
  double costmap = 0.0;   ///< [s]
  double obstacles = 0.0; ///< [s]
};

/**
 * Last gate before a candidate is executed, independent of the search and its weights: configured limits only,
 * its own collision resolution and margin, and a braking model at most as strong as configured.
 */
class SafetyValidator
{
public:
  SafetyValidator(const RobotLimits& limits, const Footprint& footprint, const SafetyParams& params);

  void setCostmap(const nav2_costmap_2d::Costmap2D* costmap, unsigned char circumscribed_cost);
  void setObstacles(const ObstaclePredictionModel* obstacles);

  bool inputsFresh(const InputAges& ages) const;

  /** Checks a candidate against the current state; learned parameters only ever make the stop check stricter. */
  SafetyVerdict validate(const Candidate& candidate, const RobotState& current, const DynamicsParams& learned,
                         double integration_dt) const;

  /** Whether braking now, after first acting for the control period, stops without contact. */
  bool canStopAfter(const RobotState& current, const Command& first, const DynamicsParams& learned,
                    double integration_dt) const;

  /** States of the stop check behind canStopAfter, for visualisation. */
  void stoppingStates(const RobotState& current, const Command& first, const DynamicsParams& learned,
                      double integration_dt, std::vector<RobotState>& out) const;

  /** Strongest configured braking from current over one control period, yaw rate ramped down too. */
  Command brakingCommand(const RobotState& current) const;

  bool currentPoseInCollision(const RobotState& current) const;

  const SafetyParams& params() const
  {
    return params_;
  }

private:
  DynamicsModel stopModel(const DynamicsParams& learned) const;

  RobotLimits limits_;
  SafetyParams params_;
  CollisionChecker checker_;
};

}  // namespace nice_differential_drive_trajectory_planner
