#pragma once

#include <memory>
#include <string>
#include <vector>

#include "nice_differential_drive_trajectory_planner/collision_checker.hpp"
#include "nice_differential_drive_trajectory_planner/dynamics_model.hpp"
#include "nice_differential_drive_trajectory_planner/motion_primitives.hpp"
#include "nice_differential_drive_trajectory_planner/obstacle_prediction.hpp"
#include "nice_differential_drive_trajectory_planner/route.hpp"
#include "nice_differential_drive_trajectory_planner/safety_validator.hpp"
#include "nice_differential_drive_trajectory_planner/trajectory_scorer.hpp"
#include "nice_differential_drive_trajectory_planner/trajectory_search.hpp"

namespace nice_differential_drive_trajectory_planner
{
struct ControllerParams
{
  RobotLimits limits;
  DynamicsParams nominal_dynamics;
  SpeedProfileParams speed_profile;
  PrimitiveParams primitives;
  SearchParams search;
  ScoringParams scoring;
  CollisionCheckParams collision;
  SafetyParams safety;
  ObstaclePredictionParams obstacles;
  Footprint footprint;
  double route_start_window = 1.0;  ///< Route length searched for the robot's projection [m].
  double goal_yaw_gain = 2.0;       ///< Yaw rate per heading error while turning to the goal heading [1/s].
  double goal_yaw_tolerance = 0.05; ///< [rad]
  int max_fallback_cycles = 15;     ///< Consecutive cycles without a valid candidate before giving up.

  /** Empty when valid, otherwise the first problem found. */
  std::string validate() const;
};

enum class ControlStatus
{
  Tracking,
  TurningToGoal,
  GoalReached,
  Fallback,
  NoValidControl,
  InvalidInput,
};

const char* toString(ControlStatus status);

struct ControlInput
{
  double now = 0.0;  ///< [s]
  RobotState state;  ///< In the costmap frame.
  InputAges ages;
  std::vector<Point2D> route;     ///< Pruned route in the costmap frame, starting near the robot.
  std::vector<double> speed_caps; ///< Optional per-point speed limits [m/s].
  bool route_ends_at_goal = true;
  bool goal_position_reached = false;  ///< Position part of the goal check holds; turn to goal_heading.
  double goal_heading = 0.0;           ///< [rad]
  double speed_limit = 0.0;            ///< External cap [m/s]; 0 or below disables it.
  const nav2_costmap_2d::Costmap2D* costmap = nullptr;
  unsigned char circumscribed_cost = 0;
  const std::vector<ObstacleTrack>* tracks = nullptr;
};

struct ControlOutput
{
  Command command;
  ControlStatus status = ControlStatus::Fallback;
  std::string reason;
  SearchStats stats;
  bool has_selection = false;
  Candidate selected;
  size_t candidates = 0;
  size_t candidates_rejected = 0;
  SafetyVerdict first_rejection = SafetyVerdict::Ok;
  uint64_t model_version = 0;
  double route_progress = 0.0;
  double route_deviation = 0.0;
  double reference_speed = 0.0;
  bool goal_position_reached = false;
  double goal_heading = 0.0;  ///< [rad]
};

/**
 * One control cycle without ROS: builds the route, runs the search against the current model snapshot, executes the
 * best candidate that passes the safety validator, and otherwise brakes within the configured limits.
 */
class ControllerCore
{
public:
  explicit ControllerCore(const ControllerParams& params);

  void reset();
  ControlOutput compute(const ControlInput& input);

  /** Planner and estimator exchange model snapshots through this store. */
  ModelSnapshotStore& snapshots()
  {
    return snapshots_;
  }
  const ControllerParams& params() const
  {
    return params_;
  }
  const Command& lastCommand() const
  {
    return last_command_;
  }

  /** Candidates of the last cycle, best first; the first verdicts().size() of them went through validation. */
  const std::vector<Candidate>& candidates() const
  {
    return candidates_;
  }
  const std::vector<SafetyVerdict>& verdicts() const
  {
    return verdicts_;
  }
  const Route& route() const
  {
    return route_;
  }
  const ObstaclePredictionModel& obstacles() const
  {
    return obstacles_;
  }
  const TrajectorySearch& search() const
  {
    return search_;
  }
  void setRecordTree(bool record)
  {
    search_.setRecordTree(record);
  }
  const SafetyValidator& validator() const
  {
    return validator_;
  }

private:
  ControlOutput fallback(ControlOutput output, const RobotState& state, const std::string& reason);
  ControlOutput turnToGoal(ControlOutput output, const ControlInput& input, const DynamicsParams& learned);

  ControllerParams params_;
  ModelSnapshotStore snapshots_;
  Route route_;
  ObstaclePredictionModel obstacles_;
  CollisionChecker checker_;
  TrajectorySearch search_;
  SafetyValidator validator_;
  std::vector<Candidate> candidates_;
  std::vector<SafetyVerdict> verdicts_;
  Command last_command_;
  bool has_previous_maneuver_ = false;
  Maneuver previous_maneuver_ = Maneuver::Follow;
  int fallback_cycles_ = 0;
};

}  // namespace nice_differential_drive_trajectory_planner
