#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "nice_differential_drive_trajectory_planner/collision_checker.hpp"
#include "nice_differential_drive_trajectory_planner/dynamics_model.hpp"
#include "nice_differential_drive_trajectory_planner/motion_primitives.hpp"
#include "nice_differential_drive_trajectory_planner/route.hpp"
#include "nice_differential_drive_trajectory_planner/trajectory_scorer.hpp"

namespace nice_differential_drive_trajectory_planner
{
struct SearchParams
{
  size_t frontier_size = 30;
  double horizon_time = 2.4;                ///< [s]
  double horizon_distance = 3.0;            ///< Route progress that completes a candidate [m].
  size_t max_nodes = 3000;                  ///< Simulated primitives kept per cycle.
  size_t max_primitive_evaluations = 3000;
  double time_budget = 0.035;               ///< Wall-clock search budget per cycle [s].
  double projection_margin = 0.3;           ///< Route window around a state's expected progress [m].
  double progress_bin = 0.15;               ///< Deduplication grid [m].
  double lateral_bin = 0.1;                 ///< [m]
  double heading_bin = 0.15;                ///< [rad]
  double speed_bin = 0.15;                  ///< [m/s]
  size_t min_per_maneuver = 2;              ///< Frontier slots reserved for each maneuver class.
  size_t max_candidates = 8;                ///< Finished candidates handed to the safety validator.
  double dense_clearance = 0.6;             ///< Dynamic clearance below which sampling densifies [m].
  double dense_static_cost = 100.0;         ///< Center cost above which sampling densifies.
  double dense_curvature = 1.0;             ///< Route curvature above which sampling densifies [1/m].

  std::string validate(const PrimitiveParams& primitives) const;
};

struct SearchInput
{
  RobotState state;
  Command pending;       ///< Last command sent; acts for the model delay.
  double s0 = 0.0;       ///< Robot route progress [m].
  double lateral0 = 0.0;
  const Route* route = nullptr;
  const DynamicsModel* model = nullptr;
  const CollisionChecker* checker = nullptr;
  bool has_previous_maneuver = false;
  Maneuver previous_maneuver = Maneuver::Follow;
};

/** A finished, fully evaluated control sequence. */
struct Candidate
{
  RobotState start;
  std::vector<RobotState> states;  ///< Simulated states after start, integration_dt apart.
  std::vector<Command> commands;   ///< One per primitive.
  Maneuver maneuver = Maneuver::Follow;
  CostVector cost{};
  double total = 0.0;
  double progress = 0.0;  ///< In-corridor route progress [m].
  double duration = 0.0;  ///< [s]
  bool reached_horizon = false;
  double min_dynamic_clearance = 0.0;
  double max_static_cost = 0.0;
};

struct SearchStats
{
  size_t nodes = 0;
  size_t evaluations = 0;
  size_t rejected_collision = 0;
  size_t rejected_route = 0;
  size_t rejected_dynamics = 0;
  size_t rejected_terminal = 0;
  size_t max_frontier = 0;
  size_t depth = 0;
  double horizon_reached = 0.0;  ///< Longest candidate duration [s].
  bool budget_exhausted = false;
  double elapsed = 0.0;  ///< [s]
};

/** One simulated primitive of the last search, for visualisation. */
struct TreePrimitive
{
  enum class Outcome : uint8_t
  {
    Kept,
    Collision,
    OffRoute,
    Infeasible,
    CannotStop,  ///< Kept, but braking from its end would hit something, so it was never a candidate.
  };
  Outcome outcome = Outcome::Kept;
  bool in_frontier = false;         ///< Survived deduplication and diversity pruning and was expanded further.
  bool complete = false;        ///< Ended a sequence: horizon, progress target, stop or route end.
  bool has_collision = false;
  Pose2D collision;             ///< First simulated pose at or past the contact.
  std::vector<RobotState> states;  ///< The state it started from, then every simulated state.
};

/**
 * Route-guided multi-stage tree search. Each depth expands every frontier node with motion primitives simulated from
 * its physical end state, rejects colliding and out-of-corridor ones, scores the rest, merges near-duplicates and keeps
 * the best per maneuver class before filling the frontier by cost. Progress counts only while inside the preferred
 * corridor, but children always continue from the true simulated state.
 */
class TrajectorySearch
{
public:
  TrajectorySearch(const SearchParams& params, const PrimitiveParams& primitives, const ScoringParams& scoring,
                   const RobotLimits& limits);

  /** Fills out with up to max_candidates fully evaluated candidates, best first. */
  void plan(const SearchInput& input, std::vector<Candidate>& out, SearchStats& stats);

  const SearchParams& params() const
  {
    return params_;
  }
  const TrajectoryScorer& scorer() const
  {
    return scorer_;
  }
  const MotionPrimitiveGenerator& generator() const
  {
    return generator_;
  }

  /** Also keep rejected primitives for treePrimitives(); allocates per rejection, so only for visualisation. */
  void setRecordTree(bool record)
  {
    record_tree_ = record;
  }

  /** Up to max primitives of the last search: kept ones in expansion order, then rejected ones if recorded. */
  void treePrimitives(std::vector<TreePrimitive>& out, size_t max) const;

private:
  struct Node
  {
    int32_t parent = -1;
    uint32_t states_begin = 0;
    uint32_t states_count = 0;
    RobotState state;
    double time = 0.0;
    double s = 0.0;
    double lateral = 0.0;
    double progress = 0.0;
    Command command;
    Maneuver maneuver = Maneuver::Follow;
    CostVector cost{};
    double rank = 0.0;
    double min_dynamic_clearance = 0.0;
    double max_static_cost = 0.0;
    bool complete = false;
    bool stopped = false;  ///< Ended at rest: a stop, or the route end.
    bool in_frontier = false;
    bool cannot_stop = false;
  };

  /** Simulates and scores one primitive from parent; returns false when rejected. */
  bool expand(const SearchInput& input, uint32_t parent_index, const Primitive& primitive, Node& child,
              SearchStats& stats);
  void selectFrontier(std::vector<uint32_t>& children, std::vector<uint32_t>& frontier);
  bool finalize(const SearchInput& input, Node& node, SearchStats& stats);
  void reconstruct(uint32_t index, Candidate& candidate) const;
  double rankOf(const Node& node) const;
  void recordRejected(const RobotState& start, TreePrimitive::Outcome outcome, const Pose2D* collision = nullptr);

  SearchParams params_;
  MotionPrimitiveGenerator generator_;
  TrajectoryScorer scorer_;
  RobotLimits limits_;
  double target_progress_ = 0.0;
  bool record_tree_ = false;
  std::vector<TreePrimitive> rejected_;

  std::vector<Node> nodes_;
  std::vector<RobotState> state_pool_;
  std::vector<RobotState> rollout_;
  std::vector<double> s_buffer_;
  std::vector<double> lateral_buffer_;
  std::vector<Primitive> primitives_;
  std::vector<uint32_t> frontier_;
  std::vector<uint32_t> children_;
  std::vector<uint32_t> finished_;
  std::vector<uint32_t> scratch_;
};

}  // namespace nice_differential_drive_trajectory_planner
