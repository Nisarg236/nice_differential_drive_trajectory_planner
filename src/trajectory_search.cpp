#include "nice_differential_drive_trajectory_planner/trajectory_search.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <tuple>

namespace nice_differential_drive_trajectory_planner
{
namespace
{
constexpr double kStoppedSpeed = 0.01;
constexpr double kRouteEndTolerance = 0.05;
constexpr double kTimeEpsilon = 1e-9;

using Clock = std::chrono::steady_clock;

bool finite(const RobotState& state)
{
  return std::isfinite(state.pose.x) && std::isfinite(state.pose.y) && std::isfinite(state.pose.theta) &&
         std::isfinite(state.v) && std::isfinite(state.omega);
}

int64_t bin(double value, double size)
{
  return static_cast<int64_t>(std::floor(value / size));
}
}  // namespace

std::string SearchParams::validate(const PrimitiveParams& primitives) const
{
  if (frontier_size == 0 || max_candidates == 0)
  {
    return "frontier_size and max_candidates must be positive";
  }
  if (!(horizon_time >= primitives.duration) || !(horizon_distance > 0.0))
  {
    return "horizon_time must cover at least one primitive and horizon_distance must be positive";
  }
  if (max_nodes == 0 || max_primitive_evaluations == 0 || !(time_budget > 0.0))
  {
    return "max_nodes, max_primitive_evaluations and time_budget must be positive";
  }
  if (!(projection_margin > 0.0))
  {
    return "projection_margin must be positive";
  }
  if (!(progress_bin > 0.0) || !(lateral_bin > 0.0) || !(heading_bin > 0.0) || !(speed_bin > 0.0))
  {
    return "deduplication bins must be positive";
  }
  return {};
}

TrajectorySearch::TrajectorySearch(const SearchParams& params, const PrimitiveParams& primitives,
                                   const ScoringParams& scoring, const RobotLimits& limits)
  : params_(params), generator_(primitives), scorer_(scoring, limits, params.horizon_time), limits_(limits)
{
  const size_t steps = static_cast<size_t>(std::lround(primitives.duration / primitives.integration_dt));
  nodes_.reserve(params_.max_nodes + 1);
  state_pool_.reserve((params_.max_nodes + 1) * steps);
  rollout_.reserve(steps);
  s_buffer_.resize(steps);
  lateral_buffer_.resize(steps);
  primitives_.reserve(primitives.max_successors);
  frontier_.reserve(params_.frontier_size);
  children_.reserve(params_.max_nodes);
  finished_.reserve(params_.max_nodes);
  scratch_.reserve(params_.max_nodes);
}

double TrajectorySearch::rankOf(const Node& node) const
{
  CostVector partial = node.cost;
  const double expected = target_progress_ * std::min(1.0, node.time / params_.horizon_time);
  partial[kProgress] = target_progress_ > 1e-3 ? std::max(0.0, expected - node.progress) / target_progress_ : 0.0;
  partial[kTerminal] = 0.0;
  return scorer_.total(partial);
}

bool TrajectorySearch::expand(const SearchInput& input, uint32_t parent_index, const Primitive& primitive,
                              Node& child, SearchStats& stats)
{
  const Node parent = nodes_[parent_index];
  const DynamicsModel& model = *input.model;
  const PrimitiveParams& primitive_params = generator_.params();
  const double dt = primitive_params.integration_dt;

  rollout_.clear();
  model.rollout(parent.state, parent.command, model.params().delay, primitive.command, primitive_params.duration, dt,
                rollout_);
  for (const auto& state : rollout_)
  {
    if (!finite(state) || state.v < 0.0)
    {
      ++stats.rejected_dynamics;
      recordRejected(parent.state, TreePrimitive::Outcome::Infeasible);
      return false;
    }
  }

  const SweepResult sweep = input.checker->sweep(parent.state.pose, parent.time, rollout_.data(), rollout_.size(), dt);
  if (sweep.collision())
  {
    ++stats.rejected_collision;
    const size_t contact = std::min(
        rollout_.size() - 1,
        static_cast<size_t>(std::max(0.0, std::ceil((sweep.collision_time - parent.time) / dt - 1e-9) - 1.0)));
    recordRejected(parent.state, TreePrimitive::Outcome::Collision, &rollout_[contact].pose);
    return false;
  }

  const ScoringParams& scoring = scorer_.params();
  double previous_s = parent.s;
  Point2D previous_position = position(parent.state.pose);
  for (size_t k = 0; k < rollout_.size(); ++k)
  {
    const Point2D p = position(rollout_[k].pose);
    const double step = distance(previous_position, p);
    const RouteProjection projection = input.route->project(p, previous_s - params_.projection_margin,
                                                            previous_s + step + params_.projection_margin);
    if (!projection.valid || projection.distance > scoring.max_route_deviation)
    {
      ++stats.rejected_route;
      recordRejected(parent.state, TreePrimitive::Outcome::OffRoute);
      return false;
    }
    s_buffer_[k] = projection.s;
    lateral_buffer_[k] = projection.lateral;
    previous_s = projection.s;
    previous_position = p;
  }

  double valid_end = parent.s;
  if (std::abs(parent.lateral) <= scoring.filter_dist)
  {
    for (size_t k = 0; k < rollout_.size() && std::abs(lateral_buffer_[k]) <= scoring.filter_dist; ++k)
    {
      valid_end = std::max(valid_end, s_buffer_[k]);
    }
  }

  child = Node{};
  child.parent = static_cast<int32_t>(parent_index);
  child.cost = parent.cost;
  scorer_.addSegment(parent.state, rollout_.data(), s_buffer_.data(), lateral_buffer_.data(), rollout_.size(), dt,
                     *input.route, parent.command, primitive.command, sweep, child.cost);
  child.maneuver = parent_index == 0 ? primitive.maneuver : parent.maneuver;
  if (parent_index == 0)
  {
    child.cost[kCommitment] =
        input.has_previous_maneuver && primitive.maneuver != input.previous_maneuver ? 1.0 : 0.0;
  }

  child.states_begin = static_cast<uint32_t>(state_pool_.size());
  child.states_count = static_cast<uint32_t>(rollout_.size());
  state_pool_.insert(state_pool_.end(), rollout_.begin(), rollout_.end());
  child.state = rollout_.back();
  child.time = parent.time + primitive_params.duration;
  child.s = s_buffer_[rollout_.size() - 1];
  child.lateral = lateral_buffer_[rollout_.size() - 1];
  child.progress = parent.progress + std::max(0.0, valid_end - parent.s);
  child.command = primitive.command;
  child.min_dynamic_clearance = std::min(parent.min_dynamic_clearance, sweep.min_dynamic_clearance);
  child.max_static_cost = sweep.max_static_cost;

  const bool stopped = child.state.v < kStoppedSpeed && primitive.command.v == 0.0 && primitive.command.omega == 0.0 &&
                       std::abs(child.state.omega) < kStoppedSpeed;
  const bool at_route_end = child.s >= input.route->length() - kRouteEndTolerance && child.state.v < kStoppedSpeed;
  child.stopped = stopped || at_route_end;
  child.complete = child.time >= params_.horizon_time - kTimeEpsilon ||
                   child.s - input.s0 >= params_.horizon_distance || child.stopped;
  child.rank = rankOf(child);
  return true;
}

void TrajectorySearch::selectFrontier(std::vector<uint32_t>& children, std::vector<uint32_t>& frontier)
{
  using Key = std::tuple<int64_t, int64_t, int64_t, int64_t, uint8_t>;
  auto key = [this](uint32_t index) {
    const Node& node = nodes_[index];
    return Key{ bin(node.s, params_.progress_bin), bin(node.lateral, params_.lateral_bin),
                bin(node.state.pose.theta, params_.heading_bin), bin(node.state.v, params_.speed_bin),
                static_cast<uint8_t>(node.maneuver) };
  };
  auto by_rank = [this](uint32_t a, uint32_t b) {
    return std::tie(nodes_[a].rank, a) < std::tie(nodes_[b].rank, b);
  };

  std::sort(children.begin(), children.end(), [&](uint32_t a, uint32_t b) {
    const Key key_a = key(a);
    const Key key_b = key(b);
    return key_a != key_b ? key_a < key_b : by_rank(a, b);
  });
  scratch_.clear();
  for (size_t i = 0; i < children.size(); ++i)
  {
    if (i == 0 || key(children[i]) != key(children[i - 1]))
    {
      scratch_.push_back(children[i]);
    }
  }
  std::sort(scratch_.begin(), scratch_.end(), by_rank);

  frontier.clear();
  std::array<size_t, kManeuverCount> per_maneuver{};
  for (uint32_t index : scratch_)
  {
    const size_t maneuver = static_cast<size_t>(nodes_[index].maneuver);
    if (frontier.size() < params_.frontier_size && per_maneuver[maneuver] < params_.min_per_maneuver)
    {
      ++per_maneuver[maneuver];
      frontier.push_back(index);
    }
  }
  for (uint32_t index : scratch_)
  {
    if (frontier.size() >= params_.frontier_size)
    {
      break;
    }
    if (std::find(frontier.begin(), frontier.end(), index) == frontier.end())
    {
      frontier.push_back(index);
    }
  }
  std::sort(frontier.begin(), frontier.end(), by_rank);
  for (uint32_t index : frontier)
  {
    nodes_[index].in_frontier = true;
  }
}

bool TrajectorySearch::finalize(const SearchInput& input, Node& node, SearchStats& stats)
{
  node.cost[kProgress] = scorer_.progressShortfall(node.progress, target_progress_);
  const double remaining = params_.horizon_time - node.time;
  if (node.stopped && remaining > kTimeEpsilon)
  {
    scorer_.addHold(node.state, node.s, node.lateral, node.max_static_cost, remaining, *input.route, node.cost);
  }
  node.cost[kTerminal] = scorer_.terminalCost(node.state, node.time, node.s, *input.route, !node.complete);

  if (node.state.v > kStoppedSpeed)
  {
    const DynamicsModel& model = *input.model;
    const double dt = generator_.params().integration_dt;
    const double delay = model.params().delay;
    const double brake_time = delay + node.state.v / model.effectiveBrake();
    const double duration = dt * std::ceil(brake_time / dt + 1.0);
    rollout_.clear();
    model.rollout(node.state, node.command, delay, Command{}, duration, dt, rollout_);
    if (input.checker->sweep(node.state.pose, node.time, rollout_.data(), rollout_.size(), dt).collision())
    {
      ++stats.rejected_terminal;
      node.cannot_stop = true;
      return false;
    }
  }
  node.rank = scorer_.total(node.cost);
  return true;
}

void TrajectorySearch::reconstruct(uint32_t index, Candidate& candidate) const
{
  const Node& last = nodes_[index];
  candidate.states.clear();
  candidate.commands.clear();
  candidate.maneuver = last.maneuver;
  candidate.cost = last.cost;
  candidate.total = last.rank;
  candidate.progress = last.progress;
  candidate.duration = last.time;
  candidate.reached_horizon = last.time >= params_.horizon_time - kTimeEpsilon;
  candidate.min_dynamic_clearance = last.min_dynamic_clearance;
  candidate.max_static_cost = last.max_static_cost;

  std::vector<uint32_t> chain;
  for (int32_t i = static_cast<int32_t>(index); i > 0; i = nodes_[i].parent)
  {
    chain.push_back(static_cast<uint32_t>(i));
  }
  candidate.start = nodes_[0].state;
  for (auto it = chain.rbegin(); it != chain.rend(); ++it)
  {
    const Node& node = nodes_[*it];
    candidate.commands.push_back(node.command);
    candidate.states.insert(candidate.states.end(), state_pool_.begin() + node.states_begin,
                            state_pool_.begin() + node.states_begin + node.states_count);
  }
}

void TrajectorySearch::plan(const SearchInput& input, std::vector<Candidate>& out, SearchStats& stats)
{
  const auto started = Clock::now();
  stats = SearchStats{};
  out.clear();
  nodes_.clear();
  state_pool_.clear();
  finished_.clear();
  children_.clear();
  frontier_.clear();
  rejected_.clear();

  target_progress_ = std::clamp(input.route->length() - input.s0, 0.0, params_.horizon_distance);

  Node root;
  root.state = input.state;
  root.s = input.s0;
  root.lateral = input.lateral0;
  root.command = input.pending;
  root.in_frontier = true;
  root.min_dynamic_clearance = input.checker->dynamicClearance(input.state.pose, 0.0);
  input.checker->staticCollision(input.state.pose, &root.max_static_cost);
  nodes_.push_back(root);
  frontier_.push_back(0);

  const size_t max_depth =
      static_cast<size_t>(std::ceil(params_.horizon_time / generator_.params().duration - kTimeEpsilon));
  bool exhausted = false;
  for (size_t depth = 0; depth < max_depth && !exhausted && !frontier_.empty(); ++depth)
  {
    children_.clear();
    for (uint32_t parent_index : frontier_)
    {
      const Node& parent = nodes_[parent_index];
      const bool dense = parent.min_dynamic_clearance < params_.dense_clearance ||
                         parent.max_static_cost > params_.dense_static_cost ||
                         std::abs(input.route->sample(parent.s).curvature) > params_.dense_curvature;
      generator_.generate(parent.state, *input.route, parent.s, dense, *input.model, primitives_);
      for (const Primitive& primitive : primitives_)
      {
        const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
        if (stats.evaluations >= params_.max_primitive_evaluations || nodes_.size() > params_.max_nodes ||
            elapsed > params_.time_budget)
        {
          exhausted = true;
          break;
        }
        ++stats.evaluations;
        Node child;
        if (!expand(input, parent_index, primitive, child, stats))
        {
          continue;
        }
        const uint32_t child_index = static_cast<uint32_t>(nodes_.size());
        nodes_.push_back(child);
        (child.complete ? finished_ : children_).push_back(child_index);
      }
      if (exhausted)
      {
        break;
      }
    }
    stats.depth = depth + 1;
    if (!exhausted)
    {
      selectFrontier(children_, frontier_);
      stats.max_frontier = std::max(stats.max_frontier, frontier_.size());
    }
  }
  stats.budget_exhausted = exhausted;

  scratch_ = finished_;
  if (exhausted || scratch_.size() < params_.max_candidates)
  {
    for (const auto* pool : { &frontier_, &children_ })
    {
      for (uint32_t index : *pool)
      {
        if (index != 0 && std::find(scratch_.begin(), scratch_.end(), index) == scratch_.end())
        {
          scratch_.push_back(index);
        }
      }
    }
  }

  std::vector<uint32_t>& accepted = children_;
  accepted.clear();
  for (uint32_t index : scratch_)
  {
    if (finalize(input, nodes_[index], stats))
    {
      accepted.push_back(index);
    }
  }
  std::sort(accepted.begin(), accepted.end(), [this](uint32_t a, uint32_t b) {
    return std::tie(nodes_[a].rank, a) < std::tie(nodes_[b].rank, b);
  });

  const size_t count = std::min(params_.max_candidates, accepted.size());
  out.resize(count);
  for (size_t i = 0; i < count; ++i)
  {
    reconstruct(accepted[i], out[i]);
    stats.horizon_reached = std::max(stats.horizon_reached, out[i].duration);
  }
  stats.nodes = nodes_.size() - 1;
  stats.elapsed = std::chrono::duration<double>(Clock::now() - started).count();
}

void TrajectorySearch::recordRejected(const RobotState& start, TreePrimitive::Outcome outcome,
                                      const Pose2D* collision)
{
  if (!record_tree_)
  {
    return;
  }
  TreePrimitive& primitive = rejected_.emplace_back();
  primitive.outcome = outcome;
  if (collision != nullptr)
  {
    primitive.has_collision = true;
    primitive.collision = *collision;
  }
  primitive.states.reserve(rollout_.size() + 1);
  primitive.states.push_back(start);
  primitive.states.insert(primitive.states.end(), rollout_.begin(), rollout_.end());
}

void TrajectorySearch::treePrimitives(std::vector<TreePrimitive>& out, size_t max) const
{
  out.clear();
  for (size_t i = 1; i < nodes_.size() && out.size() < max; ++i)
  {
    const Node& node = nodes_[i];
    TreePrimitive& primitive = out.emplace_back();
    primitive.outcome = node.cannot_stop ? TreePrimitive::Outcome::CannotStop : TreePrimitive::Outcome::Kept;
    primitive.in_frontier = node.in_frontier;
    primitive.complete = node.complete;
    primitive.states.push_back(nodes_[node.parent].state);
    primitive.states.insert(primitive.states.end(), state_pool_.begin() + node.states_begin,
                            state_pool_.begin() + node.states_begin + node.states_count);
  }
  for (size_t i = 0; i < rejected_.size() && out.size() < max; ++i)
  {
    out.push_back(rejected_[i]);
  }
}

}  // namespace nice_differential_drive_trajectory_planner
