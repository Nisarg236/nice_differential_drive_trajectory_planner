#include "nice_differential_drive_trajectory_planner/visualization.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace nice_differential_drive_trajectory_planner
{
namespace
{
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using ColorRGBA = std_msgs::msg::ColorRGBA;

constexpr double kLineZ = 0.02;
constexpr double kRouteStep = 0.1;

ColorRGBA rgba(double r, double g, double b, double a)
{
  ColorRGBA color;
  color.r = static_cast<float>(r);
  color.g = static_cast<float>(g);
  color.b = static_cast<float>(b);
  color.a = static_cast<float>(a);
  return color;
}

/** Blue at 0 through green to red at 1. */
ColorRGBA speedColor(double fraction, double alpha = 1.0)
{
  const double t = std::clamp(fraction, 0.0, 1.0);
  if (t < 0.5)
  {
    return rgba(0.0, 2.0 * t, 1.0 - 2.0 * t, alpha);
  }
  return rgba(2.0 * t - 1.0, 2.0 - 2.0 * t, 0.0, alpha);
}

ColorRGBA outcomeColor(TreePrimitive::Outcome outcome)
{
  switch (outcome)
  {
    case TreePrimitive::Outcome::Collision:
      return rgba(1.0, 0.1, 0.1, 0.45);
    case TreePrimitive::Outcome::OffRoute:
      return rgba(1.0, 0.6, 0.0, 0.45);
    case TreePrimitive::Outcome::Infeasible:
      return rgba(0.9, 0.0, 0.9, 0.45);
    case TreePrimitive::Outcome::CannotStop:
      return rgba(0.55, 0.25, 0.95, 0.7);
    case TreePrimitive::Outcome::Kept:
      break;
  }
  return rgba(0.6, 0.6, 0.6, 0.25);
}

geometry_msgs::msg::Point point(double x, double y, double z = kLineZ)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

Marker marker(const std::string& frame, const rclcpp::Time& stamp, const std::string& ns, int id, int type,
              double width)
{
  Marker m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = width;
  m.scale.y = width;
  m.scale.z = width;
  return m;
}

Marker text(const std::string& frame, const rclcpp::Time& stamp, const std::string& ns, int id, double x, double y,
            double z, double size, const ColorRGBA& color, const std::string& words)
{
  Marker m = marker(frame, stamp, ns, id, Marker::TEXT_VIEW_FACING, size);
  m.pose.position = point(x, y, z);
  m.color = color;
  m.text = words;
  return m;
}

void addFootprint(Marker& lines, const Footprint& footprint, const Pose2D& pose)
{
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  for (size_t i = 0; i < footprint.size(); ++i)
  {
    const Point2D& a = footprint[i];
    const Point2D& b = footprint[(i + 1) % footprint.size()];
    lines.points.push_back(point(pose.x + c * a.x - s * a.y, pose.y + s * a.x + c * a.y));
    lines.points.push_back(point(pose.x + c * b.x - s * b.y, pose.y + s * b.x + c * b.y));
  }
}

Marker trajectory(const std::string& frame, const rclcpp::Time& stamp, const std::string& ns, int id,
                  const Candidate& candidate, double width, double max_v, double alpha)
{
  Marker line = marker(frame, stamp, ns, id, Marker::LINE_STRIP, width);
  line.points.push_back(point(candidate.start.pose.x, candidate.start.pose.y));
  line.colors.push_back(speedColor(candidate.start.v / max_v, alpha));
  for (const auto& state : candidate.states)
  {
    line.points.push_back(point(state.pose.x, state.pose.y));
    line.colors.push_back(speedColor(state.v / max_v, alpha));
  }
  return line;
}

/** Route shifted sideways by offset, positive left. */
Marker offsetRoute(const std::string& frame, const rclcpp::Time& stamp, int id, const Route& route, double offset,
                   const ColorRGBA& color)
{
  Marker line = marker(frame, stamp, "corridor", id, Marker::LINE_STRIP, 0.015);
  line.color = color;
  for (double s = 0.0; s <= route.length(); s += kRouteStep)
  {
    const RouteSample sample = route.sample(s);
    line.points.push_back(point(sample.position.x - offset * std::sin(sample.heading),
                                sample.position.y + offset * std::cos(sample.heading), 0.005));
  }
  return line;
}

/** One short line: the manoeuvre and commanded speed while planning, otherwise the controller status. */
std::string statusText(const ControlOutput& output)
{
  char buffer[64];
  if (output.status == ControlStatus::Tracking && output.has_selection)
  {
    std::snprintf(buffer, sizeof(buffer), "%s %.2f m/s", toString(output.selected.maneuver), output.command.v);
  }
  else
  {
    std::snprintf(buffer, sizeof(buffer), "%s", toString(output.status));
  }
  return buffer;
}
}  // namespace

MarkerArray buildMarkers(const ControllerCore& core, const ControlOutput& output, const RobotState& state,
                         const DynamicsParams& learned, const VisualizationOptions& options, const std::string& frame,
                         const rclcpp::Time& stamp)
{
  MarkerArray array;
  Marker clear;
  clear.action = Marker::DELETEALL;
  array.markers.push_back(clear);

  const ControllerParams& params = core.params();
  const double max_v = params.limits.max_v;
  const double dt = params.primitives.integration_dt;
  const Route& route = core.route();
  const bool planned = output.stats.evaluations > 0;

  if (output.has_selection)
  {
    array.markers.push_back(trajectory(frame, stamp, "selected", 0, output.selected, 0.05, max_v, 1.0));

    if (options.footprints)
    {
      Marker footprints = marker(frame, stamp, "selected_footprints", 0, Marker::LINE_LIST, 0.015);
      footprints.color = rgba(0.1, 0.8, 0.3, 0.7);
      const size_t stride = std::max<size_t>(1, static_cast<size_t>(std::lround(options.footprint_spacing / dt)));
      for (size_t i = stride - 1; i < output.selected.states.size(); i += stride)
      {
        addFootprint(footprints, params.footprint, output.selected.states[i].pose);
      }
      array.markers.push_back(footprints);

      std::vector<RobotState> stopping;
      core.validator().stoppingStates(state, output.selected.commands.front(), learned, dt, stopping);
      if (!stopping.empty())
      {
        Marker stop_path = marker(frame, stamp, "stop_check", 0, Marker::LINE_STRIP, 0.03);
        stop_path.color = rgba(1.0, 0.55, 0.0, 0.9);
        stop_path.points.push_back(point(state.pose.x, state.pose.y, 2 * kLineZ));
        for (const auto& s : stopping)
        {
          stop_path.points.push_back(point(s.pose.x, s.pose.y, 2 * kLineZ));
        }
        array.markers.push_back(stop_path);
        Marker stop_footprint = marker(frame, stamp, "stop_check", 1, Marker::LINE_LIST, 0.025);
        stop_footprint.color = stop_path.color;
        addFootprint(stop_footprint, params.footprint, stopping.back().pose);
        array.markers.push_back(stop_footprint);
      }
    }
  }

  if (options.candidates && planned)
  {
    const auto& candidates = core.candidates();
    const auto& verdicts = core.verdicts();
    for (size_t i = 0; i < candidates.size(); ++i)
    {
      const bool checked = i < verdicts.size();
      const bool rejected = checked && verdicts[i] != SafetyVerdict::Ok;
      if (!(checked && !rejected))
      {
        Marker line = trajectory(frame, stamp, "candidates", static_cast<int>(i), candidates[i], 0.015, max_v, 0.35);
        if (rejected)
        {
          line.colors.clear();
          line.color = rgba(1.0, 0.0, 0.0, 0.8);
        }
        array.markers.push_back(line);
      }
      if (options.labels && !candidates[i].states.empty())
      {
        const Pose2D& end = candidates[i].states.back().pose;
        char label[96];
        std::snprintf(label, sizeof(label), "#%zu %s %.2f %s", i, toString(candidates[i].maneuver),
                      candidates[i].total, checked ? toString(verdicts[i]) : "unchecked");
        array.markers.push_back(text(frame, stamp, "candidate_labels", static_cast<int>(i), end.x, end.y, 0.25, 0.08,
                                     rejected ? rgba(1.0, 0.3, 0.3, 1.0) : rgba(0.85, 0.85, 0.85, 0.9), label));
      }
    }
  }

  if (options.reference && !route.empty() && planned)
  {
    Marker line = marker(frame, stamp, "reference", 0, Marker::LINE_STRIP, 0.03);
    for (double s = 0.0; s <= route.length(); s += kRouteStep)
    {
      const RouteSample sample = route.sample(s);
      line.points.push_back(point(sample.position.x, sample.position.y, 0.005));
      line.colors.push_back(speedColor(sample.v_ref / max_v, 0.6));
    }
    if (line.points.size() >= 2)
    {
      array.markers.push_back(line);
    }
  }

  if (options.corridor && !route.empty() && planned)
  {
    const double preferred = params.scoring.filter_dist;
    const double outer = params.scoring.max_route_deviation;
    const ColorRGBA preferred_color = rgba(0.2, 0.8, 0.4, 0.5);
    const ColorRGBA outer_color = rgba(0.9, 0.2, 0.2, 0.5);
    array.markers.push_back(offsetRoute(frame, stamp, 0, route, preferred, preferred_color));
    array.markers.push_back(offsetRoute(frame, stamp, 1, route, -preferred, preferred_color));
    array.markers.push_back(offsetRoute(frame, stamp, 2, route, outer, outer_color));
    array.markers.push_back(offsetRoute(frame, stamp, 3, route, -outer, outer_color));

    const RouteSample projected = route.sample(output.route_progress);
    Marker projection = marker(frame, stamp, "projection", 0, Marker::LINE_LIST, 0.02);
    projection.color = rgba(1.0, 1.0, 1.0, 0.8);
    projection.points.push_back(point(state.pose.x, state.pose.y));
    projection.points.push_back(point(projected.position.x, projected.position.y));
    array.markers.push_back(projection);
    Marker foot = marker(frame, stamp, "projection", 1, Marker::SPHERE, 0.08);
    foot.pose.position = point(projected.position.x, projected.position.y);
    foot.color = projection.color;
    array.markers.push_back(foot);

    const MotionPrimitiveGenerator& generator = core.search().generator();
    const double speed = std::max(state.v, output.reference_speed);
    int id = 0;
    for (double offset : generator.params().lateral_offsets)
    {
      const Point2D target = generator.trackingTarget(route, output.route_progress, offset, speed);
      Marker dot = marker(frame, stamp, "offset_targets", id, Marker::SPHERE, 0.07);
      dot.pose.position = point(target.x, target.y);
      dot.color = rgba(0.3, 0.9, 1.0, 0.9);
      array.markers.push_back(dot);
      ++id;
    }
  }

  if (options.obstacles)
  {
    const ObstaclePredictionModel& obstacles = core.obstacles();
    int id = 0;
    for (size_t i = 0; i < obstacles.size(); ++i)
    {
      for (size_t k = 0; k < options.obstacle_times.size(); ++k)
      {
        const PredictedDisc disc = obstacles.predict(i, options.obstacle_times[k]);
        Marker cylinder = marker(frame, stamp, "obstacles", id++, Marker::CYLINDER, 2.0 * disc.radius);
        cylinder.scale.z = 0.05;
        cylinder.pose.position = point(disc.center.x, disc.center.y, 0.025);
        cylinder.color = rgba(0.8, 0.2, 0.8, 0.5 / (1.0 + static_cast<double>(k)));
        array.markers.push_back(cylinder);
      }
    }
  }

  if (output.goal_position_reached)
  {
    Marker arrow = marker(frame, stamp, "goal", 0, Marker::ARROW, 0.05);
    arrow.scale.x = 0.6;
    arrow.scale.y = 0.06;
    arrow.scale.z = 0.06;
    arrow.pose.position = point(state.pose.x, state.pose.y, 0.05);
    arrow.pose.orientation.z = std::sin(output.goal_heading / 2.0);
    arrow.pose.orientation.w = std::cos(output.goal_heading / 2.0);
    arrow.color = rgba(1.0, 0.85, 0.0, 0.9);
    array.markers.push_back(arrow);
  }

  const bool healthy = output.status == ControlStatus::Tracking || output.status == ControlStatus::TurningToGoal ||
                       output.status == ControlStatus::GoalReached;
  array.markers.push_back(text(frame, stamp, "status", 0, state.pose.x, state.pose.y, 0.5, 0.08,
                               healthy ? rgba(1.0, 1.0, 1.0, 1.0) : rgba(1.0, 0.3, 0.2, 1.0), statusText(output)));

  if (options.tree && planned)
  {
    std::vector<TreePrimitive> primitives;
    core.search().treePrimitives(primitives, options.max_tree_segments);
    Marker frontier = marker(frame, stamp, "tree", 0, Marker::LINE_LIST, 0.008);
    Marker pruned = marker(frame, stamp, "tree", 1, Marker::LINE_LIST, 0.004);
    Marker rejected = marker(frame, stamp, "tree", 2, Marker::LINE_LIST, 0.006);
    Marker ends = marker(frame, stamp, "tree", 3, Marker::SPHERE_LIST, 0.04);
    Marker contacts = marker(frame, stamp, "tree", 4, Marker::SPHERE_LIST, 0.05);
    pruned.color = outcomeColor(TreePrimitive::Outcome::Kept);
    ends.color = rgba(1.0, 1.0, 1.0, 0.8);
    contacts.color = rgba(1.0, 0.0, 0.0, 0.9);
    for (const auto& primitive : primitives)
    {
      const bool kept = primitive.outcome == TreePrimitive::Outcome::Kept;
      Marker& lines = !kept ? rejected : primitive.in_frontier ? frontier : pruned;
      for (size_t k = 1; k < primitive.states.size(); ++k)
      {
        const RobotState& a = primitive.states[k - 1];
        const RobotState& b = primitive.states[k];
        lines.points.push_back(point(a.pose.x, a.pose.y, 0.01));
        lines.points.push_back(point(b.pose.x, b.pose.y, 0.01));
        if (&lines != &pruned)
        {
          const ColorRGBA color = kept ? speedColor(b.v / max_v, 0.6) : outcomeColor(primitive.outcome);
          lines.colors.push_back(color);
          lines.colors.push_back(color);
        }
      }
      if (primitive.complete && !primitive.states.empty())
      {
        ends.points.push_back(point(primitive.states.back().pose.x, primitive.states.back().pose.y, 0.015));
      }
      if (primitive.has_collision)
      {
        contacts.points.push_back(point(primitive.collision.x, primitive.collision.y, 0.015));
      }
    }
    for (const Marker* m : { &frontier, &pruned, &rejected, &ends, &contacts })
    {
      if (!m->points.empty())
      {
        array.markers.push_back(*m);
      }
    }
  }
  return array;
}

}  // namespace nice_differential_drive_trajectory_planner
