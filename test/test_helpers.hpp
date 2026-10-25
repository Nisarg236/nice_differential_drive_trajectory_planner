#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include "nice_differential_drive_trajectory_planner/controller_core.hpp"

namespace nice_differential_drive_trajectory_planner::test
{
constexpr double kResolution = 0.05;

/** 0.59 m x 0.54 m rectangle, centred 2 cm ahead of the wheel axis. */
inline Footprint rectangularFootprint()
{
  return { { 0.3154, 0.271 }, { -0.273, 0.271 }, { -0.273, -0.271 }, { 0.3154, -0.271 } };
}

/** Free costmap covering [origin, origin + size] in both axes. */
inline nav2_costmap_2d::Costmap2D freeCostmap(double size = 12.0, double origin_x = -2.0, double origin_y = -6.0)
{
  const auto cells = static_cast<unsigned int>(std::round(size / kResolution));
  return nav2_costmap_2d::Costmap2D(cells, cells, kResolution, origin_x, origin_y, nav2_costmap_2d::FREE_SPACE);
}

inline void fill(nav2_costmap_2d::Costmap2D& costmap, double min_x, double min_y, double max_x, double max_y,
                 unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE)
{
  for (double y = min_y; y <= max_y + 1e-9; y += kResolution / 2)
  {
    for (double x = min_x; x <= max_x + 1e-9; x += kResolution / 2)
    {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (costmap.worldToMap(x, y, mx, my))
      {
        costmap.setCost(mx, my, cost);
      }
    }
  }
}

/** nav2-style inflation around every lethal cell: inscribed band, then exponential decay to the radius. */
inline void inflate(nav2_costmap_2d::Costmap2D& costmap, double inscribed = 0.27, double radius = 0.7,
                    double scaling = 3.0)
{
  std::vector<std::pair<double, double>> lethal;
  for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my)
  {
    for (unsigned int mx = 0; mx < costmap.getSizeInCellsX(); ++mx)
    {
      if (costmap.getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        double wx = 0.0, wy = 0.0;
        costmap.mapToWorld(mx, my, wx, wy);
        lethal.emplace_back(wx, wy);
      }
    }
  }
  for (unsigned int my = 0; my < costmap.getSizeInCellsY(); ++my)
  {
    for (unsigned int mx = 0; mx < costmap.getSizeInCellsX(); ++mx)
    {
      if (costmap.getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        continue;
      }
      double wx = 0.0, wy = 0.0;
      costmap.mapToWorld(mx, my, wx, wy);
      double d = radius + 1.0;
      for (const auto& [lx, ly] : lethal)
      {
        d = std::min(d, std::hypot(wx - lx, wy - ly));
      }
      if (d <= inscribed)
      {
        costmap.setCost(mx, my, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      }
      else if (d <= radius)
      {
        costmap.setCost(mx, my, static_cast<unsigned char>(252.0 * std::exp(-scaling * (d - inscribed))));
      }
    }
  }
}

inline std::vector<Point2D> straightRoute(double length, double y = 0.0, double spacing = kResolution)
{
  std::vector<Point2D> points;
  const int steps = static_cast<int>(std::round(length / spacing));
  for (int i = 0; i <= steps; ++i)
  {
    points.push_back({ i * spacing, y });
  }
  return points;
}

inline ControllerParams defaultParams()
{
  ControllerParams params;
  params.footprint = rectangularFootprint();
  params.limits.max_v = 1.6;
  params.limits.max_omega = 2.5;
  params.limits.max_accel = 2.0;
  params.limits.max_brake = 2.0;
  params.limits.wheel_radius = 0.06;
  params.limits.wheel_separation = 0.32;
  params.limits.max_wheel_speed = 30.0;
  params.speed_profile.max_speed = 1.6;
  params.collision.resolution = 0.02;
  params.collision.dynamic_margin = 0.1;
  params.safety.collision = { 0.02, 0.05, true };
  return params;
}

inline RobotState stateAt(double x, double y, double theta = 0.0, double v = 0.0, double omega = 0.0)
{
  RobotState state;
  state.pose = { x, y, theta };
  state.v = v;
  state.omega = omega;
  return state;
}

}  // namespace nice_differential_drive_trajectory_planner::test
