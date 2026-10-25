#include "nice_differential_drive_trajectory_planner/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <nav2_costmap_2d/cost_values.hpp>

namespace nice_differential_drive_trajectory_planner
{
namespace
{
/** Beyond this center separation the circumscribed-circle bound is used instead of the exact polygon distance. */
constexpr double kExactDistanceRange = 0.5;
constexpr unsigned char kMaxRiskCost = nav2_costmap_2d::MAX_NON_OBSTACLE;

double segmentDistance(const Point2D& a, const Point2D& b, const Point2D& p)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_sq = dx * dx + dy * dy;
  const double t = length_sq > 0.0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length_sq, 0.0, 1.0) : 0.0;
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

Pose2D interpolate(const Pose2D& a, const Pose2D& b, double f)
{
  return { a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), wrapAngle(a.theta + f * wrapAngle(b.theta - a.theta)) };
}
}  // namespace

std::string CollisionCheckParams::validate() const
{
  if (!(resolution > 0.0) || !std::isfinite(resolution))
  {
    return "collision resolution must be positive";
  }
  if (!(dynamic_margin >= resolution))
  {
    return "dynamic_margin must be at least the collision resolution";
  }
  return {};
}

double signedDistanceToPolygon(const std::vector<Point2D>& polygon, const Point2D& p)
{
  double best = std::numeric_limits<double>::infinity();
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
  {
    const Point2D& a = polygon[j];
    const Point2D& b = polygon[i];
    best = std::min(best, segmentDistance(a, b, p));
    if ((b.y > p.y) != (a.y > p.y) && p.x < (a.x - b.x) * (p.y - b.y) / (a.y - b.y) + b.x)
    {
      inside = !inside;
    }
  }
  return inside ? -best : best;
}

CollisionChecker::CollisionChecker(const Footprint& footprint, const CollisionCheckParams& params)
  : footprint_(footprint), params_(params)
{
  for (const auto& point : footprint_)
  {
    circumscribed_radius_ = std::max(circumscribed_radius_, std::hypot(point.x, point.y));
  }
}

bool CollisionChecker::lethalCell(int mx, int my) const
{
  if (mx < 0 || my < 0 || mx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
      my >= static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return params_.unknown_is_lethal;
  }
  return lethal(costmap_->getCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my)));
}

bool CollisionChecker::edgeHitsLethal(const Point2D& a, const Point2D& b) const
{
  // Grid traversal (Amanatides & Woo): visits every cell the segment crosses, independent of any sampling step.
  const double resolution = costmap_->getResolution();
  const double ax = (a.x - costmap_->getOriginX()) / resolution;
  const double ay = (a.y - costmap_->getOriginY()) / resolution;
  const double dx = (b.x - a.x) / resolution;
  const double dy = (b.y - a.y) / resolution;
  int mx = static_cast<int>(std::floor(ax));
  int my = static_cast<int>(std::floor(ay));
  const int end_x = static_cast<int>(std::floor(ax + dx));
  const int end_y = static_cast<int>(std::floor(ay + dy));
  const int step_x = dx > 0.0 ? 1 : -1;
  const int step_y = dy > 0.0 ? 1 : -1;
  const double inf = std::numeric_limits<double>::infinity();
  const double delta_x = dx != 0.0 ? std::abs(1.0 / dx) : inf;
  const double delta_y = dy != 0.0 ? std::abs(1.0 / dy) : inf;
  double next_x = dx != 0.0 ? (step_x > 0 ? mx + 1.0 - ax : ax - mx) * delta_x : inf;
  double next_y = dy != 0.0 ? (step_y > 0 ? my + 1.0 - ay : ay - my) * delta_y : inf;
  const int cells = std::abs(end_x - mx) + std::abs(end_y - my);
  for (int i = 0; i <= cells; ++i)
  {
    if (lethalCell(mx, my))
    {
      return true;
    }
    if (next_x < next_y)
    {
      next_x += delta_x;
      mx += step_x;
    }
    else
    {
      next_y += delta_y;
      my += step_y;
    }
  }
  return false;
}

void CollisionChecker::setCostmap(const nav2_costmap_2d::Costmap2D* costmap, unsigned char circumscribed_cost)
{
  costmap_ = costmap;
  circumscribed_cost_ = circumscribed_cost;
}

void CollisionChecker::setObstacles(const ObstaclePredictionModel* obstacles)
{
  obstacles_ = obstacles;
}

bool CollisionChecker::lethal(unsigned char cost) const
{
  return cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
         (cost == nav2_costmap_2d::NO_INFORMATION && params_.unknown_is_lethal);
}

bool CollisionChecker::staticCollision(const Pose2D& pose, double* center_cost) const
{
  if (costmap_ == nullptr)
  {
    if (center_cost != nullptr)
    {
      *center_cost = 0.0;
    }
    return false;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  const unsigned char center = costmap_->worldToMap(pose.x, pose.y, mx, my) ? costmap_->getCost(mx, my) :
                                                                              nav2_costmap_2d::NO_INFORMATION;
  if (center_cost != nullptr)
  {
    *center_cost = center == nav2_costmap_2d::NO_INFORMATION ? 0.0 : std::min(center, kMaxRiskCost);
  }
  if (lethal(center))
  {
    return true;
  }
  if (center != nav2_costmap_2d::NO_INFORMATION && center < circumscribed_cost_)
  {
    return false;
  }

  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  auto world = [&](const Point2D& p) { return Point2D{ pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y }; };
  for (size_t i = 0; i < footprint_.size(); ++i)
  {
    if (edgeHitsLethal(world(footprint_[i]), world(footprint_[(i + 1) % footprint_.size()])))
    {
      return true;
    }
  }
  return false;
}

double CollisionChecker::dynamicClearance(const Pose2D& pose, double t) const
{
  double clearance = std::numeric_limits<double>::infinity();
  if (obstacles_ == nullptr)
  {
    return clearance;
  }
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  for (size_t i = 0; i < obstacles_->size(); ++i)
  {
    const PredictedDisc disc = obstacles_->predict(i, t);
    const double dx = disc.center.x - pose.x;
    const double dy = disc.center.y - pose.y;
    const double bound = std::hypot(dx, dy) - circumscribed_radius_ - disc.radius;
    if (bound > kExactDistanceRange)
    {
      clearance = std::min(clearance, bound);
      continue;
    }
    const Point2D local{ c * dx + s * dy, -s * dx + c * dy };
    clearance = std::min(clearance, signedDistanceToPolygon(footprint_, local) - disc.radius);
  }
  return clearance;
}

SweepResult CollisionChecker::sweep(const Pose2D& start, double t0, const RobotState* states, size_t count,
                                    double dt) const
{
  SweepResult result;
  const bool dynamic = obstacles_ != nullptr && !obstacles_->empty();
  const double boundary_motion = dynamic ? obstacles_->maxBoundarySpeed() * dt : 0.0;

  Pose2D previous = start;
  double t_previous = t0;
  for (size_t k = 0; k < count; ++k)
  {
    const Pose2D& current = states[k].pose;
    const double robot_motion = std::hypot(current.x - previous.x, current.y - previous.y) +
                                std::abs(wrapAngle(current.theta - previous.theta)) * circumscribed_radius_;
    const int substeps =
        std::max(1, static_cast<int>(std::ceil((robot_motion + boundary_motion) / params_.resolution)));
    for (int j = 1; j <= substeps; ++j)
    {
      const double f = static_cast<double>(j) / substeps;
      const Pose2D pose = interpolate(previous, current, f);
      const double t = t_previous + f * dt;

      double center_cost = 0.0;
      if (staticCollision(pose, &center_cost))
      {
        result.static_collision = true;
        result.collision_time = t;
        return result;
      }
      result.max_static_cost = std::max(result.max_static_cost, center_cost);

      if (dynamic)
      {
        const double clearance = dynamicClearance(pose, t);
        result.min_dynamic_clearance = std::min(result.min_dynamic_clearance, clearance);
        if (clearance < params_.dynamic_margin)
        {
          result.dynamic_collision = true;
          result.collision_time = t;
          return result;
        }
      }
    }
    previous = current;
    t_previous += dt;
  }
  return result;
}

}  // namespace nice_differential_drive_trajectory_planner
