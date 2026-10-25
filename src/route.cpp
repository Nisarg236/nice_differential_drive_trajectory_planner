#include "nice_differential_drive_trajectory_planner/route.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nice_differential_drive_trajectory_planner
{
namespace
{
constexpr double kMinSpacing = 1e-6;
constexpr double kMinCurvature = 1e-6;
}  // namespace

std::string SpeedProfileParams::validate() const
{
  if (!(max_speed > 0.0) || !std::isfinite(max_speed))
  {
    return "max_speed must be positive";
  }
  if (!(max_lateral_accel > 0.0) || !(decel > 0.0))
  {
    return "max_lateral_accel and decel must be positive";
  }
  if (!(curvature_window > 0.0))
  {
    return "curvature_window must be positive";
  }
  if (!std::isfinite(end_speed))
  {
    return "end_speed must be finite";
  }
  return {};
}

bool Route::build(const std::vector<Point2D>& points, const SpeedProfileParams& params,
                  const std::vector<double>& speed_caps)
{
  clear();
  if (!speed_caps.empty() && speed_caps.size() != points.size())
  {
    return false;
  }

  std::vector<double> caps;
  for (size_t i = 0; i < points.size(); ++i)
  {
    if (!points_.empty() && distance(points_.back(), points[i]) < kMinSpacing)
    {
      if (!speed_caps.empty())
      {
        caps.back() = std::min(caps.back(), speed_caps[i]);
      }
      continue;
    }
    s_.push_back(points_.empty() ? 0.0 : s_.back() + distance(points_.back(), points[i]));
    points_.push_back(points[i]);
    caps.push_back(speed_caps.empty() ? params.max_speed : speed_caps[i]);
  }
  if (points_.size() < 2)
  {
    clear();
    return false;
  }

  const size_t n = points_.size();
  const double half_window = params.curvature_window / 2.0;
  heading_.resize(n);
  curvature_.resize(n);
  for (size_t i = 0; i < n; ++i)
  {
    const Point2D behind = positionAt(s_[i] - half_window);
    const Point2D ahead = positionAt(s_[i] + half_window);
    const size_t segment = std::min(i, n - 2);
    heading_[i] = distance(behind, ahead) > kMinSpacing ?
                      std::atan2(ahead.y - behind.y, ahead.x - behind.x) :
                      std::atan2(points_[segment + 1].y - points_[segment].y,
                                 points_[segment + 1].x - points_[segment].x);

    const double back_length = distance(behind, points_[i]);
    const double ahead_length = distance(points_[i], ahead);
    if (back_length < kMinSpacing || ahead_length < kMinSpacing)
    {
      curvature_[i] = 0.0;
      continue;
    }
    const double back_heading = std::atan2(points_[i].y - behind.y, points_[i].x - behind.x);
    const double ahead_heading = std::atan2(ahead.y - points_[i].y, ahead.x - points_[i].x);
    curvature_[i] = wrapAngle(ahead_heading - back_heading) / ((back_length + ahead_length) / 2.0);
  }

  v_ref_.resize(n);
  for (size_t i = 0; i < n; ++i)
  {
    double v = std::min(params.max_speed, caps[i]);
    if (std::abs(curvature_[i]) > kMinCurvature)
    {
      v = std::min(v, std::sqrt(params.max_lateral_accel / std::abs(curvature_[i])));
    }
    v_ref_[i] = std::max(0.0, v);
  }
  if (params.end_speed >= 0.0)
  {
    v_ref_.back() = std::min(v_ref_.back(), params.end_speed);
  }
  for (size_t i = n - 1; i-- > 0;)
  {
    const double reachable = std::sqrt(v_ref_[i + 1] * v_ref_[i + 1] + 2.0 * params.decel * (s_[i + 1] - s_[i]));
    v_ref_[i] = std::min(v_ref_[i], reachable);
  }
  return true;
}

void Route::clear()
{
  points_.clear();
  s_.clear();
  heading_.clear();
  curvature_.clear();
  v_ref_.clear();
}

size_t Route::segmentAt(double s) const
{
  const auto it = std::upper_bound(s_.begin(), s_.end(), s);
  const size_t index = it == s_.begin() ? 0 : static_cast<size_t>(it - s_.begin()) - 1;
  return std::min(index, s_.size() - 2);
}

Point2D Route::positionAt(double s) const
{
  const double clamped = std::clamp(s, 0.0, s_.back());
  const size_t i = segmentAt(clamped);
  const double f = (clamped - s_[i]) / (s_[i + 1] - s_[i]);
  return { points_[i].x + f * (points_[i + 1].x - points_[i].x), points_[i].y + f * (points_[i + 1].y - points_[i].y) };
}

RouteSample Route::sample(double s) const
{
  RouteSample sample;
  if (empty())
  {
    return sample;
  }
  const double clamped = std::clamp(s, 0.0, s_.back());
  const size_t i = segmentAt(clamped);
  const double f = (clamped - s_[i]) / (s_[i + 1] - s_[i]);
  sample.position = positionAt(clamped);
  sample.heading = wrapAngle(heading_[i] + f * wrapAngle(heading_[i + 1] - heading_[i]));
  sample.curvature = curvature_[i] + f * (curvature_[i + 1] - curvature_[i]);
  sample.v_ref = v_ref_[i] + f * (v_ref_[i + 1] - v_ref_[i]);
  return sample;
}

double Route::referenceSpeed(double s) const
{
  if (empty())
  {
    return 0.0;
  }
  const double clamped = std::clamp(s, 0.0, s_.back());
  const size_t i = segmentAt(clamped);
  const double f = (clamped - s_[i]) / (s_[i + 1] - s_[i]);
  return v_ref_[i] + f * (v_ref_[i + 1] - v_ref_[i]);
}

RouteProjection Route::project(const Point2D& p, double s_min, double s_max) const
{
  RouteProjection best;
  if (empty() || s_max < 0.0 || s_min > length() || s_min > s_max)
  {
    return best;
  }
  const size_t first = segmentAt(std::max(0.0, s_min));
  const size_t last = segmentAt(std::min(length(), s_max));
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = first; i <= last; ++i)
  {
    const Point2D& a = points_[i];
    const Point2D& b = points_[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length_sq, 0.0, 1.0);
    const Point2D closest{ a.x + t * dx, a.y + t * dy };
    const double d = distance(p, closest);
    if (d < best_distance)
    {
      best_distance = d;
      best.valid = true;
      best.segment = i;
      best.s = s_[i] + t * (s_[i + 1] - s_[i]);
      best.distance = d;
      const double cross = dx * (p.y - a.y) - dy * (p.x - a.x);
      best.lateral = std::copysign(d, cross);
    }
  }
  return best;
}

RouteProgressTracker::RouteProgressTracker(double search_back, double search_ahead, double initial_search)
  : search_back_(search_back), search_ahead_(search_ahead), initial_search_(initial_search)
{
}

void RouteProgressTracker::reset()
{
  progress_ = 0.0;
  initialized_ = false;
}

RouteProjection RouteProgressTracker::update(const Route& route, const Point2D& p)
{
  const RouteProjection projection = initialized_ ?
                                         route.project(p, progress_ - search_back_, progress_ + search_ahead_) :
                                         route.project(p, 0.0, initial_search_);
  if (projection.valid)
  {
    progress_ = initialized_ ? std::max(progress_, projection.s) : projection.s;
    initialized_ = true;
  }
  return projection;
}

}  // namespace nice_differential_drive_trajectory_planner
