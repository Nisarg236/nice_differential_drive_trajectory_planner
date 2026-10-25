#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "nice_differential_drive_trajectory_planner/types.hpp"

namespace nice_differential_drive_trajectory_planner
{
/** How the reference speed v_ref(s) is derived along the route. */
struct SpeedProfileParams
{
  double max_speed = 1.0;          ///< [m/s]
  double max_lateral_accel = 0.8;  ///< Caps speed at sqrt(a / |curvature|) [m/s^2].
  double decel = 1.0;              ///< Slows ahead of curves and the route end [m/s^2].
  double curvature_window = 0.4;   ///< Arc length the heading and curvature are smoothed over [m].
  double end_speed = 0.0;          ///< Speed at the last point [m/s]; negative leaves it unconstrained.

  std::string validate() const;
};

struct RouteSample
{
  Point2D position;
  double heading = 0.0;    ///< [rad]
  double curvature = 0.0;  ///< [1/m], positive turns left.
  double v_ref = 0.0;      ///< [m/s]
};

struct RouteProjection
{
  bool valid = false;
  double s = 0.0;         ///< Arc length of the closest point [m].
  double lateral = 0.0;   ///< Signed offset, positive left of the route [m].
  double distance = 0.0;  ///< |lateral| except beyond the route ends [m].
  size_t segment = 0;
};

/**
 * Arc-length parameterised route P(s) = (x, y, heading, v_ref). Heading and curvature are smoothed over
 * curvature_window so jagged grid-planner paths give a continuous reference.
 */
class Route
{
public:
  /**
   * @param[in] points      Route in the planning frame; consecutive duplicates are dropped.
   * @param[in] params      Reference speed derivation.
   * @param[in] speed_caps  Optional per-point speed limits [m/s], same size as points.
   * @return False when fewer than two distinct points remain or speed_caps has the wrong size.
   */
  bool build(const std::vector<Point2D>& points, const SpeedProfileParams& params,
             const std::vector<double>& speed_caps = {});

  void clear();
  bool empty() const
  {
    return s_.size() < 2;
  }
  double length() const
  {
    return empty() ? 0.0 : s_.back();
  }
  const std::vector<Point2D>& points() const
  {
    return points_;
  }

  /** Interpolated route state at s, clamped to [0, length]. */
  RouteSample sample(double s) const;
  double referenceSpeed(double s) const;

  /**
   * Closest point among the segments overlapping [s_min, s_max]; ties go to the smaller s so the result is
   * deterministic. Restricting the window keeps a pose from snapping to a nearby but unrelated part of the route.
   */
  RouteProjection project(const Point2D& p, double s_min, double s_max) const;

private:
  size_t segmentAt(double s) const;
  Point2D positionAt(double s) const;

  std::vector<Point2D> points_;
  std::vector<double> s_;
  std::vector<double> heading_;
  std::vector<double> curvature_;
  std::vector<double> v_ref_;
};

/** Route progress that only moves forward between calls, searched in a window around the last progress. */
class RouteProgressTracker
{
public:
  /**
   * @param[in] search_back     Window behind the last progress [m].
   * @param[in] search_ahead    Window ahead of the last progress [m].
   * @param[in] initial_search  Window from the route start before the first projection [m].
   */
  RouteProgressTracker(double search_back, double search_ahead, double initial_search);

  void reset();
  RouteProjection update(const Route& route, const Point2D& p);
  double progress() const
  {
    return progress_;
  }
  bool initialized() const
  {
    return initialized_;
  }

private:
  double search_back_;
  double search_ahead_;
  double initial_search_;
  double progress_ = 0.0;
  bool initialized_ = false;
};

}  // namespace nice_differential_drive_trajectory_planner
