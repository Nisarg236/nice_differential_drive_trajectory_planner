#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "nice_differential_drive_trajectory_planner/route.hpp"

using nice_differential_drive_trajectory_planner::Point2D;
using nice_differential_drive_trajectory_planner::Route;
using nice_differential_drive_trajectory_planner::RouteProgressTracker;
using nice_differential_drive_trajectory_planner::SpeedProfileParams;

namespace
{
constexpr double kSpacing = 0.05;

std::vector<Point2D> line(const Point2D& from, const Point2D& to, bool include_end = true)
{
  std::vector<Point2D> points;
  const double length = std::hypot(to.x - from.x, to.y - from.y);
  const int steps = static_cast<int>(std::round(length / kSpacing));
  for (int i = 0; i < steps + (include_end ? 1 : 0); ++i)
  {
    const double f = static_cast<double>(i) / steps;
    points.push_back({ from.x + f * (to.x - from.x), from.y + f * (to.y - from.y) });
  }
  return points;
}

std::vector<Point2D> arc(const Point2D& center, double radius, double from, double to)
{
  std::vector<Point2D> points;
  const int steps = static_cast<int>(std::ceil(std::abs(to - from) * radius / kSpacing));
  for (int i = 0; i <= steps; ++i)
  {
    const double angle = from + (to - from) * i / steps;
    points.push_back({ center.x + radius * std::cos(angle), center.y + radius * std::sin(angle) });
  }
  return points;
}

std::vector<Point2D> concat(std::vector<Point2D> a, const std::vector<Point2D>& b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

SpeedProfileParams openEnded()
{
  SpeedProfileParams params;
  params.end_speed = -1.0;
  return params;
}
}  // namespace

TEST(Route, straight_route_has_zero_curvature_and_full_speed)
{
  Route route;
  ASSERT_TRUE(route.build(line({ 0, 0 }, { 5, 0 }), openEnded()));
  EXPECT_NEAR(route.length(), 5.0, 1e-9);
  for (double s = 0.0; s <= 5.0; s += 0.25)
  {
    const auto sample = route.sample(s);
    EXPECT_NEAR(sample.curvature, 0.0, 1e-9);
    EXPECT_NEAR(sample.heading, 0.0, 1e-9);
    EXPECT_NEAR(sample.v_ref, 1.0, 1e-9);
  }
}

TEST(Route, curved_route_recovers_the_radius)
{
  Route route;
  const double radius = 2.0;
  ASSERT_TRUE(route.build(arc({ 0, radius }, radius, -M_PI / 2, M_PI / 2), openEnded()));
  const auto sample = route.sample(route.length() / 2);
  EXPECT_NEAR(sample.curvature, 1.0 / radius, 0.01);
  EXPECT_NEAR(sample.heading, M_PI / 2, 0.01);
  EXPECT_NEAR(sample.v_ref, std::min(1.0, std::sqrt(0.8 * radius)), 0.01);
}

TEST(Route, tight_turn_slows_down_before_the_corner_and_keeps_heading_continuous)
{
  Route route;
  ASSERT_TRUE(route.build(concat(line({ 0, 0 }, { 3, 0 }, false), line({ 3, 0 }, { 3, 3 })), openEnded()));
  const double corner = 3.0;
  EXPECT_LT(route.sample(corner).v_ref, 0.6);
  EXPECT_GT(route.sample(corner - 1.0).v_ref, route.sample(corner).v_ref);
  double previous = route.sample(0.0).heading;
  for (double s = 0.01; s <= route.length(); s += 0.01)
  {
    const double heading = route.sample(s).heading;
    EXPECT_LT(std::abs(heading - previous), 0.1) << "heading jump at s=" << s;
    previous = heading;
  }
}

TEST(Route, end_speed_ramps_down_at_decel)
{
  SpeedProfileParams params;
  params.decel = 0.5;
  Route route;
  ASSERT_TRUE(route.build(line({ 0, 0 }, { 5, 0 }), params));
  EXPECT_NEAR(route.referenceSpeed(5.0), 0.0, 1e-9);
  EXPECT_NEAR(route.referenceSpeed(4.0), 1.0, 1e-9);
  EXPECT_NEAR(route.referenceSpeed(4.75), std::sqrt(2.0 * 0.5 * 0.25), 0.01);
}

TEST(Route, speed_caps_limit_the_profile)
{
  auto points = line({ 0, 0 }, { 4, 0 });
  std::vector<double> caps(points.size(), 1.0);
  for (size_t i = 40; i < 60; ++i)
  {
    caps[i] = 0.3;
  }
  Route route;
  ASSERT_TRUE(route.build(points, openEnded(), caps));
  EXPECT_NEAR(route.referenceSpeed(2.5), 0.3, 1e-9);
  EXPECT_LT(route.referenceSpeed(1.9), 1.0);
  EXPECT_NEAR(route.referenceSpeed(0.5), 1.0, 1e-9);
}

TEST(Route, rejects_degenerate_input)
{
  Route route;
  EXPECT_FALSE(route.build({}, SpeedProfileParams{}));
  EXPECT_FALSE(route.build({ { 1, 1 } }, SpeedProfileParams{}));
  EXPECT_FALSE(route.build({ { 1, 1 }, { 1, 1 } }, SpeedProfileParams{}));
  EXPECT_TRUE(route.empty());
  EXPECT_FALSE(route.build({ { 0, 0 }, { 1, 0 } }, SpeedProfileParams{}, { 1.0 }));
  EXPECT_FALSE(route.project({ 0, 0 }, 0.0, 1.0).valid);
}

TEST(Route, projection_reports_signed_lateral_offset)
{
  Route route;
  ASSERT_TRUE(route.build(line({ 0, 0 }, { 5, 0 }), openEnded()));
  auto projection = route.project({ 2.0, 0.3 }, 0.0, 5.0);
  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.s, 2.0, 1e-9);
  EXPECT_NEAR(projection.lateral, 0.3, 1e-9);
  projection = route.project({ 2.0, -0.4 }, 0.0, 5.0);
  EXPECT_NEAR(projection.lateral, -0.4, 1e-9);
}

TEST(Route, parallel_aisle_does_not_capture_the_projection)
{
  // Out along y=0, U-turn, back along y=1.
  const auto points =
      concat(concat(line({ 0, 0 }, { 6, 0 }, false), arc({ 6, 0.5 }, 0.5, -M_PI / 2, M_PI / 2)),
             line({ 6, 1 }, { 0, 1 }));
  Route route;
  ASSERT_TRUE(route.build(points, openEnded()));

  RouteProgressTracker tracker(0.5, 1.5, 1.0);
  for (int i = 0; i <= 40; ++i)
  {
    const double x = 0.1 * i;
    // Drifted towards the return aisle, still closer to it than to the outbound one.
    const auto projection = tracker.update(route, { x, 0.6 });
    ASSERT_TRUE(projection.valid);
    EXPECT_NEAR(projection.s, x, 1e-6);
    EXPECT_NEAR(projection.lateral, 0.6, 1e-6);
  }
  EXPECT_NEAR(tracker.progress(), 4.0, 1e-6);
}

TEST(Route, self_crossing_route_keeps_the_current_pass)
{
  // Figure of eight: the second pass crosses the first one at (2, 0).
  const auto first = line({ 0, 0 }, { 4, 0 }, false);
  const auto loop = arc({ 4, 1 }, 1.0, -M_PI / 2, M_PI);
  const auto second = line({ 3, 1 }, { 3, -2 });
  const auto points = concat(concat(first, loop), second);
  Route route;
  ASSERT_TRUE(route.build(points, openEnded()));

  RouteProgressTracker tracker(0.3, 1.0, 1.0);
  for (int i = 0; i <= 35; ++i)
  {
    const double x = 0.1 * i;
    tracker.update(route, { x, 0.0 });
  }
  EXPECT_NEAR(tracker.progress(), 3.5, 1e-6) << "jumped to the crossing pass";
}

TEST(Route, progress_never_decreases)
{
  Route route;
  ASSERT_TRUE(route.build(line({ 0, 0 }, { 5, 0 }), openEnded()));
  RouteProgressTracker tracker(0.5, 1.0, 1.0);
  tracker.update(route, { 0.0, 0.0 });
  tracker.update(route, { 0.8, 0.0 });
  tracker.update(route, { 0.5, 0.0 });
  EXPECT_NEAR(tracker.progress(), 0.8, 1e-9);
  tracker.reset();
  EXPECT_FALSE(tracker.initialized());
}

TEST(Route, projection_outside_the_window_is_invalid)
{
  Route route;
  ASSERT_TRUE(route.build(line({ 0, 0 }, { 5, 0 }), openEnded()));
  EXPECT_FALSE(route.project({ 1, 0 }, 6.0, 7.0).valid);
  const auto clamped = route.project({ 1, 0 }, 3.0, 4.0);
  ASSERT_TRUE(clamped.valid);
  EXPECT_NEAR(clamped.s, 3.0, 1e-9);
  EXPECT_NEAR(clamped.distance, 2.0, 1e-9);
}
