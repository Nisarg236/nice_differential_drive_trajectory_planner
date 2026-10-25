#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "test_helpers.hpp"

using namespace nice_differential_drive_trajectory_planner;
using namespace nice_differential_drive_trajectory_planner::test;

namespace
{
std::vector<RobotState> straightStates(double from_x, double to_x, int count, double y = 0.0)
{
  std::vector<RobotState> states;
  for (int i = 1; i <= count; ++i)
  {
    states.push_back(stateAt(from_x + (to_x - from_x) * i / count, y, 0.0, 1.0));
  }
  return states;
}

ObstacleTrack track(double x, double y, double vx, double vy, double radius = 0.25)
{
  ObstacleTrack obstacle;
  obstacle.position = { x, y };
  obstacle.velocity = { vx, vy };
  obstacle.radius = radius;
  obstacle.position_stddev = 0.0;
  obstacle.velocity_stddev = 0.0;
  return obstacle;
}
}  // namespace

TEST(Polygon, signed_distance_is_negative_inside)
{
  const Footprint square{ { 1, 1 }, { -1, 1 }, { -1, -1 }, { 1, -1 } };
  EXPECT_NEAR(signedDistanceToPolygon(square, { 0, 0 }), -1.0, 1e-9);
  EXPECT_NEAR(signedDistanceToPolygon(square, { 3, 0 }), 2.0, 1e-9);
  EXPECT_NEAR(signedDistanceToPolygon(square, { 2, 2 }), std::sqrt(2.0), 1e-9);
}

TEST(CollisionChecker, detects_a_lethal_cell_under_the_outline)
{
  auto costmap = freeCostmap();
  fill(costmap, 1.0, -0.05, 1.05, 0.05);
  CollisionChecker checker(rectangularFootprint(), CollisionCheckParams{});
  checker.setCostmap(&costmap, 0);
  EXPECT_FALSE(checker.staticCollision({ 0.0, 0.0, 0.0 }));
  EXPECT_TRUE(checker.staticCollision({ 0.72, 0.0, 0.0 }));
  EXPECT_FALSE(checker.staticCollision({ 0.0, 1.0, 0.0 }));
}

TEST(CollisionChecker, sweep_catches_a_thin_wall_between_sparse_states)
{
  auto costmap = freeCostmap();
  fill(costmap, 2.0, -1.0, 2.0, 1.0);
  CollisionChecker checker(rectangularFootprint(), CollisionCheckParams{});
  checker.setCostmap(&costmap, 0);

  // Both states clear the wall; only the motion between them crosses it.
  const std::vector<RobotState> states{ stateAt(4.0, 0.0, 0.0, 1.0) };
  EXPECT_FALSE(checker.staticCollision(states.front().pose));
  const SweepResult result = checker.sweep({ 0.0, 0.0, 0.0 }, 0.0, states.data(), states.size(), 1.0);
  EXPECT_TRUE(result.static_collision);
  EXPECT_LT(result.collision_time, 0.5);
}

TEST(CollisionChecker, sweep_catches_rotation_into_a_wall)
{
  auto costmap = freeCostmap();
  fill(costmap, -0.1, 0.33, 0.1, 0.36);
  CollisionChecker checker(rectangularFootprint(), CollisionCheckParams{});
  checker.setCostmap(&costmap, 0);
  const std::vector<RobotState> states{ stateAt(0.0, 0.0, M_PI) };
  EXPECT_FALSE(checker.staticCollision({ 0.0, 0.0, 0.0 }));
  EXPECT_FALSE(checker.staticCollision(states.front().pose));
  EXPECT_TRUE(checker.sweep({ 0.0, 0.0, 0.0 }, 0.0, states.data(), 1, 1.0).static_collision);
}

TEST(CollisionChecker, outline_check_does_not_depend_on_the_sweep_resolution)
{
  // Regression: point samples along the outline missed cells it only clipped, so the search (2.5 cm) accepted
  // grazing paths the validator (2 cm) rejected, and the robot froze next to obstacles.
  auto costmap = freeCostmap();
  fill(costmap, 0.6, -0.1, 0.8, 0.1);
  Footprint circle;
  for (int i = 0; i < 16; ++i)
  {
    circle.push_back({ 0.23 * std::cos(2.0 * M_PI * i / 16.0), 0.23 * std::sin(2.0 * M_PI * i / 16.0) });
  }
  CollisionChecker coarse(circle, { 0.025, 0.1, true });
  CollisionChecker fine(circle, { 0.02, 0.1, true });
  coarse.setCostmap(&costmap, 0);
  fine.setCostmap(&costmap, 0);
  int disagreements = 0;
  int contacts = 0;
  for (int i = 0; i < 4000; ++i)
  {
    const Pose2D pose{ 0.2 + 0.25 * std::sin(i * 0.37), 0.35 * std::sin(i * 0.53 + 1.0), i * 0.11 };
    const bool a = coarse.staticCollision(pose);
    disagreements += a != fine.staticCollision(pose);
    contacts += a;
  }
  EXPECT_GT(contacts, 100) << "the poses must actually reach the obstacle";
  EXPECT_EQ(disagreements, 0);
}

TEST(CollisionChecker, unknown_and_off_map_cells_are_lethal_only_when_configured)
{
  auto costmap = freeCostmap();
  fill(costmap, 1.0, -0.05, 1.05, 0.05, nav2_costmap_2d::NO_INFORMATION);
  CollisionCheckParams params;
  CollisionChecker strict(rectangularFootprint(), params);
  strict.setCostmap(&costmap, 0);
  EXPECT_TRUE(strict.staticCollision({ 0.72, 0.0, 0.0 }));
  EXPECT_TRUE(strict.staticCollision({ 100.0, 0.0, 0.0 }));

  params.unknown_is_lethal = false;
  CollisionChecker lenient(rectangularFootprint(), params);
  lenient.setCostmap(&costmap, 0);
  EXPECT_FALSE(lenient.staticCollision({ 0.72, 0.0, 0.0 }));
}

TEST(CollisionChecker, circumscribed_cost_skips_the_outline_only_when_safe)
{
  auto costmap = freeCostmap();
  fill(costmap, 1.0, -0.05, 1.05, 0.05);
  CollisionChecker checker(rectangularFootprint(), CollisionCheckParams{});
  checker.setCostmap(&costmap, 128);
  // Center cell is free and below the circumscribed cost: with a consistent inflation layer this is exact.
  EXPECT_FALSE(checker.staticCollision({ 0.72, 0.0, 0.0 }));
  fill(costmap, 0.6, -0.1, 0.8, 0.1, 200);
  EXPECT_TRUE(checker.staticCollision({ 0.72, 0.0, 0.0 }));
}

TEST(CollisionChecker, head_on_obstacle_collides_at_the_meeting_time)
{
  ObstaclePredictionModel obstacles;
  obstacles.setTracks({ track(6.0, 0.0, -1.0, 0.0) }, 0.0);
  CollisionCheckParams params;
  params.dynamic_margin = 0.05;
  CollisionChecker checker(rectangularFootprint(), params);
  checker.setObstacles(&obstacles);

  const auto states = straightStates(0.0, 4.0, 40);
  const SweepResult result = checker.sweep({ 0.0, 0.0, 0.0 }, 0.0, states.data(), states.size(), 0.1);
  ASSERT_TRUE(result.dynamic_collision);
  // Gap closes at 2 m/s from 6 - 0.3154 - 0.25 - margin.
  EXPECT_NEAR(result.collision_time, (6.0 - 0.3154 - 0.25 - 0.05) / 2.0, 0.03);
}

TEST(CollisionChecker, crossing_obstacle_is_caught_only_at_the_matching_time)
{
  ObstaclePredictionModel obstacles;
  // Crosses y = 0 at x = 2 when t = 2 s.
  obstacles.setTracks({ track(2.0, -2.0, 0.0, 1.0) }, 0.0);
  CollisionChecker checker(rectangularFootprint(), CollisionCheckParams{});
  checker.setObstacles(&obstacles);

  const auto states = straightStates(0.0, 4.0, 40);
  EXPECT_TRUE(checker.sweep({ 0.0, 0.0, 0.0 }, 0.0, states.data(), states.size(), 0.1).collision())
      << "at 1 m/s the robot reaches x=2 together with the obstacle";
  EXPECT_FALSE(checker.sweep({ 0.0, 0.0, 0.0 }, 0.0, states.data(), states.size(), 0.025).collision())
      << "at 4 m/s the robot passes x=2 at t=0.5 while the obstacle is 1.5 m away";
}

TEST(CollisionChecker, sweep_resolution_follows_fast_obstacles)
{
  ObstaclePredictionModel obstacles;
  // A fast obstacle crosses the robot's resting pose between the two sample times.
  obstacles.setTracks({ track(0.0, -3.0, 0.0, 6.0, 0.1) }, 0.0);
  CollisionChecker checker(rectangularFootprint(), CollisionCheckParams{});
  checker.setObstacles(&obstacles);
  const std::vector<RobotState> resting{ stateAt(0.0, 0.0) };
  EXPECT_GT(checker.dynamicClearance({ 0.0, 0.0, 0.0 }, 0.0), 1.0);
  EXPECT_GT(checker.dynamicClearance({ 0.0, 0.0, 0.0 }, 1.0), 1.0);
  EXPECT_TRUE(checker.sweep({ 0.0, 0.0, 0.0 }, 0.0, resting.data(), 1, 1.0).dynamic_collision);
}

TEST(ObstaclePrediction, uncertainty_grows_with_time)
{
  ObstaclePredictionParams params;
  params.uncertainty_sigmas = 2.0;
  ObstaclePredictionModel model(params);
  ObstacleTrack obstacle = track(1.0, 0.0, 1.0, 0.0);
  obstacle.position_stddev = 0.1;
  obstacle.velocity_stddev = 0.2;
  model.setTracks({ obstacle }, 0.0);
  const PredictedDisc now = model.predict(0, 0.0);
  const PredictedDisc later = model.predict(0, 1.0);
  EXPECT_NEAR(now.radius, 0.25 + 0.2, 1e-9);
  EXPECT_NEAR(later.center.x, 2.0, 1e-9);
  EXPECT_NEAR(later.radius, 0.25 + 2.0 * std::hypot(0.1, 0.2), 1e-9);
}

TEST(ObstaclePrediction, unknown_or_stale_velocity_grows_a_reachable_set_in_place)
{
  ObstaclePredictionParams params;
  params.uncertainty_sigmas = 0.0;
  params.unknown_velocity_speed = 1.5;
  params.max_track_age = 0.5;
  ObstaclePredictionModel model(params);

  ObstacleTrack position_only = track(1.0, 0.0, 0.0, 0.0);
  position_only.velocity_known = false;
  ObstacleTrack stale = track(3.0, 0.0, 1.0, 0.0);
  stale.stamp = -1.0;
  model.setTracks({ position_only, stale }, 0.0);

  EXPECT_NEAR(model.predict(0, 1.0).center.x, 1.0, 1e-9);
  EXPECT_NEAR(model.predict(0, 1.0).radius, 0.25 + 1.5, 1e-9);
  EXPECT_NEAR(model.predict(1, 0.0).center.x, 3.0, 1e-9) << "stale velocity must not be extrapolated";
  EXPECT_NEAR(model.predict(1, 0.0).radius, 0.25 + 1.5, 1e-9);
  EXPECT_NEAR(model.maxBoundarySpeed(), 1.5, 1e-9);
}

TEST(ObstaclePrediction, keeps_growing_after_the_prediction_horizon)
{
  ObstaclePredictionParams params;
  params.uncertainty_sigmas = 0.0;
  params.max_prediction_time = 1.0;
  ObstaclePredictionModel model(params);
  model.setTracks({ track(0.0, 0.0, 1.0, 0.0) }, 0.0);
  const PredictedDisc disc = model.predict(0, 3.0);
  EXPECT_NEAR(disc.center.x, 1.0, 1e-9);
  EXPECT_NEAR(disc.radius, 0.25 + 2.0, 1e-9);
}

TEST(ObstaclePrediction, drops_non_finite_tracks)
{
  ObstaclePredictionModel model;
  ObstacleTrack broken = track(std::nan(""), 0.0, 0.0, 0.0);
  EXPECT_EQ(model.setTracks({ broken, track(1.0, 0.0, 0.0, 0.0) }, 0.0), 1u);
  EXPECT_EQ(model.size(), 1u);
}
