#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <nav2_costmap_2d/costmap_2d.hpp>

#include "nice_differential_drive_trajectory_planner/obstacle_prediction.hpp"
#include "nice_differential_drive_trajectory_planner/types.hpp"

namespace nice_differential_drive_trajectory_planner
{
using Footprint = std::vector<Point2D>;

struct CollisionCheckParams
{
  double resolution = 0.02;     ///< Largest move of any footprint or obstacle boundary point between checks [m].
  double dynamic_margin = 0.1;  ///< Required separation from predicted obstacles [m]; at least resolution.
  bool unknown_is_lethal = true;

  std::string validate() const;
};

struct SweepResult
{
  bool static_collision = false;
  bool dynamic_collision = false;
  double max_static_cost = 0.0;  ///< Highest costmap cost at the robot center, 0-252.
  double min_dynamic_clearance = std::numeric_limits<double>::infinity();  ///< [m]
  double collision_time = std::numeric_limits<double>::infinity();         ///< [s]

  bool collision() const
  {
    return static_collision || dynamic_collision;
  }
};

/**
 * Swept-footprint checks against the costmap and predicted obstacles. Between consecutive states it inserts poses
 * so that neither the footprint nor any obstacle boundary moves more than resolution, so contacts cannot fall
 * between samples.
 *
 * The costmap must stay locked by the caller for as long as checks run.
 */
class CollisionChecker
{
public:
  CollisionChecker(const Footprint& footprint, const CollisionCheckParams& params);

  /**
   * @param[in] costmap            nullptr disables static checks.
   * @param[in] circumscribed_cost Center cost below which the footprint cannot touch a lethal cell; 0 always
   *                               checks the outline.
   */
  void setCostmap(const nav2_costmap_2d::Costmap2D* costmap, unsigned char circumscribed_cost);
  void setObstacles(const ObstaclePredictionModel* obstacles);

  /** Checks the motion from start (at time t0) through count states spaced dt apart; stops at the first contact. */
  SweepResult sweep(const Pose2D& start, double t0, const RobotState* states, size_t count, double dt) const;

  /** True when the footprint at pose overlaps a lethal (or, if configured, unknown or off-map) cell. */
  bool staticCollision(const Pose2D& pose, double* center_cost = nullptr) const;

  /** Smallest separation between the footprint at pose and any obstacle predicted t seconds ahead [m]. */
  double dynamicClearance(const Pose2D& pose, double t) const;

  double circumscribedRadius() const
  {
    return circumscribed_radius_;
  }
  const Footprint& footprint() const
  {
    return footprint_;
  }
  const CollisionCheckParams& params() const
  {
    return params_;
  }

private:
  bool lethal(unsigned char cost) const;
  bool lethalCell(int mx, int my) const;
  bool edgeHitsLethal(const Point2D& a, const Point2D& b) const;

  Footprint footprint_;
  CollisionCheckParams params_;
  double circumscribed_radius_ = 0.0;
  const nav2_costmap_2d::Costmap2D* costmap_ = nullptr;
  unsigned char circumscribed_cost_ = 0;
  const ObstaclePredictionModel* obstacles_ = nullptr;
};

/** Signed distance from p to the polygon boundary, negative inside. */
double signedDistanceToPolygon(const std::vector<Point2D>& polygon, const Point2D& p);

}  // namespace nice_differential_drive_trajectory_planner
