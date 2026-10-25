#pragma once

#include <cmath>

namespace nice_differential_drive_trajectory_planner
{
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

/** Planar robot state; v is body-forward and never negative in a valid state. */
struct RobotState
{
  Pose2D pose;
  double v = 0.0;      ///< [m/s]
  double omega = 0.0;  ///< [rad/s]
};

/** Body velocity command. */
struct Command
{
  double v = 0.0;      ///< [m/s]
  double omega = 0.0;  ///< [rad/s]
};

inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double distance(const Point2D& a, const Point2D& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline Point2D position(const Pose2D& pose)
{
  return { pose.x, pose.y };
}

}  // namespace nice_differential_drive_trajectory_planner
