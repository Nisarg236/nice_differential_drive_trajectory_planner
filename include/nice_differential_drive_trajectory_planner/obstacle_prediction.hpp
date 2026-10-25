#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "nice_differential_drive_trajectory_planner/types.hpp"

namespace nice_differential_drive_trajectory_planner
{
/** One tracked obstacle as a tracker reports it, in the planning frame. */
struct ObstacleTrack
{
  uint64_t id = 0;
  double stamp = 0.0;  ///< Measurement time [s].
  Point2D position;
  Point2D velocity;              ///< [m/s]
  bool velocity_known = true;    ///< False when only positions are measured.
  double radius = 0.3;           ///< [m]
  double position_stddev = 0.05; ///< [m]
  double velocity_stddev = 0.1;  ///< [m/s]
};

struct ObstaclePredictionParams
{
  double uncertainty_sigmas = 2.0;      ///< Standard deviations added to every radius.
  double unknown_velocity_speed = 1.5;  ///< Speed assumed in every direction when velocity is unknown [m/s].
  double max_prediction_time = 3.0;     ///< Constant-velocity extrapolation stops here; uncertainty keeps growing [s].
  double max_track_age = 0.5;           ///< Older tracks lose their velocity and grow at unknown_velocity_speed [s].

  std::string validate() const;
};

/** Disc that covers the obstacle with the configured confidence. */
struct PredictedDisc
{
  Point2D center;
  double radius = 0.0;
};

/**
 * Time-indexed prediction of tracked obstacles: constant velocity with uncertainty growing with age. An obstacle
 * whose velocity is unknown or stale stays at its last position and grows at unknown_velocity_speed.
 */
class ObstaclePredictionModel
{
public:
  explicit ObstaclePredictionModel(const ObstaclePredictionParams& params = {});

  /**
   * @param[in] tracks  Replaces the current tracks; tracks with non-finite fields are dropped.
   * @param[in] now     Time that predict(t = 0) refers to [s].
   * @return Number of tracks dropped.
   */
  size_t setTracks(const std::vector<ObstacleTrack>& tracks, double now);
  void clear();

  size_t size() const
  {
    return tracks_.size();
  }
  bool empty() const
  {
    return tracks_.empty();
  }

  PredictedDisc predict(size_t index, double t) const;

  /** Upper bound on how fast any disc boundary moves [m/s]; sets the sweep step for relative motion. */
  double maxBoundarySpeed() const
  {
    return max_boundary_speed_;
  }

private:
  ObstaclePredictionParams params_;
  std::vector<ObstacleTrack> tracks_;
  double now_ = 0.0;
  double max_boundary_speed_ = 0.0;
};

}  // namespace nice_differential_drive_trajectory_planner
