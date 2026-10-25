#include "nice_differential_drive_trajectory_planner/obstacle_prediction.hpp"

#include <algorithm>
#include <cmath>

namespace nice_differential_drive_trajectory_planner
{
std::string ObstaclePredictionParams::validate() const
{
  if (!(uncertainty_sigmas >= 0.0) || !std::isfinite(uncertainty_sigmas))
  {
    return "uncertainty_sigmas must be finite and non-negative";
  }
  if (!(unknown_velocity_speed >= 0.0) || !std::isfinite(unknown_velocity_speed))
  {
    return "unknown_velocity_speed must be finite and non-negative";
  }
  if (!(max_prediction_time > 0.0) || !(max_track_age > 0.0))
  {
    return "max_prediction_time and max_track_age must be positive";
  }
  return {};
}

ObstaclePredictionModel::ObstaclePredictionModel(const ObstaclePredictionParams& params) : params_(params)
{
}

size_t ObstaclePredictionModel::setTracks(const std::vector<ObstacleTrack>& tracks, double now)
{
  tracks_.clear();
  now_ = now;
  max_boundary_speed_ = 0.0;
  size_t dropped = 0;
  for (ObstacleTrack track : tracks)
  {
    const bool finite = std::isfinite(track.stamp) && std::isfinite(track.position.x) &&
                        std::isfinite(track.position.y) && std::isfinite(track.radius) && track.radius >= 0.0 &&
                        std::isfinite(track.position_stddev) && std::isfinite(track.velocity_stddev);
    if (!finite)
    {
      ++dropped;
      continue;
    }
    if (!std::isfinite(track.velocity.x) || !std::isfinite(track.velocity.y) ||
        now - track.stamp > params_.max_track_age)
    {
      track.velocity_known = false;
    }
    if (!track.velocity_known)
    {
      track.velocity = {};
    }
    const double speed = std::hypot(track.velocity.x, track.velocity.y);
    const double boundary_speed = speed + params_.uncertainty_sigmas * std::abs(track.velocity_stddev) +
                                  (track.velocity_known ? 0.0 : params_.unknown_velocity_speed);
    max_boundary_speed_ = std::max(max_boundary_speed_, boundary_speed);
    tracks_.push_back(track);
  }
  return dropped;
}

void ObstaclePredictionModel::clear()
{
  tracks_.clear();
  max_boundary_speed_ = 0.0;
}

PredictedDisc ObstaclePredictionModel::predict(size_t index, double t) const
{
  const ObstacleTrack& track = tracks_[index];
  const double age = std::max(0.0, now_ + t - track.stamp);
  const double extrapolated = std::min(age, params_.max_prediction_time);
  const double speed = std::hypot(track.velocity.x, track.velocity.y);

  PredictedDisc disc;
  disc.center = { track.position.x + track.velocity.x * extrapolated,
                  track.position.y + track.velocity.y * extrapolated };
  const double stddev = std::hypot(track.position_stddev, track.velocity_stddev * age);
  disc.radius = track.radius + params_.uncertainty_sigmas * stddev + speed * (age - extrapolated);
  if (!track.velocity_known)
  {
    disc.radius += params_.unknown_velocity_speed * age;
  }
  return disc;
}

}  // namespace nice_differential_drive_trajectory_planner
