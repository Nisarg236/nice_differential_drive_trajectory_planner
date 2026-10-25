#pragma once

#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nice_differential_drive_trajectory_planner/controller_core.hpp"

namespace nice_differential_drive_trajectory_planner
{
struct VisualizationOptions
{
  bool candidates = true;           ///< Validated candidates other than the selected one.
  bool footprints = true;           ///< Footprints along the selected trajectory and at the end of the stop check.
  bool reference = true;            ///< Route coloured by reference speed.
  bool obstacles = true;            ///< Predicted obstacle discs.
  bool corridor = true;             ///< Corridor bands, the robot's route projection and the offset targets.
  bool labels = false;              ///< Rank, maneuver, cost and safety verdict at the end of each candidate.
  bool tree = false;                ///< Every simulated primitive of the search, rejected ones included; heavy.
  size_t max_tree_segments = 6000;  ///< Primitives drawn at most.
  double footprint_spacing = 0.4;   ///< Time between drawn footprints [s].
  std::vector<double> obstacle_times{ 0.0, 1.0, 2.0 };  ///< Prediction times drawn per obstacle [s].
};

/**
 * Markers for one control cycle. Namespaces: selected, selected_footprints, stop_check, candidates,
 * candidate_labels, reference, corridor, projection, offset_targets, obstacles, goal, status, tree. The array starts
 * with DELETEALL so nothing from earlier cycles lingers.
 */
visualization_msgs::msg::MarkerArray buildMarkers(const ControllerCore& core, const ControlOutput& output,
                                                  const RobotState& state, const DynamicsParams& learned,
                                                  const VisualizationOptions& options, const std::string& frame,
                                                  const rclcpp::Time& stamp);

}  // namespace nice_differential_drive_trajectory_planner
