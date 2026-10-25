cmake_minimum_required(VERSION 3.10)
project(nice_differential_drive_trajectory_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_core SHARED
  src/collision_checker.cpp
  src/controller_core.cpp
  src/dynamics_model.cpp
  src/motion_primitives.cpp
  src/obstacle_prediction.cpp
  src/route.cpp
  src/safety_validator.cpp
  src/trajectory_scorer.cpp
  src/trajectory_search.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_20)
# ament_target_dependencies() was removed in ROS 2 Lyrical; ${<pkg>_TARGETS} lists every exported target.
target_link_libraries(${PROJECT_NAME}_core PUBLIC ${nav2_costmap_2d_TARGETS})

set(plugin_dependencies
  rclcpp
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav2_ros_common
  nav2_util
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)
foreach(dep IN LISTS plugin_dependencies)
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/predictive_controller.cpp
  src/visualization.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_core)
foreach(dep IN LISTS plugin_dependencies)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${${dep}_TARGETS})
endforeach()

pluginlib_export_plugin_description_file(nav2_core predictive_controller_plugin.xml)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  foreach(test_name
    test_collision_checker
    test_controller_core
    test_dynamics_model
    test_motion_primitives
    test_route
    test_safety_validator
    test_trajectory_scorer
    test_trajectory_search)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME}_core)
  endforeach()

  find_package(ament_cmake_ros REQUIRED)
  # Isolated ROS domain: the test uses the plugin's real topic names.
  ament_add_ros_isolated_gtest(test_predictive_controller test/test_predictive_controller.cpp)
  target_link_libraries(test_predictive_controller ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core ${PROJECT_NAME})
ament_export_dependencies(${plugin_dependencies})

ament_package()
