"""nav2_bringup's TurtleBot3 Gazebo simulation with nice_differential_drive_trajectory_planner as FollowPath."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('nice_differential_drive_trajectory_planner')
    bringup = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    headless = LaunchConfiguration('headless')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'nav2_params_tb3.yaml')),
        DeclareLaunchArgument(
            'rviz_config_file', default_value=os.path.join(share, 'rviz', 'nav2_default_view_planner.rviz')),
        DeclareLaunchArgument('headless', default_value='True', description='False opens the Gazebo window'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'tb3_simulation_launch.py')),
            launch_arguments={
                'params_file': params_file,
                'rviz_config_file': rviz_config_file,
                'headless': headless,
            }.items(),
        ),
    ])
