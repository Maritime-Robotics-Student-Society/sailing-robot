"""
ROS 2 launch file for the sailing robot simulator.

Equivalent to the ROS 1 simulator.launch file.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import yaml


def _params(filename):
    """Return the full path of a parameter file shipped with this package."""
    return os.path.join(
        get_package_share_directory('sailing_robot'),
        'launch', 'parameters', filename)


def generate_launch_description():
    params_files = [
        _params('default.yaml'),
        _params('calibration_blackpython.yaml'),
        _params('sailsettings_blackpython.yaml'),
        _params('servos_blackpython.yaml'),
        _params('sailingClub_waypoints.yaml'),
        _params('simulator.yaml'),
    ]

    shared_params = [{'log_name': 'simulator_test'}] + params_files

    return LaunchDescription([
        Node(
            package='sailing_robot',
            executable='tasks',
            name='tasks',
            parameters=shared_params,
        ),
        Node(
            package='sailing_robot',
            executable='helming',
            name='helming',
            parameters=shared_params,
        ),
        Node(
            package='sailing_robot',
            executable='simulation_position',
            name='simulation_position',
            parameters=shared_params,
        ),
        Node(
            package='sailing_robot',
            executable='simulation_velocity',
            name='simulation_velocity',
            parameters=shared_params,
        ),
        Node(
            package='sailing_robot',
            executable='simulation_wind_apparent',
            name='simulation_wind_apparent',
            parameters=shared_params,
        ),
        Node(
            package='sailing_robot',
            executable='simulation_heading',
            name='simulation_heading',
            parameters=shared_params,
        ),
        Node(
            package='sailing_robot',
            executable='sensor_processed_wind_direction',
            name='sensor_processed_wind_direction',
            parameters=shared_params,
        ),
        Node(
            package='sailing_robot',
            executable='simulation_gps_fix',
            name='simulation_gps_fix',
            parameters=shared_params,
        ),
        Node(
            package='sailing_robot',
            executable='debugging_dashboard',
            name='debugging_dashboard',
            parameters=shared_params,
        ),
    ])
