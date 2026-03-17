"""
ROS 2 launch file for testing the GPS sensor.

Equivalent to the ROS 1 test-gps.launch file.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


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
    ]

    shared_params = [{'log_name': 'sailingclub_tests'}] + params_files

    return LaunchDescription([
        Node(
            package='sailing_robot',
            executable='sensor_driver_gps',
            name='sensor_driver_gps',
            parameters=shared_params,
            respawn=True,
        ),
        Node(
            package='sailing_robot',
            executable='debugging_gps_log',
            name='debugging_gps_log',
            parameters=shared_params,
            respawn=True,
        ),
        Node(
            package='sailing_robot',
            executable='debugging_dashboard',
            name='debugging_dashboard',
            parameters=shared_params,
        ),
    ])
