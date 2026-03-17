"""
ROS 2 launch file for testing the wind direction sensor and compass calibration.

Equivalent to the ROS 1 test-wind-direction.launch file.
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
        _params('sailsettings_blackpython_rigA.yaml'),
        _params('servos_blackpython.yaml'),
    ]

    shared_params = [{'do_post': True}] + params_files

    return LaunchDescription([
        Node(
            package='sailing_robot',
            executable='sensor_service_imu',
            name='sensor_service_imu',
            parameters=shared_params,
            respawn=True,
        ),
        Node(
            package='sailing_robot',
            executable='sensor_driver_wind_direction',
            name='sensor_driver_wind_direction',
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
