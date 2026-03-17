"""
ROS 2 launch file for the XSens MT/MTi/MTi-G IMU driver.

Equivalent to the ROS 1 xsens_driver.launch file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device', default_value='auto',
            description='Device file of the IMU'),
        DeclareLaunchArgument(
            'baudrate', default_value='0',
            description='Baudrate of the IMU'),
        DeclareLaunchArgument(
            'timeout', default_value='0.002',
            description='Timeout for IMU communication'),
        DeclareLaunchArgument(
            'frame_id', default_value='odom_frame',
            description='Frame ID of the IMU'),
        DeclareLaunchArgument(
            'frame_local', default_value='NED',
            description='Desired frame orientation (ENU, NED or NWU)'),
        DeclareLaunchArgument(
            'no_rotation_duration', default_value='0',
            description='Duration (seconds) of no-rotation calibration'),
        DeclareLaunchArgument(
            'filter_scenario', default_value='53',
            description='Filter scenario: 50=general, 51=high_mag_dep, '
                         '52=dynamic, 53=north_reference, 54=vru_general'),

        Node(
            package='xsens_driver',
            executable='mtnode.py',
            name='xsens_driver',
            output='screen',
            respawn=True,
            parameters=[{
                'device': LaunchConfiguration('device'),
                'baudrate': LaunchConfiguration('baudrate'),
                'timeout': LaunchConfiguration('timeout'),
                'frame_id': LaunchConfiguration('frame_id'),
                'frame_local': LaunchConfiguration('frame_local'),
                'no_rotation_duration': LaunchConfiguration('no_rotation_duration'),
                'filter_scenario': LaunchConfiguration('filter_scenario'),
            }],
        ),
    ])
