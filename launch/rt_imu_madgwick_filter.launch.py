import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import Shutdown, GroupAction
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    autostart = LaunchConfiguration('autostart', default='true')
    log_level = LaunchConfiguration('log_level',  default='info')

    lifecycle_nodes = ['rt_usb_9axisimu_driver']
    bringup_dir = get_package_share_directory('imu_filter_madgwick')
    launch_dir = os.path.join(bringup_dir, 'launch')

    load_nodes = GroupAction(
        actions=[
            Node(
                package='rt_usb_9axisimu_driver',
                executable='rt_usb_9axisimu_driver',
                output='screen',
                parameters=[{'port': '/dev/rt-usb-9axisimu-driver'}],
                on_exit=Shutdown()),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),

                        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                           'imu_filter.launch.py'))),
        ])

    ld = LaunchDescription()
    ld.add_action(load_nodes)

    return ld
