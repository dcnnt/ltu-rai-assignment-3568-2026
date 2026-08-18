#!/usr/bin/env python3
# gz_sim_headless.launch.py
#
# Starts the Gazebo SERVER ONLY (no 3D GUI window) for a given world.
# Mirrors clearpath_gz's own gz_sim.launch.py, minus the --gui-config,
# and adds Gazebo's '-s' (server-only) flag.
#
# Why: the Gazebo GUI's 3D rendering is the heaviest part of this VM's
# workload (software-rendered via LIBGL_ALWAYS_SOFTWARE=1). Running the
# server only speeds up simulation and controller-loading noticeably.
# Camera/lidar SENSORS still render internally (Gazebo's Sensors system
# needs a render context regardless of GUI), so LIBGL_ALWAYS_SOFTWARE=1
# is still required -- this only removes the separate 3D viewport window.
#
# Usage:
#   ros2 launch gz_sim_headless.launch.py world:=warehouse

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

    world = LaunchConfiguration('world')

    arguments = [
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='Gazebo World'),
    ]

    # Same resource path setup as clearpath_gz's own gz_sim.launch.py, so
    # the warehouse world / models resolve identically.
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH', '').split(':') if p]
    gz_sim_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(pkg_clearpath_gz, 'worlds'), ':' + ':'.join(packages_paths)],
    )

    # '-s' = server only (no GUI window). '-r' = start unpaused immediately.
    # '--headless-rendering' explicitly initializes the offscreen render
    # context that camera/depth sensors need -- without it, in server-only
    # mode, rendering-dependent sensors (RGB, depth cameras) can silently
    # produce no data at all while non-rendering sensors (IMU) work fine.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [world, '.sdf', ' -s -r --headless-rendering -v 4']),
        ]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    ld = LaunchDescription(arguments)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(gz_sim)
    ld.add_action(clock_bridge)
    return ld
