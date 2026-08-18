#!/usr/bin/env python3
# multi_robot_simulation.launch.py
#
# Starts a single Gazebo world (via clearpath_gz's gz_sim.launch.py) and then
# spawns three Husky A200s (r0, r1, r2) into it (via clearpath_gz's
# robot_spawn.launch.py), each reading its own robot.yaml from its own
# setup_path so the per-robot generators never write into the same folder.
#
# Usage:
#   ros2 launch multi_robot_simulation.launch.py
#   ros2 launch multi_robot_simulation.launch.py setup_root:=$HOME/phd_assignment_ws/clearpath world:=warehouse
#
# This file needs no custom ROS 2 package -- `ros2 launch` accepts a direct
# path to a launch file, e.g.:
#   ros2 launch /home/user/multi_robot_simulation.launch.py

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


# ---------------------------------------------------------------------------
# Per-robot layout. Adjust x/y/yaw if a robot spawns inside a shelf/wall in
# the chosen world -- open the Gazebo GUI once with only r0 running to find
# clear ground, then set the offsets for r1/r2 accordingly.
# ---------------------------------------------------------------------------
ROBOTS = [
    {'name': 'r0', 'x': '0.0', 'y': '0.0', 'yaw': '0.0'},
    {'name': 'r1', 'x': '2.0', 'y': '0.0', 'yaw': '0.0'},
    {'name': 'r2', 'x': '4.0', 'y': '0.0', 'yaw': '0.0'},
]

# Seconds to wait after requesting Gazebo start before spawning robots.
# Gazebo + world loading is not synchronized with the launch system here,
# so this is a fixed delay rather than an event-based handshake. Increase
# it if robots occasionally fail to spawn (e.g. "world not found") on a
# slower machine / VM.
SPAWN_DELAY_SEC = 8.0


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
    DeclareLaunchArgument('setup_root',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description=('Parent directory containing one subfolder per robot '
                                       '(setup_root/r0/robot.yaml, setup_root/r1/robot.yaml, ...)')),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz per robot.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'], description='use_sim_time'),
    DeclareLaunchArgument('generate', default_value='true',
                          choices=['true', 'false'],
                          description='Regenerate parameters/launch files for each robot.'),
]


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')

    gz_sim_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'gz_sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])

    setup_root = LaunchConfiguration('setup_root')
    world = LaunchConfiguration('world')

    ld = LaunchDescription(ARGUMENTS)

    # --- 1. Start Gazebo exactly once. ------------------------------------
    # setup_path here only affects the teleop-topic auto-config in the
    # Gazebo GUI plugin; point it at r0 so the GUI teleop panel defaults to
    # r0's cmd_vel. It has no other effect on the world itself.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('world', world),
            ('setup_path', PathJoinSubstitution([setup_root, 'r0/'])),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
        ]
    )
    ld.add_action(gz_sim)

    # --- 2. Spawn each robot after a delay so Gazebo is up. ---------------
    #
    # IMPORTANT: each robot_spawn.launch.py is run as its own OS process via
    # `ros2 launch` (ExecuteProcess), NOT as an IncludeLaunchDescription in
    # this same process. robot_spawn.launch.py chains
    # generate_description -> generate_semantic_description -> generate_launch
    # -> generate_param -> spawn using RegisterEventHandler(OnProcessExit),
    # which fire asynchronously, *outside* the synchronous push/pop scope
    # IncludeLaunchDescription uses for LaunchConfigurations. Running three
    # concurrent IncludeLaunchDescription copies of it in one launch process
    # causes the shared 'setup_path' LaunchConfiguration to leak between them
    # (all three end up reading whichever robot's config was set last).
    # Separate OS processes each get their own Python launch context, so
    # there is no shared state to leak.
    for i, robot in enumerate(ROBOTS):
        # Stagger the *generation* start slightly (not strictly required,
        # since each is now an independent process/context, but it keeps
        # the terminal output readable and avoids all three generators
        # hammering disk/CPU at the exact same instant on a VM).
        robot_delay = SPAWN_DELAY_SEC + i * 15.0

        robot_spawn_process = ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'clearpath_gz', 'robot_spawn.launch.py',
                ['setup_path:=', setup_root, '/', robot['name'], '/'],
                ['world:=', world],
                ['use_sim_time:=', LaunchConfiguration('use_sim_time')],
                ['rviz:=', LaunchConfiguration('rviz')],
                ['generate:=', LaunchConfiguration('generate')],
                'x:=' + robot['x'],
                'y:=' + robot['y'],
                'z:=0.3',
                'yaw:=' + robot['yaw'],
            ],
            output='screen',
            name=f"robot_spawn_{robot['name']}",
            shell=False,
        )

        delayed_spawn = TimerAction(
            period=robot_delay,
            actions=[robot_spawn_process],
        )
        ld.add_action(delayed_spawn)

    return ld
