#!/usr/bin/env python3
# rtabmap_r0.launch.py
#
# RTAB-Map SLAM for a single robot (r0), using the RGB-D camera (camera_0)
# as the primary sensor and the 2D lidar (lidar2d_0) as a supplementary
# scan input for more robust registration in feature-poor areas of the
# warehouse world.
#
# Prerequisite: r0 must already be spawned and driving (multi_robot_simulation
# .launch.py), with platform_velocity_controller active (odom being published)
# and the ekf_node broadcasting the odom -> base_link tf.
#
# Usage:
#   ros2 launch rtabmap_r0.launch.py
#   ros2 launch rtabmap_r0.launch.py namespace:=r1   # reuse for other robots

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    arguments = [
        DeclareLaunchArgument('namespace', default_value='r0',
                              description='Robot namespace (must match an already-running robot).'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('localization', default_value='false',
                              choices=['true', 'false'],
                              description=('false: mapping mode (build/update the map). '
                                           'true: localization-only mode against an existing map.')),
    ]

    # Topics as published under this robot's namespace (relative names --
    # the Node namespace=... prefix below resolves these to /<namespace>/...).
    rgb_topic = 'sensors/camera_0/color/image'
    rgb_info_topic = 'sensors/camera_0/color/camera_info'
    depth_topic = 'sensors/camera_0/depth/image'
    scan_topic = 'sensors/lidar2d_0/scan'
    odom_topic = 'platform/odom'

    shared_remappings = [
        ('rgb/image', rgb_topic),
        ('rgb/camera_info', rgb_info_topic),
        ('depth/image', depth_topic),
        ('scan', scan_topic),
        ('odom', odom_topic),
        # CRITICAL: tf2_ros's TransformListener always subscribes to the
        # ABSOLUTE topics '/tf' and '/tf_static' by default, regardless of
        # the node's namespace -- it does NOT automatically pick up
        # '/r0/tf'. Without this explicit remap, rtabmap/rtabmap_viz never
        # see this robot's tf tree at all and every lookupTransform() call
        # fails with "source_frame does not exist", even though the frame
        # genuinely exists on /r0/tf. (Same gotcha we hit needing
        # `-r /tf:=/r0/tf` for `ros2 run tf2_tools view_frames` earlier.)
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    # 1. Synchronize RGB + depth + camera_info into a single RGBDImage msg.
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        namespace=namespace,
        output='screen',
        parameters=[{
            'approx_sync': True,
            'use_sim_time': use_sim_time,
            'approx_sync_max_interval': 0.05,
        }],
        remappings=shared_remappings,
    )

    # 2. RTAB-Map SLAM node itself.
    rtabmap_parameters = [{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': False,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'approx_sync': True,
        'use_sim_time': use_sim_time,
        'Reg/Strategy': '1',        # 0=Visual, 1=ICP, 2=Visual+ICP -- use combined for robustness
        'Reg/Force3DoF': 'true',    # ground robot: constrain registration to 3DoF (x, y, yaw)
        'Grid/RangeMax': '8.0',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'Optimizer/GravitySigma': '0',
        'database_path': '/tmp/rtabmap_r0.db',
        # Suppresses a cosmetic warning: at this VM's ~4 Hz camera rate,
        # rgb/depth timestamps can differ by up to ~30 ms, which is small
        # relative to the ~250 ms frame period but still triggers rgbd_sync's
        # default (tight) sync-quality warning. Loosening the tolerance here
        # doesn't change correctness, just stops it from spamming the log.
        'approx_sync_max_interval': '0.05',
    }]

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace=namespace,
        output='screen',
        parameters=rtabmap_parameters,
        remappings=shared_remappings + [('rgbd_image', 'rgbd_image')],
        arguments=['-d'] if False else [],  # pass ['-d'] manually if you want to wipe the DB each run
    )

    # 3. Optional live visualization (2D/3D map + graph). Comment out if
    # running headless / over a slow remote connection.
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace=namespace,
        output='screen',
        parameters=rtabmap_parameters,
        remappings=shared_remappings,
    )

    return LaunchDescription(arguments + [rgbd_sync, rtabmap, rtabmap_viz])
