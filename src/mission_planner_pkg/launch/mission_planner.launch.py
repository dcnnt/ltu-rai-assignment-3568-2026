import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('mission_planner_pkg'),
        'config',
        'mission_config.yaml',
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the YAML file listing robot namespaces, '
                     'waypoints and timing parameters.',
    )

    node = Node(
        package='mission_planner_pkg',
        executable='mission_planner_node',
        name='mission_planner_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([config_arg, node])
