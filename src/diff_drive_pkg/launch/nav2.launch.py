import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('diff_drive_pkg')
    map_file = os.path.join(pkg_dir, 'maps', 'my_room_map.yaml')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    common_params = [
        params_file,
        {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {
                    'yaml_filename': map_file,
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                },
            ],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=common_params,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=common_params,
        ),
        Node(
            package='diff_drive_control_cpp',
            executable='goal_to_plan_node',
            name='goal_to_plan_node',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'goal_topic': '/goal_pose',
                'plan_topic': '/plan',
                'global_frame': 'map',
                'robot_frame': 'base_footprint',
                'planner_action': '/compute_path_to_pose',
                'publish_result_path': False,
                'planner_id': 'GridBased',
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planning',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': True,
                'node_names': ['map_server', 'amcl', 'planner_server'],
            }],
        ),
    ])
