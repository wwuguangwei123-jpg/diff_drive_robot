from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('lookahead_distance', default_value='0.8'),
        DeclareLaunchArgument('target_speed', default_value='0.35'),
        DeclareLaunchArgument('min_tracking_speed', default_value='0.08'),
        DeclareLaunchArgument('max_tracking_speed', default_value='0.55'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.18'),
        DeclareLaunchArgument('obstacle_stop_distance', default_value='0.40'),
        DeclareLaunchArgument('obstacle_slow_distance', default_value='0.80'),
        Node(
            package='diff_drive_control_cpp',
            executable='traj_tracking_node',
            name='traj_tracking_node',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'lookahead_distance': float_param('lookahead_distance'),
                'target_speed': float_param('target_speed'),
                'min_tracking_speed': float_param('min_tracking_speed'),
                'max_tracking_speed': float_param('max_tracking_speed'),
                'goal_tolerance': float_param('goal_tolerance'),
                'obstacle_stop_distance': float_param('obstacle_stop_distance'),
                'obstacle_slow_distance': float_param('obstacle_slow_distance'),
                'cte_topic': '/cte',
            }],
        ),
    ])
