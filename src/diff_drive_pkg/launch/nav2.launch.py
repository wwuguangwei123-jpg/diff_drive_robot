import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    pkg_dir = get_package_share_directory('diff_drive_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_cmd_vel_topic = LaunchConfiguration('nav2_cmd_vel_topic')

    map_file = os.path.join(pkg_dir, 'maps', 'my_room_map.yaml')

    nav2_launch = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst=nav2_cmd_vel_topic),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'true',
                }.items(),
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'nav2_cmd_vel_topic',
            default_value='/nav2_cmd_vel',
            description='Remap Nav2 controller output so the custom C++ tracker can own /cmd_vel.',
        ),
        nav2_launch,
    ])
