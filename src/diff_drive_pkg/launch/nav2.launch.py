import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('diff_drive_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_dir, 'maps', 'my_room_map.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
