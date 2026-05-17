import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('diff_drive_pkg')
    
    # 1. Cartographer 核心节点 (负责算力)
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}], # 🌟 强制同步时间
        arguments=[
            '-configuration_directory', os.path.join(pkg_dir, 'config'),
            '-configuration_basename', 'cartographer_2d.lua'
        ]
    )

    # 2. 栅格地图发布节点 (负责把内部数据转成 RViz 能看的二维地图)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'resolution': 0.05} # 地图分辨率：一格 5cm
        ]
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
    ])
