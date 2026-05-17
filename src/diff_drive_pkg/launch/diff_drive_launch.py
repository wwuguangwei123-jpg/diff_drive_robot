import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, SetParameter
import xacro

def generate_launch_description():
    pkg_name = 'diff_drive_pkg'
    pkg_share = get_package_share_directory(pkg_name)
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'diff_drive_gazebo.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # 🌟 核心：全局强制注入 use_sim_time = true
    # 这样 Gazebo、RViz 和 Robot State Publisher 的时间就完全绑定了
    global_sim_time = SetParameter(name='use_sim_time', value=True)

    # 1. 启动 Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    # 2. 启动 Robot State Publisher (翻译官)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 3. 放入小车实体
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'diff_bot'],
        output='screen'
    )

    # 4. 启动 RViz2
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'diff_drive.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # 返回的列表里【没有】控制节点，这样小车生成后会静止等待你的命令
    return LaunchDescription([
        global_sim_time, 
        gazebo,
        rsp_node,
        spawn_entity,
        rviz_node
    ])
