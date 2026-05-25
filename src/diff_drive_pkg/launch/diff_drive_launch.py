import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
import xacro

def generate_launch_description():
    pkg_name = 'diff_drive_pkg'
    pkg_share = get_package_share_directory(pkg_name)
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'diff_drive_gazebo.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # 全局强制时间同步
    global_sim_time = SetParameter(name='use_sim_time', value=True)

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description] 
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'diff_bot', '-x', '0.0', '-y', '0.0', '-z', '0.05'],
        output='screen'
    )

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'diff_drive.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_stf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )

    # ========================================================================
    # 🌟 强行注入 SLAM：使用绝对路径读取 yaml，无视编译环境映射问题！
    # ========================================================================
    slam_yaml_path = os.path.expanduser('~/ros2_ws/src/diff_drive_pkg/config/slam_params.yaml')
    
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_yaml_path  # 强制灌入我们的定制参数
        }.items()
    )

    return LaunchDescription([
        global_sim_time, 
        gazebo,
        rsp_node,
        spawn_entity,
        rviz_node,
        static_tf,
        slam_toolbox_node  # 启动世界的同时直接拉起建图
    ])
