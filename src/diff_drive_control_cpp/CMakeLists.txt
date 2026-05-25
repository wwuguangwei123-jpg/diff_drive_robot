cmake_minimum_required(VERSION 3.8)
project(diff_drive_control_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找 ROS 2 核心通信依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. 包含自定义头文件目录
include_directories(include)

# 3. 添加轨迹跟踪节点可执行文件 (联合编译所有 src 下的 cpp)
add_executable(traj_tracking_node
  src/main.cpp
  src/pid_controller.cpp
  src/traj_tracking_node.cpp
)

# 4. 链接 ROS 2 动态库
ament_target_dependencies(traj_tracking_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  sensor_msgs
)

# 5. 安装节点到系统路径
install(TARGETS
  traj_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
