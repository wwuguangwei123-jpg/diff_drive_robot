# 🚗 Multi-Modal-Mobile-Robot-Control

基于 ROS 2 Humble 的麦克纳姆轮底盘 SLAM 建图、自主导航与高级 C++ 纯追踪（Pure Pursuit）+ 级联 PID 控制系统实现

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Language](https://img.shields.io/badge/Language-C%2B%2B%2014-orange)](https://en.cppreference.com/)
[![Framework](https://img.shields.io/badge/Framework-Nav2-green)](https://navigation.ros.org/)

## 1. 项目简介
本项目是在 Linux (Ubuntu 22.04) 环境下，基于 **ROS 2 Humble** 框架开发的高性能移动机器人控制与导航闭环系统。项目在 Gazebo 仿真环境中搭建了集成 2D 激光雷达与 IMU 传感器的麦克纳姆轮多模态移动底盘，**利用手写 C++ 控制节点劫持了 Navigation 2 的底层控制器接口**，实现了高精度的自主轨迹跟踪控制。该系统设计方案主要用于解决移动机器人在复杂室内场景下的精准路径滑移补偿与高频闭环追踪问题。

---

## 2. 核心技术亮点与突围痛点
* **手写 C++ 纯追踪与级联 PID 控制核心**：拒绝套用 Nav2 现成的局部规划器组件，基于经典几何与全向运动学约束纯手写高频控制节点 `traj_tracking_node`。结合手写的 **PID 控制器（PID Controller）** 动态调节解算误差，通过**动态前瞻距离（Lookahead Distance）匹配算法**，实时解算横向偏差并输出位置、速度平滑指令。
* **数据管道解耦与闭环监控**：在控制节点内部定制化设计了底层实时误差发布模块。通过向自定义话题发布横向跟踪误差（Cross Track Error, $CTE$），打通了与 `rqt_plot` 的数据流拓扑，使算法调用、响应曲线及控制残差具备工业级数据量化可视能力。
* **SLAM Toolbox 与 Cartographer 参数深度注入**：彻底解决了 ROS 2 节点名称动态映射导致 YAML 配置文件失效的隐蔽漏洞。利用**全局通配符（`/**:`）**对雷达物理极限数据进行无缝重构，拉长坐标树（TF）数据缓冲队列时间并扩容至 `message_queue_size: 100`，配合特定的 `.lua` 配置文件优化，彻底杜绝了 `queue is full` 雷达丢包现象，实现了大范围地图的零畸变构建[cite: 1, 2]。
* **动力学重构与状态空间解耦**：针对麦克纳姆轮全向滑移特性，通过在全局/局部代价地图（Costmap）中进行参数剪裁与异步时序控制，有效规避了导航初始阶段虚空边界导致的 Nav2 `Received map message is malformed` 地图崩溃异常，构建了稳定的全局闭环运动空间[cite: 1, 2]。

---

## 3. 系统拓扑与数据流架构

| 输入节点 / 硬件传感器 | 核心计算单元 / 节点 | 控制输出话题 / 反馈 | 量化性能指标 |
| :--- | :--- | :--- | :--- |
| `Lidar (/scan)` @10Hz<br>`Odom -> Base (/tf)` | `async_slam_toolbox_node`<br>(基于 Ceres 优化器解算)[cite: 1, 2] | `/map` (全局高精度占用网格地图)[cite: 1, 2] | 地图边界实时动态拓展<br>雷达数据丢包率：**0.0%**[cite: 1, 2] |
| `/plan` (Nav2 全局路径规划)<br>当前里程计位姿 `/odom`[cite: 1, 2] | `traj_tracking_node`<br>(C++ 纯手写纯追踪 + PID 控制大脑)[cite: 1, 2] | `/cmd_vel` (全向速度控制矢量)<br>`/cte/data` (横向追踪误差)[cite: 1, 2] | 控制响应周期：**< 20ms**<br>控制收敛残差 $CTE$：**< ±0.02m**[cite: 1, 2] |

---

## 4. 项目文件结构 (Multi-Package Architecture)

本项目采用工业级“算法层-物理层”彻底解耦的双工作包（Dual-Package）架构开发，严格规避了依赖污染：

```text
ros2_ws/src/
├── diff_drive_pkg/                  # 【物理与环境基建包】
│   ├── CMakeLists.txt              # 资源、地图与模型的安装映射定义
│   ├── package.xml                 # 声明 gazebo_ros, robot_state_publisher 等物理层依赖
│   ├── config/
│   │   ├── slam_params.yaml        # 采用通配符全局注入的 SLAM Toolbox 参数配置文件
│   │   └── cartographer.lua        # 用于 Cartographer 建图优化的底层后端 Lua 配置文件
│   ├── launch/
│   │   └── diff_drive_launch.py    # 整合全局时间同步、Gazebo物理引擎、TF树与RViz2的核心Launch
│   ├── maps/
│   │   ├── classroom_map.yaml      # 已构建完毕的静态环境占用网格描述文件
│   │   └── classroom_map.pgm       # 静态环境栅格地图二进制图像数据
│   ├── rviz/
│   │   └── diff_drive.rviz         # 预配置的雷达、地图、Costmap、全局/局部路径渲染视图文件
│   └── urdf/
│       └── diff_drive_gazebo.urdf.xacro # 全向麦克纳姆轮底盘及多传感器集成的物理描述文件
│
└── diff_drive_control_cpp/          # 【核心控制算法包】
    ├── CMakeLists.txt              # C++ 编译目标与依赖声明 (nav2_msgs, rclcpp, std_msgs)
    ├── package.xml                 # 声明算法所需的通信接口依赖
    ├── include/
    │   ├── pid_controller.hpp      # 级联 PID 控制算法类定义（支持积分抗饱和、输出限幅）
    │   └── traj_tracking_node.hpp  # 纯追踪控制核心及 CTE 发布器的头文件声明
    └── src/
        ├── pid_controller.cpp      # PID 误差迭代、比例/积分/微分各项逻辑实现源码
        └── traj_tracking_node.cpp  # 纯手写 C++ 轨迹跟踪算法及 O(N) 横向误差解算核心源码

5. 快速复现与部署运行说明

5.1 依赖准备与底层补丁修复
为防止 ROS 2 环境中因 Python 环境依赖冲突引发底层 C 接口断裂导致 rqt_plot 运行崩溃，必须在部署前执行以下依赖重构[cite: 1, 2]：

Bash
# 安装可视化套件与核心绘图后端
sudo apt update
sudo apt install ros-humble-rqt ros-humble-rqt-plot ros-humble-rqt-graph python3-pyqtgraph -y

# 强行降级 Python 依赖库以修正编译期底层冲突
pip3 install "numpy<2" --force-reinstall

5.2 编译与环境空间注入
Bash:
cd ~/ros2_ws/
colcon build --packages-select diff_drive_pkg diff_drive_control_cpp
source install/setup.bash

5.3 闭环系统分步启动流
为确保底层 DDS 通信拓扑完全建立与局部代价地图（Costmap）的安全加载，请严格按照以下时序在独立终端中拉起系统[cite: 1, 2]：

终端 1 - 物理仿真世界：

Bash
   ros2 launch diff_drive_pkg diff_drive_launch.py
终端 2 - 注入核心配置的建图节点（进行实时建图时启动）：

Bash
   ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$(pwd)/src/diff_drive_pkg/config/slam_params.yaml use_sim_time:=true
终端 3 - 自主导航服务：

Bash
   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true cmd_vel:=/nav2_dummy_cmd_vel
终端 4 - 启动 C++ 算法大脑：

Bash
   ros2 run diff_drive_control_cpp traj_tracking_node --ros-args -p use_sim_time:=true
终端 5 - 运行实时控制残差波形分析：

Bash
   ros2 run rqt_plot rqt_plot
   运行后在弹出的图形化面板顶部输入框中键入 /cte/data，点击右侧 + 号按钮即可实时监控误差曲线
6. 实验与算法鲁棒性评测 (Experimental Results)
在 Gazebo 仿真环境下，通过 RViz 下发包含大角度 S 型转弯、直角转弯及连续障碍物规避的复合路径[cite: 1, 2]。
在小车最大线速度配置为 0.5 m/s 的测试工况下，实时拉取 rqt_plot 反馈的误差曲线[cite: 1, 2]：直线段跟踪工况：基于 C++ 纯追踪核心与底层 PID 控制器的协同调节，
跟踪误差 CTE 完美收敛于 0.00 m 轴线附近，系统无任何静态残差[cite: 1, 2]。大曲率弯道突变工况：在进入急弯瞬间，由于动量导致的滑移使 $CTE$ 出现短暂阶跃，
瞬态极值成功控制在 0.12m 以内[cite: 1, 2]。控制算法通过动态前瞻限幅机制迅速响应，并在 1.5 秒内 将位置偏差平滑且无振荡地重新收敛至 0.02m 以下的极高精度区间[cite: 1, 2]。
实验数据充分证明了该双包解耦控制系统及手写底层算法在复杂大曲率工况下的优异性能与强鲁棒性[cite: 1, 2]
