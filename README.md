# Differential Drive Robot Simulation

ROS 2 Humble workspace for a differential-drive mobile robot in Gazebo. The project includes:

- Gazebo robot model with differential-drive and 2D lidar plugins
- Indoor obstacle world for repeatable simulation
- SLAM Toolbox and Cartographer launch files
- Nav2 bringup against the saved `my_room_map` map
- C++ pure-pursuit path tracker with simple lidar obstacle avoidance and PID-assisted velocity control

The current robot is a differential-drive platform, not a mecanum or omnidirectional chassis.

## Repository layout

```text
src/
├── diff_drive_pkg/
│   ├── config/          # SLAM / Cartographer parameters
│   ├── launch/          # Gazebo, Nav2, Cartographer and tracking launch files
│   ├── maps/            # Saved occupancy map
│   ├── rviz/            # RViz configuration
│   ├── urdf/            # Robot and Gazebo xacro files
│   └── worlds/          # Gazebo indoor obstacle world
└── diff_drive_control_cpp/
    ├── include/
    └── src/             # traj_tracking_node and PID implementation
```

## Requirements

Target platform:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic with `gazebo_ros`

Install common dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2 \
  ros-humble-slam-toolbox \
  ros-humble-cartographer-ros \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-tf2-ros \
  ros-humble-rqt-plot
```

## Build

Clone this repository as a ROS 2 workspace or put it under an existing workspace:

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run simulation and SLAM

This starts Gazebo with the included indoor obstacle world, spawns the robot, opens RViz, and starts SLAM Toolbox:

```bash
ros2 launch diff_drive_pkg diff_drive_launch.py
```

Useful launch arguments:

```bash
ros2 launch diff_drive_pkg diff_drive_launch.py rviz:=false
ros2 launch diff_drive_pkg diff_drive_launch.py slam:=false
ros2 launch diff_drive_pkg diff_drive_launch.py world:=/absolute/path/to/your.world
```

## Run the C++ tracker

In another terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch diff_drive_pkg tracking.launch.py
```

The tracker subscribes to:

- `/odom`
- `/scan`
- `/plan`

It publishes:

- `/cmd_vel`
- `/cte`

Plot tracking error:

```bash
ros2 run rqt_plot rqt_plot /cte/data
```

## Nav2 planning with saved map

The saved map uses a relative image path, so it works after installation and on different machines. The Nav2 launch starts map server, AMCL, planner server and a small `goal_to_plan_node`. RViz `/goal_pose` messages are converted into Nav2 `ComputePathToPose` action requests; the resulting `/plan` is consumed by the custom C++ tracker, which is the only node that publishes `/cmd_vel`.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch diff_drive_pkg nav2.launch.py
```

For a full navigation test, run Gazebo/RViz, Nav2 planning and the tracker in separate terminals, then use RViz 2D Goal:

```bash
ros2 launch diff_drive_pkg diff_drive_launch.py slam:=false
ros2 launch diff_drive_pkg nav2.launch.py
ros2 launch diff_drive_pkg tracking.launch.py
```

## Controller parameters

The trajectory tracker exposes these ROS parameters:

- `lookahead_distance`
- `target_speed`
- `min_tracking_speed`
- `max_tracking_speed`
- `goal_tolerance`
- `obstacle_stop_distance`
- `obstacle_slow_distance`
- `linear_kp`, `linear_ki`, `linear_kd`
- `angular_kp`, `angular_ki`, `angular_kd`
- `odom_topic`, `scan_topic`, `plan_topic`, `cmd_vel_topic`, `cte_topic`

Example:

```bash
ros2 launch diff_drive_pkg tracking.launch.py target_speed:=0.25 lookahead_distance:=0.6
```

## Notes

- The project does not currently include measured benchmark data or rosbag-based experiment reports.
- The README intentionally describes only features that exist in the repository.
- The Python demo nodes under `diff_drive_pkg/diff_drive_pkg` are kept as simple examples; the main controller is `diff_drive_control_cpp/traj_tracking_node`.
