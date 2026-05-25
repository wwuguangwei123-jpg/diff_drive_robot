#ifndef DIFF_DRIVE_CONTROL_CPP__TRAJ_TRACKING_NODE_HPP_
#define DIFF_DRIVE_CONTROL_CPP__TRAJ_TRACKING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"  // 🌟 新增：ROS 2 标准路径消息头文件
#include "diff_drive_control_cpp/pid_controller.hpp"
#include <vector>
#include "std_msgs/msg/float64.hpp"


enum class RobotState {
    GO_TO_GOAL,
    AVOID_OBSTACLE,
    REACHED_GOAL
};

class TrajTrackingNode : public rclcpp::Node {
public:
    TrajTrackingNode();
    void stop_robot(); // 紧急刹车接口

private:
    // 回调函数
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void control_loop();
    // 🌟 纯追踪算法核心变量
    std::vector<std::pair<double, double>> global_path_; // 存储我们要跟踪的全局路径点
    size_t current_path_index_ = 0;                      // 记录当前追踪到路径的哪一个点了
    
    double lookahead_distance_ = 0.8;                    // 核心参数：前瞻距离 L_d
    double target_speed_ = 0.3;                          // 核心参数：恒定巡航线速度 v

    // 🌟 纯追踪算法核心函数
    bool find_lookahead_point(double &target_x, double &target_y);
    // 🌟 新增：路径接收回调与订阅器
    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;

    // 辅助函数
    double get_min_range(const std::vector<float>& ranges, int start_idx, int end_idx);

    // ROS 2 通信接口
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cte_pub_;

    // 状态与数据变量
    RobotState current_state_;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double min_dist_front_ = 10.0;
    double min_dist_left_ = 10.0;
    double min_dist_right_ = 10.0;
    // 🌟 新增：用于记录上一帧的线速度，实现加速度限幅
    double last_linear_x_ = 0.0;
    // 🌟 核心升级：引入我们自己的数学 PID 控制器对象（纵向控制与横向控制）
    PidController linear_pid_;
    PidController angular_pid_;
};

#endif // DIFF_DRIVE_CONTROL_CPP__TRAJ_TRACKING_NODE_HPP_
