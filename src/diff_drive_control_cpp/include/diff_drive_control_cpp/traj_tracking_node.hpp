#ifndef DIFF_DRIVE_CONTROL_CPP__TRAJ_TRACKING_NODE_HPP_
#define DIFF_DRIVE_CONTROL_CPP__TRAJ_TRACKING_NODE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "diff_drive_control_cpp/pid_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"

enum class RobotState {
    GO_TO_GOAL,
    AVOID_OBSTACLE,
    REACHED_GOAL
};

class TrajTrackingNode : public rclcpp::Node {
public:
    TrajTrackingNode();
    void stop_robot();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void control_loop();

    bool find_lookahead_point(double &target_x, double &target_y);
    double get_min_range(
        const sensor_msgs::msg::LaserScan::SharedPtr msg,
        double min_angle,
        double max_angle) const;
    void publish_cte();
    void load_parameters();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cte_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::pair<double, double>> global_path_;
    size_t current_path_index_ = 0;

    RobotState current_state_ = RobotState::GO_TO_GOAL;
    PidController linear_pid_;
    PidController angular_pid_;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double current_linear_speed_ = 0.0;
    double last_linear_x_ = 0.0;

    double min_dist_front_ = 10.0;
    double min_dist_left_ = 10.0;
    double min_dist_right_ = 10.0;

    double lookahead_distance_ = 0.8;
    double target_speed_ = 0.35;
    double min_tracking_speed_ = 0.08;
    double max_tracking_speed_ = 0.55;
    double goal_tolerance_ = 0.18;
    double obstacle_stop_distance_ = 0.40;
    double obstacle_slow_distance_ = 0.80;
    double max_accel_ = 0.5;
    double max_decel_ = 0.8;
    double angular_pid_weight_ = 0.35;
    double control_period_s_ = 0.05;

    std::string odom_topic_ = "/odom";
    std::string scan_topic_ = "/scan";
    std::string plan_topic_ = "/plan";
    std::string cmd_vel_topic_ = "/cmd_vel";
    std::string cte_topic_ = "/cte";
};

#endif  // DIFF_DRIVE_CONTROL_CPP__TRAJ_TRACKING_NODE_HPP_
