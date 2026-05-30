#include "diff_drive_control_cpp/traj_tracking_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}
}  // namespace

TrajTrackingNode::TrajTrackingNode()
    : Node("traj_tracking_node"),
      linear_pid_(0.8, 0.0, 0.05, 0.6, -0.8),
      angular_pid_(1.4, 0.0, 0.05, 1.2, -1.2),
      tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_) {
    load_parameters();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&TrajTrackingNode::odom_callback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10, std::bind(&TrajTrackingNode::scan_callback, this, std::placeholders::_1));
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
        plan_topic_, 10, std::bind(&TrajTrackingNode::plan_callback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    cte_pub_ = create_publisher<std_msgs::msg::Float64>(cte_topic_, 10);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(control_period_s_),
        std::bind(&TrajTrackingNode::control_loop, this));

    RCLCPP_INFO(
        get_logger(),
        "Trajectory tracker ready: plan=%s cmd_vel=%s cte=%s",
        plan_topic_.c_str(),
        cmd_vel_topic_.c_str(),
        cte_topic_.c_str());
}

void TrajTrackingNode::load_parameters() {
    odom_topic_ = declare_parameter<std::string>("odom_topic", odom_topic_);
    scan_topic_ = declare_parameter<std::string>("scan_topic", scan_topic_);
    plan_topic_ = declare_parameter<std::string>("plan_topic", plan_topic_);
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", cmd_vel_topic_);
    cte_topic_ = declare_parameter<std::string>("cte_topic", cte_topic_);

    lookahead_distance_ = declare_parameter<double>("lookahead_distance", lookahead_distance_);
    target_speed_ = declare_parameter<double>("target_speed", target_speed_);
    min_tracking_speed_ = declare_parameter<double>("min_tracking_speed", min_tracking_speed_);
    max_tracking_speed_ = declare_parameter<double>("max_tracking_speed", max_tracking_speed_);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", goal_tolerance_);
    obstacle_stop_distance_ = declare_parameter<double>("obstacle_stop_distance", obstacle_stop_distance_);
    obstacle_slow_distance_ = declare_parameter<double>("obstacle_slow_distance", obstacle_slow_distance_);
    max_accel_ = declare_parameter<double>("max_accel", max_accel_);
    max_decel_ = declare_parameter<double>("max_decel", max_decel_);
    angular_pid_weight_ = declare_parameter<double>("angular_pid_weight", angular_pid_weight_);
    control_period_s_ = declare_parameter<double>("control_period", control_period_s_);

    linear_pid_.setGains(
        declare_parameter<double>("linear_kp", 0.8),
        declare_parameter<double>("linear_ki", 0.0),
        declare_parameter<double>("linear_kd", 0.05));
    angular_pid_.setGains(
        declare_parameter<double>("angular_kp", 1.4),
        declare_parameter<double>("angular_ki", 0.0),
        declare_parameter<double>("angular_kd", 0.05));

    lookahead_distance_ = std::max(0.1, lookahead_distance_);
    control_period_s_ = std::max(0.01, control_period_s_);
    max_tracking_speed_ = std::max(min_tracking_speed_, max_tracking_speed_);
}

void TrajTrackingNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_linear_speed_ = msg->twist.twist.linear.x;
    odom_frame_ = msg->header.frame_id.empty() ? "odom" : msg->header.frame_id;

    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;

    geometry_msgs::msg::PoseStamped pose_in_path_frame = odom_pose;
    if (!path_frame_.empty() && path_frame_ != odom_frame_) {
        try {
            pose_in_path_frame = tf_buffer_.transform(
                odom_pose,
                path_frame_,
                tf2::durationFromSec(0.05));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Could not transform odom pose from %s to %s: %s",
                odom_frame_.c_str(),
                path_frame_.c_str(),
                ex.what());
        }
    }

    current_x_ = pose_in_path_frame.pose.position.x;
    current_y_ = pose_in_path_frame.pose.position.y;
    current_yaw_ = tf2::getYaw(pose_in_path_frame.pose.orientation);
}

void TrajTrackingNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (current_state_ == RobotState::REACHED_GOAL || global_path_.empty()) {
        return;
    }

    min_dist_front_ = get_min_range(msg, -0.35, 0.35);
    min_dist_left_ = get_min_range(msg, 0.35, 1.20);
    min_dist_right_ = get_min_range(msg, -1.20, -0.35);

    if (min_dist_front_ < obstacle_slow_distance_) {
        if (current_state_ != RobotState::AVOID_OBSTACLE) {
            RCLCPP_WARN(
                get_logger(),
                "Obstacle ahead at %.2f m, switching to avoidance.",
                min_dist_front_);
            current_state_ = RobotState::AVOID_OBSTACLE;
            linear_pid_.reset();
            angular_pid_.reset();
        }
    } else if (current_state_ == RobotState::AVOID_OBSTACLE) {
        RCLCPP_INFO(get_logger(), "Front sector clear, resuming path tracking.");
        current_state_ = RobotState::GO_TO_GOAL;
        linear_pid_.reset();
        angular_pid_.reset();
    }
}

double TrajTrackingNode::get_min_range(
    const sensor_msgs::msg::LaserScan::SharedPtr msg,
    double min_angle,
    double max_angle) const {
    if (msg->ranges.empty() || msg->angle_increment == 0.0) {
        return std::numeric_limits<double>::infinity();
    }

    double min_range = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
        const float range = msg->ranges[i];
        if (angle >= min_angle && angle <= max_angle && std::isfinite(range)) {
            if (range >= msg->range_min && range <= msg->range_max) {
                min_range = std::min(min_range, static_cast<double>(range));
            }
        }
    }
    return min_range;
}

void TrajTrackingNode::control_loop() {
    geometry_msgs::msg::Twist cmd_msg;

    if (global_path_.empty()) {
        last_linear_x_ = 0.0;
        cmd_pub_->publish(cmd_msg);
        return;
    }

    if (current_state_ == RobotState::REACHED_GOAL) {
        last_linear_x_ = 0.0;
        cmd_pub_->publish(cmd_msg);
        return;
    }

    if (current_state_ == RobotState::AVOID_OBSTACLE) {
        if (min_dist_front_ < obstacle_stop_distance_) {
            cmd_msg.linear.x = -0.12;
            cmd_msg.angular.z = (min_dist_left_ > min_dist_right_) ? 1.0 : -1.0;
        } else {
            cmd_msg.linear.x = std::min(0.18, target_speed_);
            const double turn_gain = std::clamp(
                0.8 + (obstacle_slow_distance_ - min_dist_front_) * 1.2,
                0.6,
                1.4);
            cmd_msg.angular.z = (min_dist_left_ > min_dist_right_) ? turn_gain : -turn_gain;
        }
    } else if (current_state_ == RobotState::GO_TO_GOAL) {
        publish_cte();

        const auto &goal = global_path_.back();
        const double dist_to_goal = std::hypot(goal.first - current_x_, goal.second - current_y_);
        if (dist_to_goal < goal_tolerance_) {
            current_state_ = RobotState::REACHED_GOAL;
            last_linear_x_ = 0.0;
            linear_pid_.reset();
            angular_pid_.reset();
            RCLCPP_INFO(get_logger(), "Goal reached within %.2f m.", dist_to_goal);
            cmd_pub_->publish(cmd_msg);
            return;
        }

        double target_x = 0.0;
        double target_y = 0.0;
        if (!find_lookahead_point(target_x, target_y)) {
            target_x = goal.first;
            target_y = goal.second;
        }

        const double dx = target_x - current_x_;
        const double dy = target_y - current_y_;
        const double alpha = normalize_angle(std::atan2(dy, dx) - current_yaw_);
        const double curvature = 2.0 * std::sin(alpha) / lookahead_distance_;

        const double curve_speed = target_speed_ / (1.0 + 1.8 * std::abs(curvature));
        const double goal_scale = std::clamp(dist_to_goal / 1.5, 0.25, 1.0);
        const double desired_speed = std::clamp(
            curve_speed * goal_scale,
            min_tracking_speed_,
            max_tracking_speed_);

        const double accel_cmd = linear_pid_.calculate(desired_speed - current_linear_speed_, control_period_s_);
        cmd_msg.linear.x = current_linear_speed_ + accel_cmd * control_period_s_;

        const double pure_pursuit_omega = cmd_msg.linear.x * curvature;
        const double heading_pid = angular_pid_.calculate(alpha, control_period_s_);
        cmd_msg.angular.z = std::clamp(
            pure_pursuit_omega + angular_pid_weight_ * heading_pid,
            -1.5,
            1.5);
    }

    const double max_delta_up = max_accel_ * control_period_s_;
    const double max_delta_down = max_decel_ * control_period_s_;
    if (cmd_msg.linear.x > last_linear_x_) {
        cmd_msg.linear.x = std::min(cmd_msg.linear.x, last_linear_x_ + max_delta_up);
    } else {
        cmd_msg.linear.x = std::max(cmd_msg.linear.x, last_linear_x_ - max_delta_down);
    }
    last_linear_x_ = cmd_msg.linear.x;

    cmd_pub_->publish(cmd_msg);
}

void TrajTrackingNode::publish_cte() {
    double min_cte = std::numeric_limits<double>::infinity();
    for (const auto &point : global_path_) {
        min_cte = std::min(min_cte, std::hypot(point.first - current_x_, point.second - current_y_));
    }

    if (std::isfinite(min_cte)) {
        std_msgs::msg::Float64 msg;
        msg.data = min_cte;
        cte_pub_->publish(msg);
    }
}

bool TrajTrackingNode::find_lookahead_point(double &target_x, double &target_y) {
    if (global_path_.empty()) {
        return false;
    }

    const size_t start = std::min(current_path_index_, global_path_.size() - 1);
    for (size_t i = start; i < global_path_.size(); ++i) {
        const double dist = std::hypot(global_path_[i].first - current_x_, global_path_[i].second - current_y_);
        if (dist >= lookahead_distance_) {
            target_x = global_path_[i].first;
            target_y = global_path_[i].second;
            current_path_index_ = i;
            return true;
        }
    }

    return false;
}

void TrajTrackingNode::plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        RCLCPP_WARN(get_logger(), "Received an empty path.");
        return;
    }

    global_path_.clear();
    global_path_.reserve(msg->poses.size());
    path_frame_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    for (const auto &pose_stamped : msg->poses) {
        global_path_.push_back({
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
        });
    }

    current_path_index_ = 0;
    current_state_ = RobotState::GO_TO_GOAL;
    linear_pid_.reset();
    angular_pid_.reset();
    RCLCPP_INFO(get_logger(), "Received path with %zu poses.", global_path_.size());
}

void TrajTrackingNode::stop_robot() {
    geometry_msgs::msg::Twist cmd_msg;
    cmd_pub_->publish(cmd_msg);
    RCLCPP_INFO(get_logger(), "Published zero velocity command.");
}
