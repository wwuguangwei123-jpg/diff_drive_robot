#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class GoalToPlanNode : public rclcpp::Node {
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    GoalToPlanNode()
        : Node("goal_to_plan_node"),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_) {
        goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
        plan_topic_ = declare_parameter<std::string>("plan_topic", "/plan");
        publish_result_path_ = declare_parameter<bool>("publish_result_path", false);
        planner_action_ = declare_parameter<std::string>("planner_action", "/compute_path_to_pose");
        global_frame_ = declare_parameter<std::string>("global_frame", "map");
        robot_frame_ = declare_parameter<std::string>("robot_frame", "base_footprint");
        planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");

        if (publish_result_path_) {
            plan_pub_ = create_publisher<nav_msgs::msg::Path>(plan_topic_, 10);
        }
        planner_client_ = rclcpp_action::create_client<ComputePathToPose>(this, planner_action_);
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic_, 10,
            std::bind(&GoalToPlanNode::goal_callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Goal planner bridge ready: goal=%s action=%s plan=%s",
            goal_topic_.c_str(),
            planner_action_.c_str(),
            plan_topic_.c_str());
    }

private:
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {
        if (!planner_client_->wait_for_action_server(2s)) {
            RCLCPP_WARN(get_logger(), "Planner action %s is not available yet.", planner_action_.c_str());
            return;
        }

        ComputePathToPose::Goal goal;
        goal.goal = *goal_msg;
        goal.goal.header.frame_id = goal.goal.header.frame_id.empty() ? global_frame_ : goal.goal.header.frame_id;
        goal.planner_id = planner_id_;
        goal.use_start = true;

        try {
            const auto transform = tf_buffer_.lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero, 200ms);
            goal.start.header.frame_id = global_frame_;
            goal.start.header.stamp = now();
            goal.start.pose.position.x = transform.transform.translation.x;
            goal.start.pose.position.y = transform.transform.translation.y;
            goal.start.pose.position.z = transform.transform.translation.z;
            goal.start.pose.orientation = transform.transform.rotation;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Cannot create plan start from TF %s->%s: %s", global_frame_.c_str(), robot_frame_.c_str(), ex.what());
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Planning from (%.2f, %.2f) to (%.2f, %.2f).",
            goal.start.pose.position.x,
            goal.start.pose.position.y,
            goal.goal.pose.position.x,
            goal.goal.pose.position.y);

        rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
        options.result_callback = [this](const GoalHandleComputePath::WrappedResult &result) {
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
                RCLCPP_WARN(get_logger(), "Planner action did not succeed.");
                return;
            }
            if (result.result->path.poses.empty()) {
                RCLCPP_WARN(get_logger(), "Planner returned an empty path.");
                return;
            }
            if (publish_result_path_ && plan_pub_) {
                plan_pub_->publish(result.result->path);
            }
            RCLCPP_INFO(get_logger(), "Planner produced path with %zu poses.", result.result->path.poses.size());
        };

        planner_client_->async_send_goal(goal, options);
    }

    std::string goal_topic_;
    std::string plan_topic_;
    std::string planner_action_;
    std::string global_frame_;
    std::string robot_frame_;
    std::string planner_id_;
    bool publish_result_path_ = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr planner_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalToPlanNode>());
    rclcpp::shutdown();
    return 0;
}
