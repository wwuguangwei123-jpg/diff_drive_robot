#include "rclcpp/rclcpp.hpp"
#include "diff_drive_control_cpp/traj_tracking_node.hpp"
#include <csignal>
#include <memory>
#include <thread>

// 全局指针，用于生命周期中断拦截
std::shared_ptr<TrajTrackingNode> g_node = nullptr;

void sigint_handler(int sig) {
    (void)sig;
    if (g_node) {
        g_node->stop_robot(); // 触发底盘安全停机
    }
    rclcpp::shutdown();
}

int main(int argc, char ** argv) {
    // 拦截 ROS 2 默认退出时序，改用系统级底层平滑降级方案
    rclcpp::InitOptions options;
    options.shutdown_on_signal = false; 
    rclcpp::init(argc, argv, options);

    g_node = std::make_shared<TrajTrackingNode>();
    std::signal(SIGINT, sigint_handler);

    rclcpp::spin(g_node);
    return 0;
}