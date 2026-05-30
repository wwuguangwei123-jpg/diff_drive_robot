#include <csignal>
#include <memory>

#include "diff_drive_control_cpp/traj_tracking_node.hpp"
#include "rclcpp/rclcpp.hpp"

std::shared_ptr<TrajTrackingNode> g_node = nullptr;

void sigint_handler(int sig) {
    (void)sig;
    if (g_node) {
        g_node->stop_robot();
    }
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    rclcpp::InitOptions options;
    options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, options);

    g_node = std::make_shared<TrajTrackingNode>();
    std::signal(SIGINT, sigint_handler);

    rclcpp::spin(g_node);
    g_node.reset();
    return 0;
}
