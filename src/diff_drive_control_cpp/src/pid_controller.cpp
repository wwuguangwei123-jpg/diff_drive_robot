#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" // 🌟 引入雷达消息类型
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>

// 定义状态机的两个状态
enum class RobotState {
    GO_TO_GOAL,
    AVOID_OBSTACLE
};

class SmartController : public rclcpp::Node {
public:
    SmartController() : Node("smart_controller_node"), current_state_(RobotState::GO_TO_GOAL) {
        // 1. 订阅 Odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SmartController::odom_callback, this, std::placeholders::_1));
        
        // 2. 🌟 订阅激光雷达数据
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SmartController::scan_callback, this, std::placeholders::_1));

        // 3. 发布控制指令
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // 4. 控制循环 (20Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&SmartController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "具备激光雷达避障能力的 C++ 控制器已启动！");
    }
    // 🌟 新增：优雅退出时的紧急刹车函数
    void stop_robot() {
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        cmd_pub_->publish(cmd_msg);
        
        RCLCPP_INFO(this->get_logger(), "🔴 接收到 Ctrl+C 退出信号，已发送紧急停车指令！");
        
        // Pro-Tip: 暂停 100 毫秒，确保 DDS 中间件有足够的时间把这条消息发到网络上，节点再彻底死亡
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }

    // 🌟 雷达回调函数：处理 360 度的环境点云数据
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 获取前方、左前、右前三个扇区的最小距离
        // 注意：在你的配置中，数组大小 360。正前方是索引 180。
        
        min_dist_front_ = get_min_range(msg->ranges, 160, 200); // 正前方 +/- 20度
        min_dist_left_  = get_min_range(msg->ranges, 200, 240); // 左前方
        min_dist_right_ = get_min_range(msg->ranges, 120, 160); // 右前方

        // 状态机切换逻辑：如果前方距离小于 0.8 米，切换为避障模式
        if (min_dist_front_ < 0.8) {
            if (current_state_ != RobotState::AVOID_OBSTACLE) {
                RCLCPP_WARN(this->get_logger(), "前方遇阻！距离: %.2f 米。切换至避障模式！", min_dist_front_);
                current_state_ = RobotState::AVOID_OBSTACLE;
            }
        } else {
            if (current_state_ != RobotState::GO_TO_GOAL) {
                RCLCPP_INFO(this->get_logger(), "前方安全。恢复寻迹模式。");
                current_state_ = RobotState::GO_TO_GOAL;
            }
        }
    }

    // 辅助函数：计算指定扇区内的最短距离（过滤掉无穷大或非法值）
    double get_min_range(const std::vector<float>& ranges, int start_idx, int end_idx) {
        double min_val = 100.0;
        for (int i = start_idx; i <= end_idx; ++i) {
            if (std::isnormal(ranges[i]) && ranges[i] < min_val) {
                min_val = ranges[i];
            }
        }
        return min_val;
    }

    void control_loop() {
        auto cmd_msg = geometry_msgs::msg::Twist();

        if (current_state_ == RobotState::AVOID_OBSTACLE) {
            // 🌟 避障算法逻辑
            cmd_msg.linear.x = 0.0; // 降低线速度，甚至可以设为负数倒车

            // 比较左右两侧哪边更空旷，决定转弯方向
            if (min_dist_left_ > min_dist_right_) {
                cmd_msg.angular.z = 1.0;  // 左侧空旷，左转
            } else {
                cmd_msg.angular.z = -1.0; // 右侧空旷，右转
            }

        } else if (current_state_ == RobotState::GO_TO_GOAL) {
            // 🌟 原本的 PID 寻迹逻辑 (目标设得远一点，方便测试避障)
            double target_x = 5.0;
            double target_y = 5.0;

            double distance_error = std::hypot(target_x - current_x_, target_y - current_y_);
            double target_yaw = std::atan2(target_y - current_y_, target_x - current_x_);
            double angle_error = target_yaw - current_yaw_;

            while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
            while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

            if (distance_error > 0.1) {
                cmd_msg.linear.x = std::min(0.5, 0.5 * distance_error);
                cmd_msg.angular.z = 1.5 * angle_error;
            } else {
                cmd_msg.linear.x = 0.0;
                cmd_msg.angular.z = 0.0;
            }
        }

        cmd_pub_->publish(cmd_msg);
    }

    RobotState current_state_;
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
    double min_dist_front_ = 10.0, min_dist_left_ = 10.0, min_dist_right_ = 10.0;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
// 🌟 定义一个全局指针，为了让信号处理函数能调到我们的节点
std::shared_ptr<SmartController> g_node = nullptr;

// 🌟 自己写一个信号处理函数
void sigint_handler(int sig) {
    (void)sig;
    if (g_node) {
        g_node->stop_robot(); // 发送全零速度
    }
    rclcpp::shutdown(); // 确认发完之后，再彻底销毁 ROS 2 网络
}
int main(int argc, char ** argv) {
    // 🌟 关键修改：禁用 ROS 2 默认的 Ctrl+C 处理机制
    rclcpp::InitOptions options;
    options.shutdown_on_signal = false; 
    rclcpp::init(argc, argv, options);

    g_node = std::make_shared<SmartController>();

    // 告诉操作系统：如果遇到 Ctrl+C (SIGINT)，去执行我上面写的 sigint_handler
    std::signal(SIGINT, sigint_handler);

    rclcpp::spin(g_node);
    return 0;
}
