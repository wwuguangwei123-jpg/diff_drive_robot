#include "diff_drive_control_cpp/traj_tracking_node.hpp"
#include <cmath>
#include <algorithm>

TrajTrackingNode::TrajTrackingNode() 
    : Node("traj_tracking_node"), 
      current_state_(RobotState::GO_TO_GOAL),
      // 初始化PID参数：Kp, Ki, Kd, Max_Output, Min_Output
      linear_pid_(0.6, 0.01, 0.1, 0.5, -0.1),   // 纵向速度控制：最大前行0.5m/s，最大倒车-0.2m/s
      angular_pid_(2.0, 0.02, 0.1, 1.5, -1.5)  // 横向角速度控制：最大旋转1.5rad/s
{
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&TrajTrackingNode::odom_callback, this, std::placeholders::_1));
        
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&TrajTrackingNode::scan_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    cte_pub_ = this->create_publisher<std_msgs::msg::Float64>("/cte", 10);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&TrajTrackingNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "工业级解耦全栈控制节点已成功挂载！");
    // 🌟 新增：订阅 Nav2 的全局路径话题
    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&TrajTrackingNode::plan_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // ... timer_ 的创建 ...

    RCLCPP_INFO(this->get_logger(), "纯追踪控制节点就绪！等待接收 /plan 话题路径...");
}

void TrajTrackingNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
}

void TrajTrackingNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
// 🌟 核心逻辑 1：如果已经到了终点，直接无视雷达数据，原地躺平
    if (current_state_ == RobotState::REACHED_GOAL) {
        return; 
    }
    min_dist_front_ = get_min_range(msg->ranges, 160, 200); 
    min_dist_left_  = get_min_range(msg->ranges, 200, 240); 
    min_dist_right_ = get_min_range(msg->ranges, 120, 160); 

    if (min_dist_front_ < 0.8) {
        if (current_state_ != RobotState::AVOID_OBSTACLE) {
            RCLCPP_WARN(this->get_logger(), "【安全警报】前方障碍物逼近 (%.2f米)！切入避障状态。", min_dist_front_);
            current_state_ = RobotState::AVOID_OBSTACLE;
            linear_pid_.reset();  // 状态切换时及时重置积分项，避免控制过冲
            angular_pid_.reset();
        }
    } else {
        if (current_state_ != RobotState::GO_TO_GOAL) {
            RCLCPP_INFO(this->get_logger(), "【环境安全】障碍解除，恢复 PID 寻迹目标。");
            current_state_ = RobotState::GO_TO_GOAL;
        }
    }
}

double TrajTrackingNode::get_min_range(const std::vector<float>& ranges, int start_idx, int end_idx) {
    double min_val = 100.0;
    for (int i = start_idx; i <= end_idx; ++i) {
        if (std::isnormal(ranges[i]) && ranges[i] < min_val) {
            min_val = ranges[i];
        }
    }
    return min_val;
}

void TrajTrackingNode::control_loop() {
    auto cmd_msg = geometry_msgs::msg::Twist();
    double dt = 0.05; // 20Hz 对应的时间步长
    // 🌟 核心逻辑 3：到达终点，彻底断电
    if (current_state_ == RobotState::REACHED_GOAL) {
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        last_linear_x_ = 0.0; // 停机时也要重置历史速度，防止下次起步突变
        cmd_pub_->publish(cmd_msg);
        return;
    }
	
    if (current_state_ == RobotState::AVOID_OBSTACLE) {
        // 🌟 避障全面升级：老司机分级顺滑绕行策略
        if (min_dist_front_ < 0.4) {
            // 1. 极限危险区 (<0.4m)：贴脸了，必须挂倒挡强行拉开空间
            cmd_msg.linear.x = -0.15; 
            cmd_msg.angular.z = (min_dist_left_ > min_dist_right_) ? 1.5 : -1.5;
        } else {
            // 2. 顺滑绕行区 (0.4m ~ 1.2m)：边往前开边切弯，拒绝原地打转
            cmd_msg.linear.x = 0.25; // 保持 0.25m/s 的速度往前滑行
            
            // 🌟 比例灵敏度魔法：距离越近，方向盘打得越狠
            // 假设距离是 0.8，turn_speed = 1.0；距离逼近 0.4 时，turn_speed 飙升到 1.6
            double turn_speed = 1.0 + (0.8 - min_dist_front_) * 1.5; 
            
            cmd_msg.angular.z = (min_dist_left_ > min_dist_right_) ? turn_speed : -turn_speed;
        }
    }
    else if (current_state_ == RobotState::GO_TO_GOAL) {
    // 🌟 新增：空载保护
        if (global_path_.empty()) {
            return;
        }
        // ==========================================
        // 🌟 新篇章：实时计算并发布横向跟踪误差 (CTE)
        // ==========================================
        double min_cte = 1000.0; // 初始设一个极大的距离
        // 遍历整个全局路径，寻找离小车当前位置最近的那个点
        for (const auto& point : global_path_) {
            double dist = std::hypot(point.first - current_x_, point.second - current_y_);
            if (dist < min_cte) {
                min_cte = dist;
            }
        }
        
        // 发布 CTE 数据供 rqt_plot 画图使用
        auto cte_msg = std_msgs::msg::Float64();
        cte_msg.data = min_cte;
        cte_pub_->publish(cte_msg);
        double target_x = 0.0;
        double target_y = 0.0;

        // 1. 寻找前瞻点
        if (!find_lookahead_point(target_x, target_y)) {
            // 🌟 修复直线 Bug：一旦找不到合法的前瞻点，说明真正进入终点盲区，果断刹车锁死
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
            current_state_ = RobotState::REACHED_GOAL;
            RCLCPP_INFO_ONCE(this->get_logger(), "🏁 S 型赛道追踪完成，完美冲线！");
        } else {
            // 2. 几何计算
            double dx = target_x - current_x_;
            double dy = target_y - current_y_;
            double alpha = std::atan2(dy, dx) - current_yaw_;

            while (alpha > M_PI) alpha -= 2.0 * M_PI;
            while (alpha < -M_PI) alpha += 2.0 * M_PI;

            double kappa = (2.0 * std::sin(alpha)) / lookahead_distance_;

            // 3. 🌟🌟🌟 动态速度控制引擎 🌟🌟🌟
            double max_v = 0.6; // 直道最大速度拉高到 0.6 m/s
            double min_v = 0.15; // 弯道最低保障速度

            // 策略 A：弯道自适应减速（根据曲率绝对值衰减线速度）
            // 曲率越大，分母越大，速度越慢
            double v_curve = max_v / (1.0 + 2.0 * std::abs(kappa)); 

            // 策略 B：终点减速（计算小车到赛道最后一个终点的绝对距离）
            double dist_to_endpoint = std::hypot(global_path_.back().first - current_x_, 
                                                 global_path_.back().second - current_y_);
            double v_stage = 1.0;
            if (dist_to_endpoint < 1.5) {
                v_stage = dist_to_endpoint / 1.5; // 进站 1.5 米内，速度随距离线性衰减
            }

            // 联合输出线速度：弯道速度 乘以 终点衰减系数，最后用 std::max 兜底
            cmd_msg.linear.x = std::max(min_v, v_curve * v_stage);

            // 如果已经极其靠近终点（比如 0.15米内），强制切入停机，防止冲过头
            if (dist_to_endpoint < 0.15) {
                cmd_msg.linear.x = 0.0;
                cmd_msg.angular.z = 0.0;
                current_state_ = RobotState::REACHED_GOAL;
                RCLCPP_INFO(this->get_logger(), "🎉 距离终点 %.2f 米，自适应减速进站成功！", dist_to_endpoint);
            } else {
                // 4. 运动学输出：omega = v * kappa
                cmd_msg.angular.z = cmd_msg.linear.x * kappa;
                // 调试信息：你可以取消下面这行的注释来观察底层的计算过程
            //RCLCPP_INFO(this->get_logger(), "跟踪点:(%.2f, %.2f), 夹角:%.2f, 输出角速度:%.2f", target_x, target_y, alpha, cmd_msg.angular.z);
            }
        }
    }
    // --- 2. 🌟🌟🌟 新增核心：工业级加速度限幅 (Slew Rate Limiter) 🌟🌟🌟 ---
    double max_accel = 0.5;              // 设定最大加速度：0.5 m/s^2 (防止起步打滑)
    double max_decel = 0.8;              // 设定最大减速度(刹车)：0.8 m/s^2 (防止点头扫地)
    
    double max_delta_v_accel = max_accel * dt; // 0.05秒内允许的最大加速增量
    double max_delta_v_decel = max_decel * dt; // 0.05秒内允许的最大刹车减量

    // 对向前行驶的加速/减速进行限制
    if (cmd_msg.linear.x > last_linear_x_) {
        // 正在加速
        if (cmd_msg.linear.x - last_linear_x_ > max_delta_v_accel) {
            cmd_msg.linear.x = last_linear_x_ + max_delta_v_accel;
        }
    } else {
        // 正在减速 (刹车)
        if (last_linear_x_ - cmd_msg.linear.x > max_delta_v_decel) {
            cmd_msg.linear.x = last_linear_x_ - max_delta_v_decel;
        }
    }

    // 更新历史速度记录，留给下一帧使用
    last_linear_x_ = cmd_msg.linear.x;
    // --------------------------------------------------
    cmd_pub_->publish(cmd_msg);
}

void TrajTrackingNode::stop_robot() {
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 0.0;
    cmd_pub_->publish(cmd_msg);
    RCLCPP_INFO(this->get_logger(), "🔴 节点安全卸载：紧急全零刹车指令已投递至 DDS 中间件。");
}
// 🌟 新增：遍历全局路径，寻找前瞻点
bool TrajTrackingNode::find_lookahead_point(double &target_x, double &target_y) {
    for (size_t i = current_path_index_; i < global_path_.size(); ++i) {
        double dist = std::hypot(global_path_[i].first - current_x_, global_path_[i].second - current_y_);
        
        if (dist >= lookahead_distance_) {
            target_x = global_path_[i].first;
            target_y = global_path_[i].second;
            current_path_index_ = i; // 记住这次看到哪了，下次不走回头路
            return true;
        }
    }
    return false; // 如果遍历完了都没找到大于 L_d 的点，说明快到终点了
}

void TrajTrackingNode::plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    // 如果收到的路径是空的，直接忽略
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "收到了空路径！");
        return;
    }

    // 清空旧路径，把 Nav2 发来的新坐标点全部装进去
    global_path_.clear();
    for (const auto& pose_stamped : msg->poses) {
        global_path_.push_back({pose_stamped.pose.position.x, pose_stamped.pose.position.y});
    }

    // 重置追踪进度，并激活寻迹状态
    current_path_index_ = 0; 
    current_state_ = RobotState::GO_TO_GOAL;
    RCLCPP_INFO(this->get_logger(), "🗺️ 成功接收新全局路径！共 %zu 个轨迹点，起步追踪！", global_path_.size());
}
