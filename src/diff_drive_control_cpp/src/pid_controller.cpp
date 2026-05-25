#include "diff_drive_control_cpp/pid_controller.hpp"
#include <algorithm>

PidController::PidController(double kp, double ki, double kd, double max_output, double min_output)
    : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), min_output_(min_output) {}

double PidController::calculate(double error, double dt) {
    if (dt <= 0.0) return 0.0;

    // 1. 比例项
    double p_out = kp_ * error;

    // 2. 积分项
    integral_ += error * dt;
    double i_out = ki_ * integral_;

    // 3. 微分项
    double derivative = (error - prev_error_) / dt;
    double d_out = kd_ * derivative;

    // 4. 总输出
    double output = p_out + i_out + d_out;

    // 5. 工业级限幅（抗积分饱和限制）
    if (output > max_output_) {
        output = max_output_;
        integral_ -= error * dt; // 饱和了就不再积分
    } else if (output < min_output_) {
        output = min_output_;
        integral_ -= error * dt;
    }

    prev_error_ = error;
    return output;
}

void PidController::setGains(double kp, double ki, double kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}

void PidController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
}