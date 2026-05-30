#include "diff_drive_control_cpp/pid_controller.hpp"

PidController::PidController(double kp, double ki, double kd, double max_output, double min_output)
    : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), min_output_(min_output) {}

double PidController::calculate(double error, double dt) {
    if (dt <= 0.0) {
        return 0.0;
    }

    const double p_out = kp_ * error;

    integral_ += error * dt;
    const double i_out = ki_ * integral_;

    const double derivative = (error - prev_error_) / dt;
    const double d_out = kd_ * derivative;

    double output = p_out + i_out + d_out;
    if (output > max_output_) {
        output = max_output_;
        integral_ -= error * dt;
    } else if (output < min_output_) {
        output = min_output_;
        integral_ -= error * dt;
    }

    prev_error_ = error;
    return output;
}

void PidController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
}
