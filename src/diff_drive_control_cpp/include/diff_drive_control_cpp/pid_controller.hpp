#ifndef DIFF_DRIVE_CONTROL_CPP__PID_CONTROLLER_HPP_
#define DIFF_DRIVE_CONTROL_CPP__PID_CONTROLLER_HPP_

class PidController {
public:
    // 构造函数，初始化 PID 参数
    PidController(double kp, double ki, double kd, double max_output, double min_output);

    // 核心计算函数：输入误差和时间间隔，输出控制量
    double calculate(double error, double dt);

    // 动态调整参数的接口
    void setGains(double kp, double ki, double kd);
    
    // 重置积分项（换目标或者卡死时很有用）
    void reset();

private:
    double kp_;
    double ki_;
    double kd_;
    double max_output_;
    double min_output_;

    double integral_ = 0.0;
    double prev_error_ = 0.0;
};

#endif  // DIFF_DRIVE_CONTROL_CPP__PID_CONTROLLER_HPP_