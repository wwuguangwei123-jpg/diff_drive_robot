#ifndef DIFF_DRIVE_CONTROL_CPP__PID_CONTROLLER_HPP_
#define DIFF_DRIVE_CONTROL_CPP__PID_CONTROLLER_HPP_

class PidController {
public:
    PidController(double kp, double ki, double kd, double max_output, double min_output);

    double calculate(double error, double dt);
    void setGains(double kp, double ki, double kd);
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
