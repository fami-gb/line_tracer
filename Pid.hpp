#pragma once

#include <chrono>
#include <cmath>

struct PidGain {
    double kp_, ki_, kd_;
    PidGain(double kp, double ki, double kd);
};

class Pid
{
public:
    Pid(int target, double kp, double ki, double kd);
    int calculate(int error, double dt);
    void setPidGain(double kp, double ki, double kd);
    PidGain getPidGain() const;
    void Reset();

private:
    int target_ = 20;
    PidGain pidGain_;
    double integral = 0.0;
    double previousDerivative = 0.0;
    double previous_error = 0.0;
    double alpha = 0.7; // LPFの係数
    std::chrono::milliseconds previous_time = std::chrono::milliseconds(0);
};
