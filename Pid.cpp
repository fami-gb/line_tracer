#include "Pid.hpp"

Pid::Pid(int target, double kp, double ki, double kd)
    : target_(target), pidGain_(kp, ki, kd) {}

PidGain::PidGain(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd) {}

void Pid::setPidGain(double kp, double ki, double kd) {
    pidGain_.kd_ = kd;
    pidGain_.ki_ = ki;
    pidGain_.kp_ = kp;
}

PidGain Pid::getPidGain() const {
    return pidGain_;
}

void Pid::Reset() {
    integral = 0.0;
    previousDerivative = 0.0;
    previous_error = 0.0;
    previous_time = std::chrono::milliseconds(0);
}

int Pid::calculate(int error, double dt) {
    if (dt == 0) dt = 0.008;

    integral += error * dt; // 積分項の計算
    double derivative = alpha * (error - previous_error) / dt + (1 - alpha) * previousDerivative; // 微分項の計算
    previous_error = error; // 前回の誤差を更新
    previousDerivative = derivative; // 前回の微分を更新

    // 積分項の制限
    if (integral > 100) {
        integral = 100;
    } else if (integral < -100) {
        integral = -100;
    }

    // PID制御の計算
    double control = pidGain_.kp_ * error + pidGain_.ki_ * integral + pidGain_.kd_ * derivative;

    // 制御量を整数に変換して返す
    return static_cast<int>(std::round(control));
}
