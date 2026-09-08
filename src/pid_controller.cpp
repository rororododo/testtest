#include "pid_controller.h"

PIDController::PIDController(double kp, double ki, double kd)
    : Kp(kp), Ki(ki), Kd(kd), integral(0.0), lastError(0.0), firstRun(true) {}

double PIDController::compute(double setpoint, double measurement, double dt) {
    double error = setpoint - measurement;

    // 적분 (windup 방지: -50 ~ +50 제한)
    integral += error * dt;
    if (integral >  50.0) integral =  50.0;
    if (integral < -50.0) integral = -50.0;

    // 미분 (첫 실행 시 0으로)
    double derivative = 0.0;
    if (!firstRun) {
        derivative = (error - lastError) / dt;
    }
    firstRun = false;
    lastError = error;

    double output = Kp * error + Ki * integral + Kd * derivative;

    // 출력 제한 (-1.0 ~ +1.0)
    if (output >  1.0) output =  1.0;
    if (output < -1.0) output = -1.0;

    return output;
}

void PIDController::reset() {
    integral  = 0.0;
    lastError = 0.0;
    firstRun  = true;
}

void PIDController::setGains(double kp, double ki, double kd) {
    Kp = kp; Ki = ki; Kd = kd;
}
