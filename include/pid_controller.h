#pragma once

class PIDController {
public:
    PIDController(double kp, double ki, double kd);

    // setpoint: 목표값, measurement: 현재 측정값, dt: 시간 간격
    double compute(double setpoint, double measurement, double dt);

    void reset();
    void setGains(double kp, double ki, double kd);

    double getKp() const { return Kp; }
    double getKi() const { return Ki; }
    double getKd() const { return Kd; }
    double getLastError() const { return lastError; }

private:
    double Kp, Ki, Kd;
    double integral;
    double lastError;
    bool firstRun;
};
