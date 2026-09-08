#pragma once

class ReactionWheel {
public:
    // maxRPM: 최대 회전수, inertia: 휠 관성 모멘트 (kg*m^2)
    ReactionWheel(double maxRPM = 8000.0, double inertia = 0.01);

    // pidOutput: PID 제어기 출력 (-1.0 ~ +1.0), dt: 시간 간격
    void update(double pidOutput, double dt);

    double getRPM() const { return rpm; }
    double getTorque() const { return torque; }  // 로켓에 가하는 반토크 (N*m)
    double getMaxRPM() const { return maxRPM; }

private:
    double rpm;
    double maxRPM;
    double inertia;
    double torque;
};
