#include "reaction_wheel.h"
#include <cmath>

ReactionWheel::ReactionWheel(double maxRPM, double inertia)
    : rpm(0.0), maxRPM(maxRPM), inertia(inertia), torque(0.0) {}

void ReactionWheel::update(double pidOutput, double dt) {
    // PID 출력 (-1 ~ +1) -> 목표 RPM
    double targetRPM = pidOutput * maxRPM;

    // 실제 RPM은 목표 RPM을 향해 서서히 따라감 (응답 지연)
    double rpmError = targetRPM - rpm;
    rpm += rpmError * dt * 3.0; // 3.0: 응답 속도 (빠를수록 즉각 반응)

    // RPM 클램핑
    if (rpm >  maxRPM) rpm =  maxRPM;
    if (rpm < -maxRPM) rpm = -maxRPM;

    // 토크 = 관성 * 각가속도
    // 각속도 (rad/s) = rpm * 2pi / 60
    double omega = rpm * 2.0 * 3.14159265 / 60.0;
    double omegaTarget = targetRPM * 2.0 * 3.14159265 / 60.0;
    torque = inertia * (omegaTarget - omega) / dt * 0.01; // 스케일 조정
}
