#include "rocket.h"
#include <cmath>
#include <cstdlib>

Rocket::Rocket() : windTimer(0.0), currentWind(0.0) {
    state.altitude        = 0.0;
    state.velocity        = 0.0;
    state.time            = 0.0;
    state.roll            = 0.0;
    state.rollRate        = 0.0;
    state.windDisturbance = 0.0;
}

void Rocket::update(double dt, double reactionTorque) {
    updateFlight(dt);
    updateAttitude(dt, reactionTorque);
    state.time += dt;
}

// 포물선 비행: 0 -> 10000m -> 0 (약 60초)
void Rocket::updateFlight(double dt) {
    const double totalTime  = 60.0;
    const double maxAlt     = 10000.0;

    // 사인파로 부드러운 포물선 궤도
    double t = state.time / totalTime * 3.14159265;
    state.altitude = maxAlt * std::sin(t);
    if (state.altitude < 0.0) state.altitude = 0.0;

    // 속도 = 고도 미분 근사
    double tNext = (state.time + dt) / totalTime * 3.14159265;
    double altNext = maxAlt * std::sin(tNext);
    state.velocity = (altNext - state.altitude) / dt;
}

// 롤 자세: 바람 외란 + 리액션휠 반토크
void Rocket::updateAttitude(double dt, double reactionTorque) {
    const double rocketInertia = 2.0; // 로켓 관성 (kg*m^2)

    state.windDisturbance = generateWindDisturbance();

    // 롤 각가속도 = (외란 - 리액션휠 반토크) / 관성
    double rollAccel = (state.windDisturbance - reactionTorque) / rocketInertia;

    state.rollRate += rollAccel * dt;
    state.roll     += state.rollRate * dt;
}

double Rocket::generateWindDisturbance() {
    windTimer += 0.016;
    if (windTimer > 2.0) {
        windTimer = 0.0;
        currentWind = ((double)rand() / RAND_MAX - 0.5) * 6.0; // -3 ~ +3 deg/s^2
    }
    return currentWind;
}
