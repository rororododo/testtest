#pragma once
#include <cmath>
#include <cstdlib>

struct RocketState {
    double altitude;        // 고도 (m)
    double velocity;        // 수직 속도 (m/s)
    double time;            // 경과 시간 (s)
    double roll;            // 롤 각도 (deg)
    double rollRate;        // 롤 각속도 (deg/s)
    double windDisturbance; // 현재 바람 외란 (deg/s^2)
};

class Rocket {
public:
    Rocket();
    void update(double dt, double reactionTorque);
    const RocketState& getState() const { return state; }
    bool isFlying() const { return state.altitude >= 0.0 && state.time <= 65.0; }

private:
    RocketState state;
    void updateFlight(double dt);
    void updateAttitude(double dt, double reactionTorque);
    double generateWindDisturbance();
    double windTimer;
    double currentWind;
};
