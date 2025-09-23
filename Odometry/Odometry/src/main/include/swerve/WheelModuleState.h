#pragma once

struct WheelModuleState {
    double speed;    // meters per second
    double angle;    // radians, 0 = forward

    WheelModuleState(double s = 0.0, double a = 0.0) : speed(s), angle(a) {}
};
