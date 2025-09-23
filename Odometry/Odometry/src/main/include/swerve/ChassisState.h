#pragma once
#include "Math/Vector2D.h"

struct ChassisState {
    Vector2D velocity;  // vx, vy in robot coordinates
    double omega;       // angular velocity rad/s

    ChassisState(const Vector2D& v = Vector2D(0,0), double w = 0.0)
        : velocity(v), omega(w) {}
};
