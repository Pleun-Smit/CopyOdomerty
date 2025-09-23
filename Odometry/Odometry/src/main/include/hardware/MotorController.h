#pragma once

class MotorController {
public:
    MotorController(int id) {}
    void Set(double value) {}    // set speed or angle
    double Get() const { return 0; }  // get current speed
};
