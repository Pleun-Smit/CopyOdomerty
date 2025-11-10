#pragma once

class Module {
public:
    virtual void update() = 0; // dt = delta time, can help calculate how far the robot travaled since the last update, it is not yet in use
    virtual ~Module() = default;
};
