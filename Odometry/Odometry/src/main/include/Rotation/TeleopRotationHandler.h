#pragma once
#include "RotationController.h"
#include "math/MathUtils.h"
#include <cmath>

// Helper that turns a joystick input into a desired angular velocity for
// teleop, and uses a RotationController for heading-hold when stick released.
class TeleopRotationHandler {
public:
    TeleopRotationHandler(RotationController* controller, double maxAngularSpeed, double deadband = 0.05)
        : controller(controller), maxAngularSpeed(maxAngularSpeed), deadband(deadband) {}

    // call at robot init
    void reset(double currentHeading) {
        controller->reset(currentHeading);
        headingSetpoint = currentHeading;
        lastDesiredOmega = 0.0;
    }

    // joystickInput in [-1,1], dt in seconds
    double update(double joystickInput, double currentHeading, double dt) {
        // apply deadband
        double rot = joystickInput;
        if (std::fabs(rot) < deadband) rot = 0.0;

        // cubic scaling for fine control near center
        double scaled = std::copysign(rot * rot * rot, rot);

        // If driver commanding rotation, use open-loop mapping and update setpoint
        if (std::fabs(scaled) > 0.0) {
            double commandedOmega = scaled * maxAngularSpeed;
            // ensure the heading controller's setpoint gets reset to current so transition smooth
            controller->setTargetHeading(currentHeading);
            // optionally clear any integral/forced states if supported
            // if controller exposes clearForcedOmega, call it (not in RotationController interface)
            return commandedOmega;
        } else {
            // no driver input -> heading hold via controller
            // ensure we have a valid setpoint (first time after driver release)
            controller->setTargetHeading(headingSetpoint); // safe no-op if same
            double out = controller->update(currentHeading, dt);
            return out;
        }
    }

    // When driver begins to command rotation, update the internal heading setpoint
    void setHoldHeading(double heading) { headingSetpoint = MathUtils::normalizeAngle(heading); }

    void updateHoldHeadingFromCurrent(double currentHeading) { headingSetpoint = MathUtils::normalizeAngle(currentHeading); }

private:
    RotationController* controller;
    double maxAngularSpeed;
    double deadband;
    double headingSetpoint = 0.0;
    double lastDesiredOmega = 0.0;
};