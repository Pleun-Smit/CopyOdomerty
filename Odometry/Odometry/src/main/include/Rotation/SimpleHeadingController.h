#pragma once
#include "RotationController.h"
#include "math/MathUtils.h"
#include <algorithm>
#include <cmath>

class SimpleHeadingController : public RotationController {
public:
    // kp in units (rad/s) per radian of heading error — i.e. desiredOmega = Kp * error
    // optional kd and ki can be added if needed
    SimpleHeadingController(double kp = 2.0, double kd = 0.0, double ki = 0.0,
                            double maxOmega = 6.28) :
        Kp(kp), Kd(kd), Ki(ki), maxOmega(maxOmega) {}

    void reset(double currentHeading) override {
        targetHeading = currentHeading;
        lastError = 0.0;
        integral = 0.0;
    }

    void setTargetHeading(double headingRad) override {
        targetHeading = MathUtils::normalizeAngle(headingRad);
    }

    void setTargetOmega(double omegaRadPerSec) override {
        forcedOmega = omegaRadPerSec;
        forcedOmegaActive = true;
    }

    void setProfiledTarget(double targetHeading_, ProfileConstraints) override {
        // no profiling in simple controller
        setTargetHeading(targetHeading_);
    }

    double update(double currentHeading, double dt) override {
        // If forcedOmega active (explicit angular velocity command), return it (but clamp)
        if (forcedOmegaActive) {
            double out = std::clamp(forcedOmega, -maxOmega, maxOmega);
            // clear forced once requested (optional behavior); leave active until cleared externally
            return out;
        }

        double error = MathUtils::normalizeAngle(targetHeading - currentHeading);
        integral += error * dt;
        double derivative = (error - lastError) / (dt > 0 ? dt : 1e-6);

        double out = Kp * error + Ki * integral + Kd * derivative;
        out = std::clamp(out, -maxOmega, maxOmega);

        lastError = error;
        return out;
    }

    bool atSetpoint(double positionToleranceRad = 0.02, double velocityTolerance = 0.05) const override {
        return std::fabs(lastError) <= positionToleranceRad;
    }

    // Programmatic helper to cancel forced omega
    void clearForcedOmega() { forcedOmegaActive = false; forcedOmega = 0.0; }

private:
    double targetHeading = 0.0;
    double Kp = 1.0;
    double Kd = 0.0;
    double Ki = 0.0;
    double lastError = 0.0;
    double integral = 0.0;
    double maxOmega = 6.28;

    // optional direct-omega mode
    double forcedOmega = 0.0;
    bool forcedOmegaActive = false;
};