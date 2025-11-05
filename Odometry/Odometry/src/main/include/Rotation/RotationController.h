#pragma once
// Simple rotation controller interfaces and helpers used by teleop/autonomous.

#include <optional>

class RotationController {
public:
    virtual ~RotationController() = default;

    // Reset internal integrators/state to current heading
    virtual void reset(double currentHeading) = 0;

    // Set a desired absolute heading (radians). Controller will try to reach & hold it.
    virtual void setTargetHeading(double headingRad) = 0;

    // Directly request an angular velocity setpoint (rad/s). Useful for open-loop command.
    virtual void setTargetOmega(double omegaRadPerSec) = 0;

    // Some controllers support motion profile requests (target heading with constraints).
    // By default this can be a no-op for simple controllers.
    struct ProfileConstraints {
        double maxVelocity;   // rad/s
        double maxAcceleration; // rad/s^2
    };
    virtual void setProfiledTarget(double targetHeading, ProfileConstraints constraints) {
        (void)targetHeading; (void)constraints;
    }

    // Main update method: returns commanded angular velocity (rad/s) to apply this cycle.
    // currentHeading must be in radians. dt is loop period in seconds.
    virtual double update(double currentHeading, double dt) = 0;

    // Whether controller is within tolerance of its heading setpoint. Tolerance in radians.
    virtual bool atSetpoint(double positionToleranceRad = 0.02, double velocityTolerance = 0.05) const = 0;
};