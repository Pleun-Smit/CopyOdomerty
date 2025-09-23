#include "Robot.h"
#include <frc/MathUtil.h>  // optional deadband helpers

void Robot::RobotInit() {
    // Example wheel offsets (distance from center in meters)
    std::array<Vector2D, NUM_WHEELS> wheelOffsets = {
        Vector2D(0.3,  0.3),  // front-left
        Vector2D(0.3, -0.3),  // front-right
        Vector2D(-0.3, 0.3),  // back-left
        Vector2D(-0.3, -0.3)  // back-right
    };

    // Initialize kinematics
    kinematics = new SwerveDriveKinematics(wheelOffsets);

    // Create 4 swerve modules (IDs depend on your wiring)
    modules[0] = new SwerveModule(1, 2, 0.0508); // front-left
    modules[1] = new SwerveModule(3, 4, 0.0508); // front-right
    modules[2] = new SwerveModule(5, 6, 0.0508); // back-left
    modules[3] = new SwerveModule(7, 8, 0.0508); // back-right

    // Reset odometry
    pose.resetPose(0.0, 0.0, 0.0);
    lastHeading = gyro.getHeading();
}

void Robot::TeleopPeriodic() {
    // --- 1. Read joystick axes ---
    double x = frc::ApplyDeadband(joystick.GetX(), 0.05);
    double y = frc::ApplyDeadband(joystick.GetY(), 0.05);
    double rot = frc::ApplyDeadband(joystick.GetZ(), 0.05);

    // --- 2. Scale to physical units ---
    Vector2D velocity(
        x * maxForwardSpeed,
        y * maxSideSpeed
    );
    double omega = rot * maxAngularSpeed;

    // --- 3. Build chassis state ---
    ChassisState chassisState(velocity, omega);

    // --- 4. Odometry update ---
    double heading = gyro.getHeading();
    double deltaHeading = heading - lastHeading;
    double avgDistance = 0.0;
    for (auto& module : modules) {
        avgDistance += module->getCurrentState().speed * 0.02; // 20ms loop
    }
    avgDistance /= NUM_WHEELS;
    pose.updatePose(avgDistance, deltaHeading);
    lastHeading = heading;

    // --- 5. Kinematics: chassis → wheel states ---
    auto wheelStates = kinematics->toWheelStates(chassisState, pose);

    // --- 6. Apply to each swerve module ---
    for (int i = 0; i < NUM_WHEELS; i++) {
        modules[i]->setDesiredState(wheelStates[i]);
    }
}

