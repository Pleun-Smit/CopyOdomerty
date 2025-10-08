#include "Robot.h"
#include <frc/MathUtil.h>
#include <cstdio>

Robot::Robot() 
    : kinematics(nullptr), odometry(nullptr) 
{
    // Nothing else here; RobotInit() does the real setup
}

Robot::~Robot() {
    for (auto& module : modules) {
        delete module;
    }
    delete kinematics;
    delete odometry;
}

void Robot::RobotInit() {
    modules = {
        new SwerveModule(10, 11, 0.0375, 5.50), // front-left
        modules[1] = new SwerveModule(16, 17, 0.0375, 5.50), // front-right
        modules[2] = new SwerveModule(12, 13, 0.0375, 5.50), // back-lef
        modules[3] = new SwerveModule(14, 15, 0.0375, 5.50) // back-right
    };
    // Example wheel offsets (meters)
    std::array<Vector2D, NUM_WHEELS> wheelOffsets = {
        Vector2D(0.2575,  0.2575),   // front-left
        Vector2D(0.2575, -0.2575),   // front-right
        Vector2D(-0.2575, 0.2575),   // back-left
        Vector2D(-0.2575, -0.2575)   // back-right
    };

    // Initialize kinematics
    kinematics = new SwerveDriveKinematics(wheelOffsets);

    odometry = new Odometry(modules, &gyro);

    gyro.reset();
    odometry->resetPose(0,0,0);
    
    // Reset getDriveDistance
    for (int i = 0; i < NUM_WHEELS; i++) {
        modules[i]->reset();  // resets drive encoder and steer encoder
    }

    printf("RobotInit complete.\n");
    // Create swerve modules (IDs depend on wiring)
    // modules[0] = new SwerveModule(10, 11, 0.0375, 5.50); // front-left
    // modules[1] = new SwerveModule(16, 17, 0.0375, 5.50); // front-right
    // modules[2] = new SwerveModule(12, 13, 0.0375, 5.50); // back-left
    // modules[3] = new SwerveModule(14, 15, 0.0375, 5.50); // back-right
    
    // Reset odometry
    // pose.resetPose(0.0, 0.0, 0.0);
    // lastHeading = gyro.getHeading();
}

void Robot::TeleopPeriodic() {
    // --- 1. Read joystick axes with deadband ---
    double x = frc::ApplyDeadband(joystick.GetX(), 0.05);
    double y = frc::ApplyDeadband(joystick.GetY(), 0.05);
    double rot = frc::ApplyDeadband(joystick.GetZ(), 0.05);

    // --- 2. Scale joystick input to physical units ---
    Vector2D velocity(
        x * maxForwardSpeed,  // m/s
        y * maxSideSpeed      // m/s
    );
    double omega = rot * maxAngularSpeed; // rad/s

    odometry->update();

    Pose pose = odometry->getPose();
    ChassisState chassisState(velocity, omega);
    auto wheelStates = kinematics->toWheelStates(chassisState, pose);

    for (int i =0; i< NUM_WHEELS; i++){
        modules[i]->setDesiredState(wheelStates[i]);
    }

        // --- 5. Debug output ---
    printf("Pose: x=%.2f, y=%.2f, heading=%.2f deg\n",
           pose.position.x, pose.position.y,
           MathUtils::radToDeg(pose.getHeading()));

    // // --- 3. Odometry update (Phase 1) ---
    // double heading = gyro.getHeading();
    // double deltaHeading = heading - lastHeading;

    // // double avgDistance = 0.0;
    // // for (auto& module : modules) {
    // //     avgDistance += module->getCurrentState().speed * 0.02; // 20ms loop
    // // }
    // // avgDistance /= NUM_WHEELS;
    // double avgDistance = 0.0;
    // for (int i = 0; i < NUM_WHEELS; i++) {
    //     double delta = modules[i]->getDriveDistance() - lastDistances[i];
    //     lastDistances[i] = modules[i]->getDriveDistance();  // update for next loop
    //     avgDistance += delta;
    // }
    // avgDistance /= NUM_WHEELS;

    // pose.updatePose(avgDistance, deltaHeading);
    // lastHeading = heading;

    // // Debug print
    // printf("Pose: x=%.2f, y=%.2f, heading=%.2f deg\n", pose.position.x, pose.position.y, MathUtils::radToDeg(pose.getHeading()));
    
    // printf("Drive distances: FL=%.3f FR=%.3f BL=%.3f BR=%.3f\n",
    //        modules[0]->getDriveDistance(),
    //        modules[1]->getDriveDistance(),
    //        modules[2]->getDriveDistance(),
    //        modules[3]->getDriveDistance());

    // // --- 4. Build chassis state (Phase 2) ---0
    // ChassisState chassisState(velocity, omega);

    // // --- 5. Compute wheel states from chassis state ---
    // auto wheelStates = kinematics->toWheelStates(chassisState, pose);

    // // --- 6. Apply wheel states to each swerve module ---
    // for (int i = 0; i < NUM_WHEELS; i++) {
    //     modules[i]->setDesiredState(wheelStates[i]);
    // }
}

void Robot::DisabledInit(){
    for (int i = 0; i < NUM_WHEELS; i++) {
        modules[i]->stop();  // call stop() on each module
    }
    printf("Robot disabled: all motors stopped.\n");
}


