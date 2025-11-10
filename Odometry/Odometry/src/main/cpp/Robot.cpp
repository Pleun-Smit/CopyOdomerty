#include "Robot.h"
#include <frc/MathUtil.h>
#include <cstdio>
#include "math/MathUtils.h"

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
    // Define wheel offsets first
    std::array<Vector2D, SwerveConstants::NUM_WHEELS> wheelOffsets = {
        Vector2D(0.2575,  0.2575),   // front-left
        Vector2D(0.2575, -0.2575),   // front-right
        Vector2D(-0.2575, 0.2575),   // back-left
        Vector2D(-0.2575, -0.2575)   // back-right
    };

    // Create swerve modules
    modules = {
        new SwerveModule(10, 11, wheelOffsets[0], 0.0375, 5.50, 0.0),
        new SwerveModule(16, 17, wheelOffsets[1], 0.0375, 5.50, 0.0),
        new SwerveModule(12, 13, wheelOffsets[2], 0.0375, 5.50, 0.0),
        new SwerveModule(14, 15, wheelOffsets[3], 0.0375, 5.50, 0.0)
    };

    // Initialize kinematics
    kinematics = new SwerveDriveKinematics(wheelOffsets);
    kinematics->toggleFieldRelativeControl(false);

    // Initialize odometry
    odometry = new Odometry(modules, &gyro);
    odometry->resetPose(0,0,0);

    // Reset drive encoders only
    for (int i = 0; i < SwerveConstants::NUM_WHEELS; i++) {
        modules[i]->reset();
    }

    gyro.reset();

    // Print initial absolute angles
    for (int i = 0; i < SwerveConstants::NUM_WHEELS; i++) {
        printf("Wheel %d initial absolute angle=%.3f rad\n", i, modules[i]->getSteerAngle());
    }

    printf("RobotInit complete.\n");
}


void Robot::TeleopPeriodic() {
    // printf("FL absolute encoder: %.3f rad\n", modules[0]->getSteerAngle());
    // printf("FR absolute encoder: %.3f rad\n", modules[1]->getSteerAngle());
    // printf("BL absolute encoder: %.3f rad\n", modules[2]->getSteerAngle());
    // printf("BR absolute encoder: %.3f rad\n", modules[3]->getSteerAngle());

    // --- 1. Read joystick axes with deadband ---
    double fwd = -frc::ApplyDeadband(joystick.GetRawAxis(1), 0.05); // forward/back
    double str = frc::ApplyDeadband(joystick.GetRawAxis(0), 0.05);  // strafe
    double rot = frc::ApplyDeadband(joystick.GetRawAxis(4), 0.05);  // rotation

    // --- 2. Scale to max speeds ---
    fwd *= OperatorConstants::MAX_FORWARD_SPEED_MPS;
    str *= OperatorConstants::MAX_SIDE_SPEED_MPS;
    rot *= OperatorConstants::MAX_ANGULAR_SPEED_RPS;

    // --- 3. Build chassis motion command ---
    ChassisState chassisState(Vector2D(fwd, str), rot);

    // --- 4. Convert chassis command to wheel states ---
    auto wheelStates = kinematics->toWheelStates(chassisState, odometry->getPose());

    // --- 5. Apply wheel states to modules using absolute steering encoder ---
    for (int i = 0; i < SwerveConstants::NUM_WHEELS; i++) {
        modules[i]->setDesiredState(wheelStates[i]);
    }

    // --- 6. Update odometry based on current wheel states and gyro ---
    odometry->update();

    // --- 7. Debug print: chassis state, wheel states, robot pose ---
    Pose currentPose = odometry->getPose();
    printf("Robot Pose: x=%.2f, y=%.2f, heading=%.2f°\n",
           currentPose.position.x,
           currentPose.position.y,
           currentPose.heading * 180.0 / OperatorConstants::PI);

    for (int i = 0; i < SwerveConstants::NUM_WHEELS; i++) {
        auto ws = modules[i]->getCurrentState();
        printf("Wheel %d: speed=%.2f m/s, angle=%.3f rad (%.1f°), driveDist=%.2f m\n",
               i, ws.speed, ws.angle, ws.angle * 180.0 / OperatorConstants::PI,
               modules[i]->getDriveDistance());
    }

    // --- 8. Optional: print raw joystick for debugging ---
    printf("Joystick raw axes: fwd=%.2f str=%.2f rot=%.2f\n", 
           joystick.GetRawAxis(1), joystick.GetRawAxis(0), joystick.GetRawAxis(4));
}

void Robot::DisabledInit(){
    for (int i = 0; i < SwerveConstants::NUM_WHEELS; i++) {
        modules[i]->stop();  // call stop() on each module
    }
    printf("Robot disabled: all motors stopped.\n");
}
