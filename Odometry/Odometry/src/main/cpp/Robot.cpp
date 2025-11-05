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
    std::array<Vector2D, NUM_WHEELS> wheelOffsets = {
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
    for (int i = 0; i < NUM_WHEELS; i++) {
        modules[i]->reset();
    }

    gyro.reset();

    // Print initial absolute angles
    for (int i = 0; i < NUM_WHEELS; i++) {
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
    fwd *= maxForwardSpeed;
    str *= maxSideSpeed;
    rot *= maxAngularSpeed;

    // --- 3. Build chassis motion command ---
    ChassisState chassisState(Vector2D(fwd, str), rot);

    // --- 4. Convert chassis command to wheel states ---
    auto wheelStates = kinematics->toWheelStates(chassisState, odometry->getPose());

    // --- 5. Apply wheel states to modules using absolute steering encoder ---
    for (int i = 0; i < NUM_WHEELS; i++) {
        modules[i]->setDesiredState(wheelStates[i]);
    }

    // --- 6. Update odometry based on current wheel states and gyro ---
    odometry->update();

    // --- 7. Debug print: chassis state, wheel states, robot pose ---
    Pose currentPose = odometry->getPose();
    printf("Robot Pose: x=%.2f, y=%.2f, heading=%.2f°\n",
           currentPose.position.x,
           currentPose.position.y,
           currentPose.heading * 180.0 / PI);

    for (int i = 0; i < NUM_WHEELS; i++) {
        auto ws = modules[i]->getCurrentState();
        printf("Wheel %d: speed=%.2f m/s, angle=%.3f rad (%.1f°), driveDist=%.2f m\n",
               i, ws.speed, ws.angle, ws.angle * 180.0 / PI,
               modules[i]->getDriveDistance());
    }

    // --- 8. Optional: print raw joystick for debugging ---
    printf("Joystick raw axes: fwd=%.2f str=%.2f rot=%.2f\n", 
           joystick.GetRawAxis(1), joystick.GetRawAxis(0), joystick.GetRawAxis(4));
}

void Robot::DisabledInit(){
    for (int i = 0; i < NUM_WHEELS; i++) {
        modules[i]->stop();  // call stop() on each module
    }
    printf("Robot disabled: all motors stopped.\n");
}

// #include "Robot.h"
// #include <frc/MathUtil.h>
// #include <cstdio>
// #include "math/MathUtils.h"

// Robot::Robot() 
//     : kinematics(nullptr), odometry(nullptr) 
// {
//     // Nothing else here; RobotInit() does the real setup
// }

// Robot::~Robot() {
//     for (auto& module : modules) {
//         delete module;
//     }
//     delete kinematics;
//     delete odometry;
// }

// void Robot::RobotInit() {
//     // Define wheel offsets first
//     std::array<Vector2D, NUM_WHEELS> wheelOffsets = {
//         Vector2D(0.2575,  0.2575),   // front-left
//         Vector2D(0.2575, -0.2575),   // front-right
//         Vector2D(-0.2575, 0.2575),   // back-left
//         Vector2D(-0.2575, -0.2575)   // back-right
//     };

//     // Create swerve modules
//     modules = {
//         new SwerveModule(10, 11, wheelOffsets[0], 0.0375, 5.50, 0.0),
//         new SwerveModule(16, 17, wheelOffsets[1], 0.0375, 5.50, 0.0),
//         new SwerveModule(12, 13, wheelOffsets[2], 0.0375, 5.50, 0.0),
//         new SwerveModule(14, 15, wheelOffsets[3], 0.0375, 5.50, 0.0)
//     };

//     // Initialize kinematics
//     kinematics = new SwerveDriveKinematics(wheelOffsets);
//     kinematics->toggleFieldRelativeControl(false);

//     // Initialize odometry
//     odometry = new Odometry(modules, &gyro);
//     odometry->resetPose(0,0,0);

//     // Reset drive encoders only
//     for (int i = 0; i < NUM_WHEELS; i++) {
//         modules[i]->reset();
//     }

//     gyro.reset();

//     // Print initial absolute angles
//     for (int i = 0; i < NUM_WHEELS; i++) {
//         printf("Wheel %d initial absolute angle=%.3f rad\n", i, modules[i]->getSteerAngle());
//     }

//     printf("RobotInit complete.\n");
// }

// void Robot::TeleopPeriodic() {
//     // Static state for hybrid rotation handler (keeps state across calls)
//     static bool teleopInitialized = false;
//     static double headingSetpoint = 0.0;
//     static double lastHeadingError = 0.0;
//     static double lastDesiredOmega = 0.0;

//     // tuning parameters (adjust as needed)
//     const double ROT_DEADBAND = 0.05;       // joystick deadband for rotation
//     const double HEADING_KP = 3.0;          // P gain for heading hold (tune)
//     const double HEADING_KD = 0.0;          // optional D term
//     const double MAX_ANG_ACCEL = 6.28;      // rad/s^2 for slew limiting (tune)
//     const double dt = 0.02;                 // loop period in seconds (approx. 50Hz)

//     if (!teleopInitialized) {
//         headingSetpoint = gyro.getHeading();
//         lastHeadingError = 0.0;
//         lastDesiredOmega = 0.0;
//         teleopInitialized = true;
//     }

//     // --- 1. Read joystick axes with deadband ---
//     double rawAxisFwd = joystick.GetRawAxis(1);
//     double rawAxisStr = joystick.GetRawAxis(0);
//     double rawAxisRot = joystick.GetRawAxis(4);

//     double fwd = -frc::ApplyDeadband(rawAxisFwd, 0.05); // forward/back
//     double str = frc::ApplyDeadband(rawAxisStr, 0.05);  // strafe
//     double rotInput = frc::ApplyDeadband(rawAxisRot, ROT_DEADBAND);  // rotation

//     // --- 2. Scale to max speeds ---
//     fwd *= maxForwardSpeed;
//     str *= maxSideSpeed;

//     // --- 3. Rotation handling: hybrid teleop (driver direct + heading hold) ---
//     // cubic scaling for fine control near center
//     double scaledRot = std::copysign(rotInput * rotInput * rotInput, rotInput);

//     double currentHeading = gyro.getHeading(); // radians
//     double desiredOmega = 0.0;

//     if (std::fabs(scaledRot) > 1e-6) {
//         // Driver commanding rotation: open-loop mapping, update heading setpoint so hold will keep current heading
//         desiredOmega = scaledRot * maxAngularSpeed;
//         // update hold target to current heading so transition is smooth on release
//         headingSetpoint = currentHeading;
//         // If you want to use a different behavior (e.g., accumulate heading setpoint while turning),
//         // change the line above accordingly.
//     } else {
//         // No driver input: heading-hold P(D) controller
//         double error = MathUtils::normalizeAngle(headingSetpoint - currentHeading);
//         double derivative = (error - lastHeadingError) / dt;
//         desiredOmega = HEADING_KP * error + HEADING_KD * derivative;

//         // clamp to max angular speed
//         if (desiredOmega > maxAngularSpeed) desiredOmega = maxAngularSpeed;
//         if (desiredOmega < -maxAngularSpeed) desiredOmega = -maxAngularSpeed;

//         lastHeadingError = error;
//     }

//     // --- 4. Slew limiting on angular rate to avoid jerky transitions ---
//     double maxDeltaOmega = MAX_ANG_ACCEL * dt;
//     double deltaOmega = desiredOmega - lastDesiredOmega;
//     if (deltaOmega > maxDeltaOmega) deltaOmega = maxDeltaOmega;
//     if (deltaOmega < -maxDeltaOmega) deltaOmega = -maxDeltaOmega;
//     desiredOmega = lastDesiredOmega + deltaOmega;
//     lastDesiredOmega = desiredOmega;

//     // Debug prints: raw axes, desired omega, heading error
//     printf("RAW AXES: rawFwd=%.4f rawStr=%.4f rawRot=%.4f -> fwd=%.4f str=%.4f\n",
//            rawAxisFwd, rawAxisStr, rawAxisRot, fwd, str);
//     printf("Rotation: scaledRot=%.4f desiredOmega=%.4f rad/s headingSetpoint=%.3f currHeading=%.3f err=%.3f deg\n",
//            scaledRot, desiredOmega, headingSetpoint, currentHeading, MathUtils::radToDeg(MathUtils::normalizeAngle(headingSetpoint - currentHeading)));

//     // --- 5. Build chassis motion command and convert to wheel states ---
//     ChassisState chassisState(Vector2D(fwd, str), desiredOmega);
//     auto wheelStates = kinematics->toWheelStates(chassisState, odometry->getPose());

//     // Debug: print wheel speeds computed by kinematics before applying to modules
//     for (int i = 0; i < NUM_WHEELS; i++) {
//         printf("Pre-Module Wheel %d: speed=%.4f m/s angle=%.3f rad\n",
//                i, wheelStates[i].speed, wheelStates[i].angle);
//     }

//     // --- 6. Apply wheel states to modules using absolute steering encoder ---
//     for (int i = 0; i < NUM_WHEELS; i++) {
//         modules[i]->setDesiredState(wheelStates[i]);
//     }

//     // --- 7. Update odometry based on current wheel states and gyro ---
//     odometry->update();

//     // --- 8. Debug print: pose and wheel state info ---
//     Pose currentPose = odometry->getPose();
//     printf("Robot Pose: x=%.2f, y=%.2f, heading=%.2f°\n",
//            currentPose.position.x,
//            currentPose.position.y,
//            currentPose.heading * 180.0 / PI);

//     for (int i = 0; i < NUM_WHEELS; i++) {
//         auto ws = modules[i]->getCurrentState();
//         printf("Wheel %d: speed=%.2f m/s, angle=%.3f rad (%.1f°), driveDist=%.2f m\n",
//                i, ws.speed, ws.angle, ws.angle * 180.0 / PI,
//                modules[i]->getDriveDistance());
//     }

//     // --- 9. Optional: print raw joystick for debugging ---
//     printf("Joystick raw axes: fwd=%.2f str=%.2f rot=%.2f\n", 
//            joystick.GetRawAxis(1), joystick.GetRawAxis(0), joystick.GetRawAxis(4));
// }

// void Robot::DisabledInit(){
//     for (int i = 0; i < NUM_WHEELS; i++) {
//         modules[i]->stop();  // call stop() on each module
//     }
//     printf("Robot disabled: all motors stopped.\n");
// }