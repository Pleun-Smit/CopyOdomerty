// #pragma once
// #include "odometry/Pose.h"
// #include "hardware/IEncoderSensor.h"
// #include "hardware/IGyroSensor.h"
// #include "swerve/SwerveModule.h"
// #include "math/Vector2D.h"

// #include <array>
// class Odometry {
// private:
//     std::array<SwerveModule*, 4> modules;
//     IGyroSensor* gyro;

//     Pose pose;
//     std::array<double, 4> lastDistances;
//     double lastHeading;

// public:
//     Odometry(std::array<SwerveModule*, 4> modules_, IGyroSensor* gyro_);

//     void update();               // Call every cycle to update pose
//     Pose getPose() const;        // Returns a copy of current pose
//     void resetPose(double x = 0, double y = 0, double heading = 0);
// };
#pragma once
#include "swerve/SwerveModule.h"
#include "hardware/IGyroSensor.h"
#include "odometry/Pose.h"
#include <array>

class Odometry {
public:
    Odometry(std::array<SwerveModule*, 4> modules_, IGyroSensor* gyro_);
    void update();
    Pose getPose() const;
    void resetPose(double x, double y, double heading);

private:
    std::array<SwerveModule*, 4> modules;
    IGyroSensor* gyro;
    Pose pose;
    double lastDistances[4];
    double lastHeading;
};
