#pragma once
#include "swerve/SwerveModule.h"
#include "hardware/IGyroSensor.h"
#include "odometry/Pose.h"
#include "Constants.h"
#include <array>

class Odometry {
public:
    Odometry(std::array<SwerveModule*, SwerveConstants::NUMBER_OF_MODULES> modules_, IGyroSensor* gyro_);
    void update();
    Pose getPose() const;
    void resetPose(double x, double y, double heading);

private:
    std::array<SwerveModule*, SwerveConstants::NUMBER_OF_MODULES> modules;
    IGyroSensor* gyro;
    Pose pose;
    double lastDistances[SwerveConstants::NUMBER_OF_MODULES];
    double lastHeading;
};
