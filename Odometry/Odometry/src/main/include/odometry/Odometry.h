#pragma once
#include "swerve/SwerveModule.h"
#include "hardware/IGyroSensor.h"
#include "odometry/Pose.h"
#include "Constants.h"
#include <array>
using namespace OperatorConstants;

class Odometry {
public:
    Odometry(std::array<SwerveModule*, numberOfModules> modules_, IGyroSensor* gyro_);
    void update();
    Pose getPose() const;
    void resetPose(double x, double y, double heading);

private:
    std::array<SwerveModule*, numberOfModules> modules;
    IGyroSensor* gyro;
    Pose pose;
    double lastDistances[numberOfModules];
    double lastHeading;
};
