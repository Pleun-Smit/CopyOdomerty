#pragma once
#include "swerve/SwerveModule.h"
#include "hardware/Module.h"
#include "hardware/Interfaces/IGyroSensor.h"
#include "odometry/Pose.h"
#include "Constants.h"
#include <array>
#include <memory>

class Odometry : public Module {
public:
    Odometry(std::array<std::unique_ptr<SwerveModule>, SwerveConstants::NUMBER_OF_MODULES> modules_, std::unique_ptr<IGyroSensor> gyro_);
    void update() override;
    Pose getPose() const;
    void resetPose(double x, double y, double heading);

private:
    std::array<std::unique_ptr<SwerveModule>, SwerveConstants::NUMBER_OF_MODULES> modules;
    std::unique_ptr<IGyroSensor> gyro;
    Pose pose;
    double lastDistances[SwerveConstants::NUMBER_OF_MODULES];
    double lastHeading;
};
