#pragma once
#include "WheelModuleState.h"
#include "Math/Vector2D.h"
#include "Math/MathUtils.h"
#include "Constants.h"
#include "hardware/Interfaces/IMotorController.h"

#include <rev/SparkMax.h>
#include <rev/RelativeEncoder.h>
#include <rev/AbsoluteEncoder.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

#include <memory>
#include "math/Vector2D.h"

class SwerveModule {
public:
    SwerveModule(
        std::unique_ptr<IMotorController> driveMotor,
        std::unique_ptr<IMotorController> steerMotor,
        Vector2D wheelOffset,
        double wheelRadius,
        double gearRatio,
        double steerOffset
    );

    void setDesiredState(const WheelModuleState& state);
    WheelModuleState getCurrentState() const;
    void reset(); // resets drive encoder only
    double getDriveDistance() const;
    double getSteerAngle() const;
    void stop();

private:
    std::unique_ptr<IMotorController> driveMotor;
    std::unique_ptr<IMotorController> steerMotor;
    Vector2D wheelOffset;
    double wheelRadius;
    double gearRatio;
    double steerOffset;
    double optimizeSteerAngle(double currentAngle, double targetAngle) const;
};