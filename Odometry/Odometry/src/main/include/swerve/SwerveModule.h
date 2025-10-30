#pragma once
#include "WheelModuleState.h"
#include "Math/Vector2D.h"
#include "Math/MathUtils.h"

#include <rev/SparkMax.h>
#include <rev/RelativeEncoder.h>
#include <rev/AbsoluteEncoder.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

class SwerveModule {
public:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double MAX_SPEED = 3.0; // m/s

    SwerveModule(int driveID, int steerID, Vector2D wheelOffset, double wheelRadius, double gearRatio = 1.0, double steerOffset = 0.0);

    void setDesiredState(const WheelModuleState& state);
    WheelModuleState getCurrentState() const;
    void reset(); // resets drive encoder only
    double getDriveDistance() const;
    double getSteerAngle() const;
    void stop();

private:
    rev::spark::SparkMax driveMotor;
    rev::spark::SparkMax steerMotor;

    rev::RelativeEncoder& driveEncoder;
    rev::AbsoluteEncoder& steerEncoder;

    Vector2D wheelOffset;
    double wheelRadius;
    double gearRatio;
    double steerOffset;

    double optimizeSteerAngle(double currentAngle, double targetAngle) const;
};
