#pragma once

#include <array>
#include "swerve/ChassisState.h"
#include "swerve/WheelModuleState.h"
#include "math/Vector2D.h"
#include "odometry/Pose.h"
#include "Constants.h"

class SwerveDriveKinematics {
public:

    explicit SwerveDriveKinematics(const std::array<Vector2D, SwerveConstants::NUM_WHEELS>& offsets);

    std::array<WheelModuleState, SwerveConstants::NUM_WHEELS> toWheelStates(
        const ChassisState& chassisState, 
        const Pose& pose
    );

    ChassisState toChassisState(
        const std::array<WheelModuleState, SwerveConstants::NUM_WHEELS>& wheelStates
    );

    void normalizeWheelSpeeds(
        std::array<WheelModuleState, SwerveConstants::NUM_WHEELS>& states, 
        double maxSpeed
    );

    void toggleFieldRelativeControl(bool enabled);
    bool isFieldRelative() const;

private:
    bool fieldRelative = false;
    std::array<Vector2D, SwerveConstants::NUM_WHEELS> wheelOffsets;
};
