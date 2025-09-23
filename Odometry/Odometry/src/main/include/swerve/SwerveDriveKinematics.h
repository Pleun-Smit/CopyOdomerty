#pragma once
#include <array>
#include "WheelModuleState.h"
#include "ChassisState.h"
#include "Math/Vector2D.h"

class SwerveDriveKinematics {
public:
    static constexpr int NUM_WHEELS = 4;
    static constexpr int MAX_SPEED = 5;

    std::array<Vector2D, NUM_WHEELS> wheelOffsets;

    SwerveDriveKinematics(const std::array<Vector2D, NUM_WHEELS>& offsets);

    std::array<WheelModuleState, NUM_WHEELS> toWheelStates(const ChassisState& chassisState, const Pose& pose);
    ChassisState toChassisState(const std::array<WheelModuleState, NUM_WHEELS>& wheelStates);
    
    void toggleFieldRelativeControl(bool enabled);
    bool isFieldRelative() const;

private:
    bool fieldRelative = true;
    void normalizeWheelSpeeds(std::array<WheelModuleState, NUM_WHEELS>& states, double maxSpeed);
};
