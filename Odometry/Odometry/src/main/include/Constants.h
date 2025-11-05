#pragma once
// -----------------------------------------------------------------------------
// Odometry project
// -----------------------------------------------------------------------------
/*
 * constants.h
 * CopyOdometry - constants grouped by purpose
 * Author: Pleun-Smit
 */

namespace OperatorConstants {
// *** BUILD / FEATURES ***

// *** UNITS & CONVERSIONS ***

// *** CONSTANTS ***
static constexpr double PI = 3.14159265358979323846;

// *** ROBOT GEOMETRY (METERS) ***
double wheelRadius = ;
double gearRatio = ;
const std::size_t numberOfModules = 4; 

// *** WHEEL / DRIVE PARAMETERS ***

// *** ENCODER / SENSOR SPECS ***

// *** IMU ***

// *** ODOMETRY ***

// *** KINEMATICS ***

// *** CONTROL GAINS ***

// *** MOTION LIMITS ***

// *** ***


inline constexpr int kDriverControllerPort = 0;

    

    constexpr double TRACK_WIDTH = 0.515;
    constexpr double WHEEL_BASE  = 0.515;

    // Max speeds
    constexpr double MAX_SPEED = 0.5; // m/s (same as Java's limit for safety)

    // Steer offsets (from the working Java code)
    constexpr double FRONT_LEFT_OFFSET  = M_PI;
    constexpr double FRONT_RIGHT_OFFSET = 0.0;
    constexpr double BACK_LEFT_OFFSET   = M_PI;
    constexpr double BACK_RIGHT_OFFSET  = 0.0;

    constexpr double DRIVE_DEADBAND = 0.2;
    constexpr double STEER_MOTOR_KP = 0.25;

}  // namespace OperatorConstants
