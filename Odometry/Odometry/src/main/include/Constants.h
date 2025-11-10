#pragma once
// -----------------------------------------------------------------------------
// Odometry project - constants grouped by purpose
// -----------------------------------------------------------------------------
/*
 * Constants.h
 * CopyOdometry - centralized robot-wide constants
 * Author: Pleun-Smit (refactor)
 */

#include <array>
#include <cstddef>
#include <utility> // for std::pair

namespace OperatorConstants {

// Controller port
inline constexpr int kDriverControllerPort = 0;

// Joystick mapping & filters (units: axis indices, deadband fraction)
inline constexpr int JOYSTICK_AXIS_FWD = 1;   // forward/back axis
inline constexpr int JOYSTICK_AXIS_STR = 0;   // strafe axis
inline constexpr int JOYSTICK_AXIS_ROT = 4;   // rotation axis
inline constexpr double JOYSTICK_DEADBAND = 0.05;

// Units & conversions
inline constexpr double PI = 3.14159265358979323846;

// Speed limits (tunable)
// Units: m/s for linear, rad/s for angular
inline constexpr double MAX_FORWARD_SPEED_MPS = 0.5;  // m/s
inline constexpr double MAX_SIDE_SPEED_MPS    = 0.5;  // m/s
inline constexpr double MAX_ANGULAR_SPEED_RPS = 6.28; // rad/s (~1 rev/s)

// Safety / drive input deadband
inline constexpr double DRIVE_DEADBAND = 0.2;

} // namespace OperatorConstants


namespace SwerveConstants {

// Number of modules/wheels
inline constexpr std::size_t NUMBER_OF_MODULES = 4;
inline constexpr std::size_t NUM_WHEELS = NUMBER_OF_MODULES; // alias for existing code

// Physical module parameters (units: meters, ratios)
inline constexpr double WHEEL_RADIUS_M   = 0.0375; // wheel radius in meters
inline constexpr double DRIVE_GEAR_RATIO = 5.50;   // motor->wheel gear ratio

// Module PID / control gains
inline constexpr double kP_STEER = 0.25; // proportional gain for steering

// Steering offsets (radians) - keep if needed for absolute encoders
inline constexpr double FRONT_LEFT_OFFSET  = PI;
inline constexpr double FRONT_RIGHT_OFFSET = 0.0;
inline constexpr double BACK_LEFT_OFFSET   = PI;
inline constexpr double BACK_RIGHT_OFFSET  = 0.0;

inline constexpr double MAX_WHEEL_SPEED_MPS = 3.0;

} // namespace SwerveConstants


namespace PIDGains{
    // steer gains
    double STEER_kP = 0.25;
}