#include "swerve/SwerveModule.h"

SwerveModule::SwerveModule(int driveID, int steerID, Vector2D wheelOffset_, double wheelRadius_, double gearRatio_, double steerOffset_)
    : driveMotor(driveID, rev::spark::SparkMax::MotorType::kBrushless),
      steerMotor(steerID, rev::spark::SparkMax::MotorType::kBrushless),
      driveEncoder(driveMotor.GetEncoder()),
      steerEncoder(steerMotor.GetAbsoluteEncoder()),
      wheelOffset(wheelOffset_),
      wheelRadius(wheelRadius_),
      gearRatio(gearRatio_),
      steerOffset(steerOffset_)
{
    driveEncoder.SetPosition(0);
    // Do NOT reset absolute encoder; just apply offset
}

void SwerveModule::setDesiredState(const WheelModuleState& state) {
    static int loopCount = 0;

    // Drive motor percent output
    double percentOutput = std::clamp(state.speed / MAX_SPEED, -1.0, 1.0);
    driveMotor.Set(percentOutput);

    // Steering motor control using absolute encoder
    double currentAngle = MathUtils::normalizeAngle(steerEncoder.GetPosition() - steerOffset);
    double targetAngle = optimizeSteerAngle(currentAngle, state.angle);
    double error = MathUtils::normalizeAngle(targetAngle - currentAngle);

    double kP = 0.25;
    double output = std::clamp(kP * error, -1.0, 1.0);
    steerMotor.Set(output);

    if (++loopCount % 10 == 0) {
        printf("Module[%d] drive=%.3f steerOut=%.3f target=%.3f curr=%.3f err=%.3f\n",
               driveMotor.GetDeviceId(), percentOutput, output,
               targetAngle, currentAngle, error);
    }
}

WheelModuleState SwerveModule::getCurrentState() const {
    WheelModuleState state;
    state.speed = (driveEncoder.GetVelocity() / 60.0) * 2.0 * PI * wheelRadius / gearRatio;
    state.angle = MathUtils::normalizeAngle(steerEncoder.GetPosition() - steerOffset);
    return state;
}

void SwerveModule::reset() {
    driveEncoder.SetPosition(0);
    // Do NOT reset absolute encoder
}

double SwerveModule::getDriveDistance() const {
    return driveEncoder.GetPosition() * 2.0 * PI * wheelRadius / gearRatio;
}

double SwerveModule::getSteerAngle() const {
    return MathUtils::normalizeAngle(steerEncoder.GetPosition() - steerOffset);
}

void SwerveModule::stop() {
    driveMotor.Set(0);
    steerMotor.Set(0);
}

double SwerveModule::optimizeSteerAngle(double currentAngle, double targetAngle) const {
    double delta = MathUtils::normalizeAngle(targetAngle - currentAngle);

    if (delta > PI/2) delta -= PI;
    if (delta < -PI/2) delta += PI;

    return MathUtils::normalizeAngle(currentAngle + delta);
}
