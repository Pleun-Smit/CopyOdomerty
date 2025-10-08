#include "hardware/GyroSensor.h"
#include <numbers>


GyroSensor::GyroSensor() : ahrs(studica::AHRS::kMXP_SPI, studica::AHRS::k50Hz)
{
    ahrs.Reset();
}

double GyroSensor::getValue() const {
    // Convert degrees to radians
    return ahrs.GetAngle() * std::numbers::pi / 180.0;
}

double GyroSensor::getHeading() const{
    return getValue();
}

void GyroSensor::reset() {
    ahrs.Reset();
}

double GyroSensor::getRate() const {
    // Return rotation rate in radians per second
    return ahrs.GetRate() * std::numbers::pi / 180.0;
}
