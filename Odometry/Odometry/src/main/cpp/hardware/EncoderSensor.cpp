#include "hardware/EncoderSensor.h"
#include <cmath>

EncoderSensor::EncoderSensor(rev::spark::SparkMax& motor_, double wheelRadiusMeters, double gearRatio_, bool isSteering)
    : encoder(motor_.GetEncoder()), wheelRadius(wheelRadiusMeters), gearRatio(gearRatio_), steeringMode(isSteering) 
{
    encoder.SetPosition(0); // start counting from 0 rotations
}

double EncoderSensor::getValue() const {
    if (steeringMode) {
        // encoder returns motor rotations -> convert to wheel radians
        // motor rotations * 2π = motor radians; divide by gear ratio to get wheel radians
        return encoder.GetPosition() * 2.0 * OperatorConstants::PI / gearRatio;
    } else {
        // drive encoder: rotations -> meters
        return encoder.GetPosition() * 2.0 * OperatorConstants::PI * wheelRadius / gearRatio;
    }
}

void EncoderSensor::reset() {
    encoder.SetPosition(0);
}
