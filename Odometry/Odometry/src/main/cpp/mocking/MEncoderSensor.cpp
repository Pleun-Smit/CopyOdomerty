#include "mocking/MEncoderSensor.h"

MEncoderSensor::MEncoderSensor(double initialValue) : value(initialValue) {}

double MEncoderSensor::getValue() const {
    return value;
}

void MEncoderSensor::reset(){}

