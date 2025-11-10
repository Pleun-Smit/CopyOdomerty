#include "mocking/MGyroSensor.h"

MockGyroSensor::MockGyroSensor(double initialAngle, double initialRate) : angle(initialAngle), rate(initialRate) {}

double MockGyroSensor::getHeading() const {
    return angle;
}

void MockGyroSensor::getRate(double Rate) {
    this->rate = rate;
}

void MockGyroSensor::reset(){}