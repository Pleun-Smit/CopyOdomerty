#include "hardware/GyroSensor.h"
#include <numbers>

GyroSensor::GyroSensor():gyro(){
    	gyro.Reset(); // standart WPILib gyro
}

double GyroSensor::getValue() const{
    return gyro.GetAngle() * std::numbers::pi / 180.0; //conversion to radians
}

void GyroSensor::reset(){
    gyro.Reset();
}