#pragma once
#include "Sensor.h"
#include <studica/AHRS.h>
#include <frc/SPI.h>

class GyroSensor : public Sensor{
public:
    GyroSensor();
    double getValue() override;  // heading in radians
    double getHeading();
    void reset() override;
private:
    studica::AHRS ahrs;  // navX gyro
};

