#pragma once
#include "hardware/Interfaces/IGyroSensor.h"
#include "Constants.h"
#include <studica/AHRS.h>
#include <frc/SPI.h>
#include <cmath>

class GyroSensor : public IGyroSensor{
public:
    GyroSensor();
    double getValue() const;  // heading in radians
    double getHeading() const override;  // radians
    double getRate() const override;     // radians per second
    void reset() override;
private:
    mutable studica::AHRS ahrs;  // navX gyro
};

