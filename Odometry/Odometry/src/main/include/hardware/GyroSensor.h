#pragma once
#include "Sensor.h"
#include <frc/ADXRS450_Gyro.h> //WPIlib for gyro

class GyroSensor : public Sensor{
    private:
        frc::ADXRS450_Gyro gyro;
    public:
        GyroSensor();
        double getValue() const override;
        void reset() override;
};
