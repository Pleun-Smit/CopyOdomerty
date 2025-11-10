#pragma once
#include "hardware/Interfaces/IEncoderSensor.h"

class MEncoderSensor : public IEncoderSensor {
private:
    double value;
public:
    MEncoderSensor(double initialValue);
    double getValue() const override; // Distance in meters
    void reset() override;
};