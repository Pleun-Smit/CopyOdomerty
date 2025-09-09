#pragma once // make sure that this file is only included once
#include "Sensor.h"
#include <frc/Encoder.h>
class EncoderSensor : public Sensor{
private: 
    int channelA, channelB;
    frc::Encoder encoder;
public:
    EncoderSensor(int channelA, int channelB);
    double getValue() const override;
    void reset() override;
};