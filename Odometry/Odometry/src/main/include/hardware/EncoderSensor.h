#pragma once
#include <rev/SparkMax.h>
#include <rev/RelativeEncoder.h>
#include "Interfaces/IEncoderSensor.h"
#include "Constants.h"

class EncoderSensor : public IEncoderSensor {
public:
    // Constructor: motor object, wheel radius (m), optional gear ratio
    EncoderSensor(rev::spark::SparkMax& motor_, double wheelRadiusMeters, double gearRatio_ = 1.0, bool isSteering = false);

    double getValue() const override;// distance in meters
    void reset() override;

private:
    rev::RelativeEncoder& encoder;  // reference to NEO encoder
    double wheelRadius;        // meters
    double gearRatio;          // motor-to-wheel
    bool steeringMode;
};
