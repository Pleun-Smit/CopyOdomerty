#pragma once
#include <rev/SparkMax.h>
#include <rev/RelativeEncoder.h>

class EncoderSensor {
public:
    // Constructor: motor object, wheel radius (m), optional gear ratio
    EncoderSensor(rev::spark::SparkMax& motor_, double wheelRadiusMeters, double gearRatio_ = 1.0);

    double getValue() const;   // distance in meters
    void reset();

private:
    rev::RelativeEncoder& encoder;  // reference to NEO encoder
    double wheelRadius;        // meters
    double gearRatio;          // motor-to-wheel
};
