#pragma once
// For linear position/velocity sensors (e.g., wheel encoders)

class IEncoderSensor {
public:
    virtual double getValue() const = 0; // Distance in meters
    virtual void reset() = 0;
    virtual ~IEncoderSensor() = default;
};
