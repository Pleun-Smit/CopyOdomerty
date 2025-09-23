#pragma once
#include "odometry/Pose.h"
#include "hardware/IEncoderSensor.h"
#include "hardware/IGyroSensor.h"

class Odometry {
    private:
        Pose pose;
        double lastDistance;
        IGyroSensor& gyro;
        std::vector<IEncoderSensor*> encoders;

        // For tracking previous values
        std::vector<double> prevEncoderDistances;
        double prevHeading;

    public:
        Odometry(IGyroSensor& gyro, const std::vector<IEncoderSensor*>& encoders);

        void update();

        const Pose& getPose() const;

        void reset(double x = 0.0, double y = 0.0, double heading = 0.0);
};