#pragma once
#include "odometry/Pose.h"
#include "hardware/EncoderSensor.h"
#include "hardware/GyroSensor.h"

class Odometry {
    private:
        EncoderSensor* encoderSensor;
        GyroSensor* gyroSensor;
        Pose pose;
        double lastDistance;

    public:
        Odometry(EncoderSensor* encoder, GyroSensor* gyro);

        void update();

        Pose getPose() const;

        void resetPose(double x = 0, double y = 0, double heading = 0);
};