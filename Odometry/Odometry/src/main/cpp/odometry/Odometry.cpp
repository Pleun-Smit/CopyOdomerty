#include "odometry/Odometry.h"
#include "math/MathUtils.h"
#include <cmath>  // only for cos/sin

Odometry::Odometry(EncoderSensor* encoder, GyroSensor* gyro)
    : encoderSensor(encoder), gyroSensor(gyro), pose(), lastDistance(0.0) {}

void Odometry::update() {
    // Read current encoder distance and gyro heading
    double currentDistance = encoderSensor->getValue();
    double currentHeading = gyroSensor->getValue();

    // Calculate delta distance since last update
    double deltaDistance = currentDistance - lastDistance;
    lastDistance = currentDistance;

    // Update the pose own math
    pose.updatePose(deltaDistance, currentHeading);

    //Normalize heading
    pose.heading = MathUtils::normalizeAngle(pose.heading);

    printf("Pose: x= %.2f, y= %.2f, heading= %.2f deg \n", pose.position.x, pose.position.y, MathUtils::radToDeg(pose.heading));
}

Pose Odometry::getPose() const {
    return pose;  // returns a copy of current pose
}

void Odometry::resetPose(double x, double y, double heading) {
    pose.resetPose(x, y, heading);
    lastDistance = encoderSensor->getValue(); // reset delta calculation
}
