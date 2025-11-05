#include "odometry/Odometry.h"
#include "math/MathUtils.h"
#include <cmath>
#include <cstdio>

Odometry::Odometry(std::array<SwerveModule*, numberOfModules> modules_, IGyroSensor* gyro_)
    : modules(modules_), gyro(gyro_), pose() 
{
    // Record initial distances
    for (int i = 0; i < numberOfModules; i++) {
        lastDistances[i] = modules[i]->getDriveDistance();
    }
    lastHeading = gyro->getHeading();
}

void Odometry::update() {
    // --- 1. Read gyro --- (absolute heading in rad)
    double currentHeading = gyro->getHeading();

    // Compute change in heading (wrapped) and midpoint heading
    double deltaHeading = MathUtils::normalizeAngle(currentHeading - lastHeading);
    double midHeading = lastHeading + deltaHeading * 0.5;


    // --- 2. Compute robot-relative delta pos ---
    Vector2D robotDelta(0.0, 0.0);
    
    for (int i = 0; i < numberOfModules; i++) {
        double dist = modules[i]->getDriveDistance();
        double delta = dist - lastDistances[i];
        lastDistances[i] = dist;

        // Each wheel's movement direction = its steering angle
        double angle = modules[i]->getCurrentState().angle;

        // Convert to vector
        robotDelta.x += delta * cos(angle);
        robotDelta.y += delta * sin(angle);
    }

    // Average across modules
    robotDelta.x /= static_cast<double>(numberOfModules);
    robotDelta.y /= static_cast<double>(numberOfModules);

    // --- 3. Convert from robot frame to field frame ---
    double cosT = std::cos(midHeading);
    double sinT = std::sin(midHeading);
    Vector2D fieldDelta(
        robotDelta.x * cosT - robotDelta.y * sinT,
        robotDelta.x * sinT + robotDelta.y * cosT
    );

    // --- 4. Integrate into pose ---
    pose.position.x += fieldDelta.x;
    pose.position.y += fieldDelta.y;
    pose.heading = currentHeading;

    lastHeading = currentHeading;

    // --- 5. Debug output ---
    printf("Pose: x=%.2f, y=%.2f, heading=%.2f deg\n",
           pose.position.x, pose.position.y,
           MathUtils::radToDeg(pose.heading));

}

Pose Odometry::getPose() const {
    return pose;   // copy
}

void Odometry::resetPose(double x, double y, double heading) {
    pose.resetPose(x, y, heading);
    for(int i =0; i<4; i++){
        lastDistances[i] = modules[i]->getDriveDistance();
    }
    lastHeading  = gyro->getHeading();
}
