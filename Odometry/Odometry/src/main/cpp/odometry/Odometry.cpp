#include "odometry/Odometry.h"
#include "math/MathUtils.h"
#include <cmath>
#include <cstdio>

Odometry::Odometry(std::array<SwerveModule*, 4> modules_, IGyroSensor* gyro_)
    : modules(modules_), gyro(gyro_), pose() 
{
    // Record initial distances
    for (int i = 0; i < 4; i++) {
        lastDistances[i] = modules[i]->getDriveDistance();
    }
    lastHeading = gyro->getHeading();
}

void Odometry::update() {
    // --- 1. Read gyro ---
    double currentHeading = gyro->getHeading();

    // --- 2. Compute robot-relative delta pos ---
    Vector2D robotDelta(0.0, 0.0);
    
    for (int i = 0; i < 4; i++) {
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
    robotDelta.x /= 4.0;
    robotDelta.y /= 4.0;

    // --- 3. Convert from robot frame to field frame ---
    double heading = currentHeading; // radians
    Vector2D fieldDelta(
        robotDelta.x * cos(heading) - robotDelta.y * sin(heading),
        robotDelta.x * sin(heading) + robotDelta.y * cos(heading)
    );

    // --- 4. Integrate into pose ---
    pose.position.x += fieldDelta.x;
    pose.position.y += fieldDelta.y;
    pose.heading = heading;

    // --- 5. Debug output ---
    printf("Pose: x=%.2f, y=%.2f, heading=%.2f deg\n",
           pose.position.x, pose.position.y,
           MathUtils::radToDeg(heading));
           
    // // --- 2. Compute average delta distance from all 4 wheels ---
    // double avgDelta = 0.0;
    // for (int i = 0; i < 4; i++) {
    //     double dist = modules[i]->getDriveDistance();
    //     double delta = dist - lastDistances[i];
    //     avgDelta += delta;
    //     lastDistances[i] = dist;
    // }
    // avgDelta /= 4.0;

    // // --- 3. Compute change in heading ---
    // double deltaHeading = currentHeading - lastHeading;
    // lastHeading = currentHeading;

    // // --- 4. Update pose (simple integration) ---
    // pose.updatePose(avgDelta, deltaHeading);

    // // --- 5. Debug print ---
    // printf("Pose: x=%.2f, y=%.2f, heading=%.2f deg\n",
    //        pose.position.x, pose.position.y,
    //        MathUtils::radToDeg(pose.getHeading()));

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
