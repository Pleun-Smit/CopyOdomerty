#include "odometry/Pose.h"
#include <cmath>

Pose::Pose(double x, double y, double heading_) 
    : position(x, y), heading(heading_) {}

void Pose::updatePose(double deltaDistance, double deltaHeading) {
    heading += deltaHeading;
    position.x += deltaDistance * std::cos(heading);
    position.y += deltaDistance * std::sin(heading);
    history.push_back(position); // log pose
}

void Pose::resetPose(double x, double y, double heading_) {
    position = Vector2D(x, y);
    heading = heading_;
    history.clear();
}
