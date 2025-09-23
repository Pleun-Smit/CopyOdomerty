#include "odometry/Pose.h"
#include "math/MathUtils.h"
#include <cmath>

Pose::Pose(double x, double y, double heading_) 
    : position(x, y), heading(heading_), historyIndex(0) {}

void Pose::updatePose(double deltaDistance, double deltaHeading) {
    heading += deltaHeading;
    //Normalize heading [Pi, -Pi or 180 to -180] (gradencirkel)
    MathUtils::normalizeAngle(heading);

    //Update posistion based on heading
    position.x += deltaDistance * std::cos(heading);
    position.y += deltaDistance * std::sin(heading);

    //Store position in history
    history[historyIndex] = position;
    historyIndex = (historyIndex + 1) % MAX_HISTORY;
}

void Pose::resetPose(double x, double y, double heading_) {
    position = Vector2D(x, y);
    heading = heading_;

    for(int i = 0; i < MAX_HISTORY; i++){
        history[i] = position;
    }

    historyIndex = 0;
}

double Pose::getHeading() const{
    return heading;
}
