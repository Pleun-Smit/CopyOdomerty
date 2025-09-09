#pragma once
#include "math/Vector2D.h"
#include <vector>

class Pose{
    public:
    Vector2D position;
    double heading; // rad
    std::vector<Vector2D> history;

    Pose(double x=0, double y=0, double heading_=0);

    void updatePose(double deltaDistance, double deltaHeading);

    void resetPose(double x=0, double y=0, double heading_ =0);
};