#pragma once
#include "math/Vector2D.h"
#include <vector>

class Pose{
    public:
    Vector2D position;
    double heading; // rad
    static const int MAX_HISTORY = 100;
    Vector2D history[MAX_HISTORY];
    int historyIndex;

    Pose(double x=0, double y=0, double heading_=0);

    double getHeading() const;

    void updatePose(double deltaDistance, double deltaHeading);

    void resetPose(double x=0, double y=0, double heading_ =0);
};