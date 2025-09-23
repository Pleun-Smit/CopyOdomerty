// make unit tests for this
#include "math/MathUtils.h"
#include <cmath>

double MathUtils::radToDeg(double radians){
    return radians * 180.0 / M_PI;
}

double MathUtils::degToRad(double degrees){
    return degrees * M_PI / 180.0;
}

double MathUtils::normalizeAngle(double radians){
    while (radians > M_PI) radians -= 2 * M_PI;
    while (radians <= -M_PI) radians += 2 * M_PI;
    return radians;
}