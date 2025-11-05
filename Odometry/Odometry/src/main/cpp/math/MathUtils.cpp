// make unit tests for this
#define _USE_MATH_DEFINES
#include "math/MathUtils.h"
#include <cmath>

double MathUtils::radToDeg(double radians){
    return radians * 180.0 / OperatorConstants::PI;
}

double MathUtils::degToRad(double degrees){
    return degrees * OperatorConstants::PI / 180.0;
}

double MathUtils::normalizeAngle(double radians){
    while (radians > OperatorConstants::PI) radians -= 2 * OperatorConstants::PI;
    while (radians <= -OperatorConstants::PI) radians += 2 * OperatorConstants::PI;
    return radians;
}