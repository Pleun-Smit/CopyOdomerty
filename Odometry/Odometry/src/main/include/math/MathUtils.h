#pragma once
class MathUtils {
    public:
    // Converts radians to degrees
    static double radToDeg(double radians);

    // Converts degrees to radians
    static double degToRad(double degrees);

    // Normalizes angle to [-pi, pi]/[-180, +180]
    static double normalizeAngle(double radians);
};
