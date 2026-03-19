#pragma once
#include <cmath>
namespace pathing::math {
    double clamp(double num, double lower, double upper);
    double normalizeAngle(double angleRadians);
    double normalizeAngleSigned(double angleRadians);
    double getSmallestAngleDifference(double one, double two);
    double getTurnDirection(double startHeading, double endHeading);
    bool roughlyEquals(double one, double two, double accuracy);
    bool roughlyEquals(double one, double two);
    double scale(double n, double x1, double x2, double y1, double y2);
}

