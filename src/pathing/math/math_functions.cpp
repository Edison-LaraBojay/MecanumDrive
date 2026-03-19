#include "pathing/math/math_functions.hpp"
#include <stdexcept>
namespace pathing::math {
    double clamp(double num, double lower, double upper)
    {
        if (num < lower) return lower;
        if (num > upper) return upper;
        return num;
    }

    double normalizeAngle(double angleRadians)
    {
        double angle = std::fmod(angleRadians, 2*M_PI);

        if (angle < 0)
        angle += 2*M_PI;

        return angle;
    }

    double normalizeAngleSigned(double angleRadians)
    {
        double angle = normalizeAngle(angleRadians);

        if (angle >= M_PI)
        angle -= 2*M_PI;

        return angle;
    }

    double getSmallestAngleDifference(double one, double two) 
    {
        return std::min(
            normalizeAngle(one - two),
            normalizeAngle(two - one)
        );
    }

    double getTurnDirection(double startHeading, double endHeading)
    {
        double diff = normalizeAngle(endHeading - startHeading);

        if (diff >= 0 && diff <= M_PI)
        return 1;

        return -1;
    }

    bool roughlyEquals(double one, double two, double accuracy)
    {
        return (one < two + accuracy && one > two - accuracy);

    }

    bool roughlyEquals(double one, double two)
    {
        return roughlyEquals(one, two, 0.0001);
    }

    double scale(double n, double x1, double x2, double y1, double y2)
    {
        if (x2 - x1 == 0)
        throw std::invalid_argument("Input range cannot be zero.");

        return (n - x1) * (y2 - y1) / (x2 - x1) + y1;
    }
}