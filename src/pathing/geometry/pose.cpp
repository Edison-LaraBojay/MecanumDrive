#include "pathing/geometry/pose.hpp"
#include "pathing/math/math_functions.hpp"

#include <cmath>

namespace pathing::geometry {
    using namespace pathing::math;

    Pose::Pose()
    {
        x = 0;
        y = 0;
        heading = 0;
    }

    Pose::Pose(double x, double y, double heading)
    {
        this->x = x;
        this->y = y;
        this->heading = heading;

        normalizeHeading();
    }

    Vector2 Pose::position() const
    {
        return Vector2(x, y);
    }

    double Pose::distanceTo(const Pose& other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;

        return std::sqrt(dx*dx + dy*dy);
    }

    Vector2 Pose::fieldToRobot(const Vector2& v) const
    {
        double cosH = std::cos(-heading);
        double sinH = std::sin(-heading);

        double rx = v.x * cosH - v.y * sinH;
        double ry = v.x * sinH + v.y * cosH;

        return Vector2(rx, ry);
    }

    Vector2 Pose::robotToField(const Vector2& v) const
    {
        double cosH = std::cos(heading);
        double sinH = std::sin(heading);

        double fx = v.x * cosH - v.y * sinH;
        double fy = v.x * sinH + v.y * cosH;

        return Vector2(fx, fy);
    }

    Vector2 Pose::errorTo(const Pose& target) const 
    {
        Vector2 fieldError = target.position() - position();

        return fieldToRobot(fieldError);
    }

    void Pose::normalizeHeading()
    {
        heading = normalizeAngleSigned(heading);
    }
}