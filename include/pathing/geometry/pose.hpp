#pragma once

#include "pathing/geometry/vector2.hpp"

namespace pathing::geometry {

    class Pose
    {
        public:

        double x;
        double y;
        double heading;

        Pose();
        Pose(double x, double y, double heading);

        Vector2 position() const;

        double distanceTo(const Pose& other) const;

        Vector2 fieldToRobot(const Vector2& v) const;
        Vector2 robotToField(const Vector2& v) const;

        Vector2 errorTo(const Pose& target) const;

        void normalizeHeading();
    };
}