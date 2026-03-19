#pragma once

#include "pathing/geometry/curve.hpp"
#include "pathing/geometry/vector2.hpp"

namespace pathing::geometry {

class PointProjection {

public:

    static double project(
        const Curve& curve,
        const Vector2& point,
        double initialT = 0.5,
        int iterations = 10
    );

};

}