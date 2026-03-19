#pragma once

#include "pathing/geometry/vector2.hpp"

namespace pathing::geometry {

class Curve {
public:

    virtual ~Curve() = default;

    virtual Vector2 getPoint(double t) const = 0;
    virtual Vector2 getDerivative(double t) const = 0;

};

}