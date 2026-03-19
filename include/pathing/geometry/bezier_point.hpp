#pragma once

#include "pathing/geometry/curve.hpp"

namespace pathing::geometry {

class BezierPoint {

private:

    const Curve* curve;
    double t;

public:

    BezierPoint();
    BezierPoint(const Curve* curve, double t);

    Vector2 getPoint() const;
    Vector2 getDerivative() const;

    double getT() const;

};

}