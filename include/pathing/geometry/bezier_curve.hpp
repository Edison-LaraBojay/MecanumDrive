#pragma once

#include <vector>
#include "pathing/geometry/vector2.hpp"

namespace pathing::geometry {

class BezierCurve {

private:

    std::vector<Vector2> controlPoints;
    int order;

public:

    BezierCurve();
    BezierCurve(const std::vector<Vector2>& points);

    Vector2 getPoint(double t) const;
    Vector2 getDerivative(double t) const;

    int getOrder() const;
    const std::vector<Vector2>& getControlPoints() const;

};

}