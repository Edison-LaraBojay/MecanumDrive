#pragma once

#include <memory>
#include "pathing/geometry/curve.hpp"

namespace pathing::path {

class PathSegment {

private:

    std::unique_ptr<pathing::geometry::Curve> curve;

public:

    PathSegment();

    PathSegment(std::unique_ptr<pathing::geometry::Curve> curve);

    template<typename T>
    PathSegment(std::unique_ptr<T> curve)
    {
        this->curve = std::move(curve);
    }

    pathing::geometry::Vector2 getPoint(double t) const;
    pathing::geometry::Vector2 getDerivative(double t) const;

    const pathing::geometry::Curve* getCurve() const;

};

}