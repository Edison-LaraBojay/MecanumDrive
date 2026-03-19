#include "pathing/path/path_segment.hpp"

namespace pathing::path {

using namespace pathing::geometry;

PathSegment::PathSegment() {}

PathSegment::PathSegment(std::unique_ptr<Curve> curve)
{
    this->curve = std::move(curve);
}

Vector2 PathSegment::getPoint(double t) const
{
    return curve->getPoint(t);
}

Vector2 PathSegment::getDerivative(double t) const
{
    return curve->getDerivative(t);
}

const Curve* PathSegment::getCurve() const
{
    return curve.get();
}

}