#include "pathing/geometry/bezier_point.hpp"

namespace pathing::geometry {

BezierPoint::BezierPoint()
{
    curve = nullptr;
    t = 0;
}

BezierPoint::BezierPoint(const Curve* curve, double t)
{
    this->curve = curve;
    this->t = t;
}

Vector2 BezierPoint::getPoint() const
{
    return curve->getPoint(t);
}

Vector2 BezierPoint::getDerivative() const
{
    return curve->getDerivative(t);
}

double BezierPoint::getT() const
{
    return t;
}

}