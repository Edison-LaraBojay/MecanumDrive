#include "pathing/geometry/bezier_line.hpp"

namespace pathing::geometry {

BezierLine::BezierLine() {}

BezierLine::BezierLine(const Vector2& start, const Vector2& end)
{
    this->start = start;
    this->end = end;
}

Vector2 BezierLine::getPoint(double t) const
{
    return start * (1 - t) + end * t;
}   

Vector2 BezierLine::getDerivative(double) const
{
    return end - start;
}

}