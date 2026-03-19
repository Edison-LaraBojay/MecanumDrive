#include <vector>
#include "pathing/geometry/bezier_curve.hpp"

namespace pathing::geometry {

BezierCurve::BezierCurve() : order(0) {}

BezierCurve::BezierCurve(const std::vector<Vector2>& points)
{
    controlPoints = points;
    order = points.size() - 1;
}

Vector2 BezierCurve::getPoint(double t) const
{
    std::vector<Vector2> temp = controlPoints;

    for (int k = 1; k <= order; k++)
    {
        for (int i = 0; i <= order - k; i++)
        {
            temp[i] = temp[i] * (1 - t) + temp[i + 1] * t;
        }
    }

    return temp[0];
}

Vector2 BezierCurve::getDerivative(double t) const
{
    if (order <= 0)
        return Vector2(0,0);

    std::vector<Vector2> derivativePoints;

    for (int i = 0; i < order; i++)
    {
        derivativePoints.push_back(
            (controlPoints[i+1] - controlPoints[i]) * order
        );
    }

    BezierCurve derivativeCurve(derivativePoints);

    return derivativeCurve.getPoint(t);
}

int BezierCurve::getOrder() const
{
    return order;
}

const std::vector<Vector2>& BezierCurve::getControlPoints() const
{
    return controlPoints;
}

}