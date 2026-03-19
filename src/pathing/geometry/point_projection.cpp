#include "pathing/geometry/point_projection.hpp"

namespace pathing::geometry {

double PointProjection::project(
    const Curve& curve,
    const Vector2& point,
    double t,
    int iterations)
{
    for(int i = 0; i < iterations; i++)
    {
        Vector2 curvePoint = curve.getPoint(t);
        Vector2 derivative = curve.getDerivative(t);

        Vector2 diff = curvePoint - point;

        double numerator = diff.dot(derivative);
        double denominator = derivative.magnitudeSquared();

        if(denominator == 0)
            break;

        t -= numerator / denominator;

        if(t < 0) t = 0;
        if(t > 1) t = 1;
    }

    return t;
}

}