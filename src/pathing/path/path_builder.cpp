#include "pathing/path/path_builder.hpp"

namespace pathing::path {

using namespace pathing::geometry;

PathBuilder::PathBuilder() {}

PathBuilder& PathBuilder::addLine(
    const Vector2& start,
    const Vector2& end)
{
    auto line = std::make_unique<BezierLine>(start, end);
    path.addSegment(PathSegment(std::move(line)));

    return *this;
}

PathBuilder& PathBuilder::addBezier(
    const std::vector<Vector2>& points)
{
    auto curve = std::make_unique<BezierCurve>(points);

    path.addSegment(PathSegment(std::move(curve)));

    return *this;
}

Path PathBuilder::build()
{
    return path;
}

}