#pragma once

#include "pathing/path/path.hpp"
#include "pathing/geometry/bezier_curve.hpp"
#include "pathing/geometry/bezier_line.hpp"
#include <memory>

namespace pathing::path {

class PathBuilder {

private:

    Path path;

public:

    PathBuilder();

    PathBuilder& addLine(
        const pathing::geometry::Vector2& start,
        const pathing::geometry::Vector2& end
    );

    PathBuilder& addBezier(
        const std::vector<pathing::geometry::Vector2>& points
    );

    Path build();

};

}