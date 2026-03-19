#pragma once

#include "pathing/geometry/vector2.hpp"

namespace pathing::path {

class PathPoint {

private:
    pathing::geometry::Vector2 position;
    double heading;
    double curvature;

public:
    PathPoint();

    PathPoint(
        const pathing::geometry::Vector2& position,
        double heading,
        double curvature
    );

    const pathing::geometry::Vector2& getPosition() const;

    double getHeading() const;
    double getCurvature() const;
};

}