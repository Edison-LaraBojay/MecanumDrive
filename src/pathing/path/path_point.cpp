#include "pathing/path/path_point.hpp"

namespace pathing::path {

PathPoint::PathPoint()
    : position(0, 0), heading(0), curvature(0) {}

PathPoint::PathPoint(
    const pathing::geometry::Vector2& position,
    double heading,
    double curvature)
    : position(position), heading(heading), curvature(curvature) {}

const pathing::geometry::Vector2& PathPoint::getPosition() const {
    return position;
}

double PathPoint::getHeading() const {
    return heading;
}

double PathPoint::getCurvature() const {
    return curvature;
}

}