#pragma once

#include "pathing/path/path.hpp"
#include "pathing/geometry/pose.hpp"
#include "pathing/geometry/vector2.hpp"
#include "pathing/control/pidf_controller.hpp"

namespace pathing::control {

class PathFollower {

private:
    const pathing::path::Path* path;

    int currentSegment;

    double lookaheadDistance;

    PIDFController xPID;
    PIDFController yPID;

public:
    PathFollower();

    void setPath(const pathing::path::Path& path);

    pathing::geometry::Vector2 update(
        const pathing::geometry::Pose& robotPose
    );
};

}