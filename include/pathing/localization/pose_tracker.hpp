#pragma once

#include "pathing/geometry/pose.hpp"
#include "pathing/geometry/vector2.hpp"

namespace pathing::localization {

class PoseTracker {

private:
    pathing::geometry::Pose pose;

public:

    PoseTracker();

    PoseTracker(
        const pathing::geometry::Pose& initialPose
    );

    void setPose(
        const pathing::geometry::Pose& pose
    );

    const pathing::geometry::Pose& getPose() const;


    /*
     * Integrates velocity wrt time
     */
    void update(
        const pathing::geometry::Vector2& velocity,
        double angularVelocity,
        double dt
    );

};

}