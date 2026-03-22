#include "pathing/localization/pose_tracker.hpp"

#include <cmath>

namespace pathing::localization {

using namespace pathing::geometry;


PoseTracker::PoseTracker()
    : pose(0, 0, 0)
{
}


PoseTracker::PoseTracker(
    const Pose& initialPose
)
    : pose(initialPose)
{
}


void PoseTracker::setPose(
    const Pose& newPose
)
{
    pose = newPose;
}


const Pose& PoseTracker::getPose() const
{
    return pose;
}


/*
 * holonomic integration
 */
void PoseTracker::update(
    const Vector2& velocity,
    double angularVelocity,
    double dt
)
{
    pose.x += velocity.x * dt;
    pose.y += velocity.y * dt;
    pose.heading += angularVelocity * dt;
}

}