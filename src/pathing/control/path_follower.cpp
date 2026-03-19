#include "pathing/control/path_follower.hpp"
#include "pathing/geometry/point_projection.hpp"

#include <cmath>

namespace pathing::control {

using namespace pathing::geometry;
using namespace pathing::path;

PathFollower::PathFollower()
    : xPID(1.0, 0.0, 0.1),   // tune later
      yPID(1.0, 0.0, 0.1)
{
    path = nullptr;
    currentSegment = 0;
    lookaheadDistance = 6.0;
}

void PathFollower::setPath(const Path& p)
{
    path = &p;
    currentSegment = 0;

    xPID.reset();
    yPID.reset();
}

Vector2 PathFollower::update(const Pose& robotPose)
{
    if(path == nullptr || path->size() == 0)
        return Vector2(0, 0);

    const PathSegment& segment = path->getSegment(currentSegment);

    Vector2 robotPos(robotPose.x, robotPose.y);

    //Project robot onto curve
    double t = PointProjection::project(
        *segment.getCurve(),
        robotPos,
        0.5,
        8
    );

    //Lookahead along curve
    double lookaheadT = t + 0.1;

    if(lookaheadT > 1.0)
    {
        if(currentSegment < path->size() - 1)
        {
            currentSegment++;
            lookaheadT = 0.0;
        }
        else
        {
            lookaheadT = 1.0;
        }
    }

    Vector2 target = segment.getPoint(lookaheadT);

    //PID control
    double dt = 0.01; // timestep

    double vx = xPID.update(target.x, robotPose.x, dt);
    double vy = yPID.update(target.y, robotPose.y, dt);

    //Clamp max velocity
    double maxVel = 10.0;

    double mag = std::sqrt(vx * vx + vy * vy);
    if (mag > maxVel && mag > 0.0) {
        vx *= maxVel / mag;
        vy *= maxVel / mag;
    }

    return Vector2(vx, vy);
}

}