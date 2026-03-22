#include "pathing/control/path_follower.hpp"

#include "pathing/control/error_calculator.hpp"
#include "pathing/control/follower_constants.hpp"

#include "pathing/geometry/point_projection.hpp"

#include <cmath>

namespace pathing::control {

using namespace pathing::geometry;
using namespace pathing::path;


PathFollower::PathFollower()
    : xPID(
        FollowerConstants::FORWARD_kP,
        FollowerConstants::FORWARD_kI,
        FollowerConstants::FORWARD_kD
      ),
      yPID(
        FollowerConstants::LATERAL_kP,
        FollowerConstants::LATERAL_kI,
        FollowerConstants::LATERAL_kD
      )
{
    path = nullptr;
    currentSegment = 0;
    lookaheadDistance =
        FollowerConstants::LOOKAHEAD_DISTANCE;
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


    const PathSegment& segment =
        path->getSegment(currentSegment);


    Vector2 robotPos(
        robotPose.x,
        robotPose.y
    );


    // project robot onto curve
    double t = PointProjection::project(
        *segment.getCurve(),
        robotPos,
        0.5,
        8
    );


    // lookahead
    double lookaheadT =
        t + 0.1;


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


    // path geom from lookahead
    Vector2 targetPos =
        segment.getPoint(lookaheadT);

    Vector2 tangent =
        segment.getDerivative(lookaheadT);


    // path heading from tangent
    double heading =
        std::atan2(
            tangent.y,
            tangent.x
        );


    Pose targetPose(
        targetPos.x,
        targetPos.y,
        heading
    );


    // path-relative errors
    double forwardError =
        ErrorCalculator::xError(
            targetPose,
            robotPose
        );

    double lateralError =
        ErrorCalculator::yError(
            targetPose,
            robotPose
        );


    // PID corrections
    double dt = 0.01;

    double forwardCmd =
        xPID.update(
            forwardError,
            0,
            dt
        );

    double lateralCmd =
        yPID.update(
            lateralError,
            0,
            dt
        );


    // path relative command to global velocity
    double cosH =
        std::cos(heading);

    double sinH =
        std::sin(heading);


    double vx =
        forwardCmd * cosH -
        lateralCmd * sinH;

    double vy =
        forwardCmd * sinH +
        lateralCmd * cosH;


    // vel magnitude clamping
    double maxVel =
        FollowerConstants::MAX_VELOCITY;

    double mag =
        std::sqrt(vx*vx + vy*vy);

    if(mag > maxVel && mag > 0.0)
    {
        vx *= maxVel / mag;
        vy *= maxVel / mag;
    }


    return Vector2(vx, vy);
}


// heading contorl
double PathFollower::getOmega(const Pose& robotPose)
{
    if(path == nullptr || path->size() == 0)
        return 0.0;

    const PathSegment& segment =
        path->getSegment(currentSegment);

    Vector2 robotPos(
        robotPose.x,
        robotPose.y
    );

    double t = PointProjection::project(
        *segment.getCurve(),
        robotPos,
        0.5,
        8
    );

    double lookaheadT = t + 0.1;

    if(lookaheadT > 1.0)
        lookaheadT = 1.0;

    Vector2 tangent =
        segment.getDerivative(lookaheadT);

    double targetHeading =
        std::atan2(
            tangent.y,
            tangent.x
        );

    double headingError =
        targetHeading - robotPose.heading;

    // wrap to [-pi, pi]
    while(headingError > M_PI) headingError -= 2*M_PI;
    while(headingError < -M_PI) headingError += 2*M_PI;

    return FollowerConstants::HEADING_kP * headingError;
}


Pose PathFollower::getTargetPose(const Pose& robotPose)
{
    if(path == nullptr || path->size() == 0)
        return robotPose;

    const PathSegment& segment =
        path->getSegment(currentSegment);

    Vector2 robotPos(
        robotPose.x,
        robotPose.y
    );

    double t = PointProjection::project(
        *segment.getCurve(),
        robotPos,
        0.5,
        8
    );

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

    Vector2 targetPos =
        segment.getPoint(lookaheadT);

    Vector2 tangent =
        segment.getDerivative(lookaheadT);

    double heading =
        std::atan2(
            tangent.y,
            tangent.x
        );

    return Pose(
        targetPos.x,
        targetPos.y,
        heading
    );
}

}