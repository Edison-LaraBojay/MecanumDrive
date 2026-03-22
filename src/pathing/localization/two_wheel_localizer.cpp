#include "pathing/localization/two_wheel_localizer.hpp"

#include "pathing/math/math_functions.hpp"

#include <cmath>

namespace pathing::localization {

using namespace pathing::geometry;


TwoWheelLocalizer::TwoWheelLocalizer(
    EncoderSource* forwardEncoder,
    EncoderSource* strafeEncoder,
    IMUSource* imu,
    double forwardTicksToDistance,
    double strafeTicksToDistance,
    double forwardPodY,
    double strafePodX
)
    :
    forwardEncoder(forwardEncoder),
    strafeEncoder(strafeEncoder),
    imu(imu),

    forwardTicksToDistance(forwardTicksToDistance),
    strafeTicksToDistance(strafeTicksToDistance),

    forwardPodY(forwardPodY),
    strafePodX(strafePodX),

    startPose(0,0,0),
    displacementPose(0,0,0),
    velocityPose(0,0,0),

    prevForwardTicks(0),
    prevStrafeTicks(0),

    previousHeading(0),
    totalHeading(0)
{
}


void TwoWheelLocalizer::reset()
{
    forwardEncoder->reset();
    strafeEncoder->reset();
    imu->reset();

    displacementPose = Pose(0,0,0);
    velocityPose = Pose(0,0,0);

    prevForwardTicks = 0;
    prevStrafeTicks = 0;

    previousHeading = imu->getHeading();
    totalHeading = 0;
}


const Pose& TwoWheelLocalizer::getPose() const
{
    static Pose pose;

    pose =
        startPose + //fix this teehee
        displacementPose;

    return pose;
}


void TwoWheelLocalizer::setPose(
    const Pose& pose
)
{
    startPose = pose;

    displacementPose =
        Pose(0,0,0);

    reset();
}


Pose TwoWheelLocalizer::getVelocity() const
{
    return velocityPose;
}


double TwoWheelLocalizer::getTotalHeading() const
{
    return totalHeading;
}


void TwoWheelLocalizer::update(double dt)
{

    /*
     * read sensors
     */
    double forwardTicks =
        forwardEncoder->getPosition();

    double strafeTicks =
        strafeEncoder->getPosition();

    double heading =
        imu->getHeading();


    /*
     * compute deltas
     */
    double dForward =
        (forwardTicks - prevForwardTicks)
        * forwardTicksToDistance;

    double dStrafe =
        (strafeTicks - prevStrafeTicks)
        * strafeTicksToDistance;

    double dTheta =
        math::normalizeAngle(
            heading - previousHeading
        );


    prevForwardTicks = forwardTicks;
    prevStrafeTicks = strafeTicks;
    previousHeading = heading;


    /*
     * correct for pod offset
     */
    dForward -= forwardPodY * dTheta;
    dStrafe  -= strafePodX * dTheta;


    /*
     * pose exponential integration
     */
    double sinTerm;
    double cosTerm;

    if (std::abs(dTheta) < 1e-6)
    {
        sinTerm =
            1.0 - (dTheta*dTheta)/6.0;

        cosTerm =
            dTheta/2.0;
    }
    else
    {
        sinTerm =
            std::sin(dTheta)/dTheta;

        cosTerm =
            (1 - std::cos(dTheta))/dTheta;
    }


    double dx =
        sinTerm * dForward -
        cosTerm * dStrafe;

    double dy =
        cosTerm * dForward +
        sinTerm * dStrafe;


    /*
     * rotate into global frame
     */
    double cosH =
        std::cos(displacementPose.heading);

    double sinH =
        std::sin(displacementPose.heading);


    displacementPose.x +=
        dx*cosH - dy*sinH;

    displacementPose.y +=
        dx*sinH + dy*cosH;

    displacementPose.heading +=
        dTheta;


    /*
     * compute velocity
     */
    velocityPose =
        Pose(
            dx/dt,
            dy/dt,
            dTheta/dt
        );


    totalHeading += dTheta;
}

}