#include <main.h>
#include <ThreeWheelLocalizer.hpp>
#include <ThreeWheelConstants.hpp>
#include <Encoder.hpp>
#include <pose.hpp>
#include <NanoTimer.hpp>

ThreeWheelLocalizer::ThreeWheelLocalizer(int leftPort, int rightPort, int strafePort, const ThreeWheelConstants& constants, pathing::geometry::Pose setStartPose) : leftEncoder(leftPort), rightEncoder(rightPort), strafeEncoder(strafePort), startPose(setStartPose){
    FORWARD_TICKS_TO_INCHES = constants.forwardTicktoInches;
    STRAFE_TICKS_TO_INCHES = constants.strafeTicksToInches;
    TURN_TICKS_TO_RADIANS = constants.turnTicksToInches;

    leftEncoderPose = pathing::geometry::Pose(0, constants.leftPodY, 0);
    rightEncoderPose = pathing::geometry::Pose(0, constants.rightPodY, 0);
    strafeEncoderPose = pathing::geometry::Pose(constants.strafePodX, 0, M_PI_2);


    deltaTimeNano = 1; 
    displacementPose = pathing::geometry::Pose();
    currentVelocity = pathing::geometry::Pose();
    totalHeading = 0;

    timer = NanoTimer();
    resetEncoders();

}