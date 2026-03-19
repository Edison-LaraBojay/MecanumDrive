#include <main.h>
#include <Encoder.hpp>

struct ThreeWheelConstants { 

    // number of inches per tick of the encoder for forward movement
    double forwardTicktoInches = 1;

    // number of inches per tick of the encoder for lateral movement (strafing)
    double strafeTicksToInches = 1;

    // number of inches per tick of the encoder for turning
    double turnTicksToInches = 1;

    // the Y offset of the left encoder from the center of the robot
    double leftPodY = 1;

    // the Y offset of the Right encoder from the center of the robot
    double rightPodY = -1;

    // the X offset of the strafe encoder from the center of the robot
    double strafePodX = -2.5;

    // direction of the left encoder
    double leftEncoderDirection = Encoder::REVERSE;

    // direction direction of right encoder
    double rightEncoderDirection = Encoder::REVERSE;

    // direction of the strafe encoder
    double strafeEncoderDirection = Encoder::FORWARD;
};