#include <pose.hpp>
#include <Encoder.hpp>

class ThreeWheelLocalizer { 

    public:
    ThreeWheelLocalizer();

    double FORWARD_TICKS_TO_INCHES;
    double STRAFE_TICKS_TO_INCHES;
    double TURN_TICKS_TO_RADIANS;

    private:

    pathing::geometry::Pose startPose;
    pathing::geometry::Pose displacementPose;
    pathing::geometry::Pose currentVelocity;
    // pathing::math::Matrix prevRotationMatrix;
    // NanoTimer timer; ??
    long deltaTimeNano;
    Encoder leftEncoder;
    Encoder rightEncoder;
    Encoder strafeEncoder;
    pathing::geometry::Pose leftEncoderPose;
    pathing::geometry::Pose rightEncoderPose;
    pathing::geometry::Pose strafeEncoderPose;
    double totalHeading;

};