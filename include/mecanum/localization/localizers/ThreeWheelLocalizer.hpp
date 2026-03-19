#include <pose.hpp>
#include <Encoder.hpp>
#include <ThreeWheelConstants.hpp>
#include <NanoTimer.hpp>

class ThreeWheelLocalizer { 

    public:
    ThreeWheelLocalizer(int leftPort, int rightPort, int strafePort, const ThreeWheelConstants& constants, pathing::geometry::Pose setStartPose);
    pathing::geometry::Pose getPose();
    pathing::geometry::Pose getVelocity();
    pathing::geometry::Vector2 getVelocityVector();
    void setStartPose(pathing::geometry::Pose setStart);
    void setPrevRotationMatrix(double heading);
    void setPose(pathing::geometry::Pose setPose);
    void update();
    void updateEncoders();
    void resetEncoders();
    // Matrix getRobotDeltas
    double getTotalHeading();
    double getForwardMultiplier();
    double getLateralMultiplier();
    double getTurningMultiplier();
    bool isNAN();

    double FORWARD_TICKS_TO_INCHES;
    double STRAFE_TICKS_TO_INCHES;
    double TURN_TICKS_TO_RADIANS;

    private:

    pathing::geometry::Pose startPose;
    pathing::geometry::Pose displacementPose;
    pathing::geometry::Pose currentVelocity;
    // pathing::math::Matrix prevRotationMatrix;
    NanoTimer timer;
    long deltaTimeNano;
    Encoder leftEncoder;
    Encoder rightEncoder;
    Encoder strafeEncoder;
    pathing::geometry::Pose leftEncoderPose;
    pathing::geometry::Pose rightEncoderPose;
    pathing::geometry::Pose strafeEncoderPose;
    double totalHeading;



};