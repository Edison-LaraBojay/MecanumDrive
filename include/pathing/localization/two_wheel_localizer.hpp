#pragma once

#include "pathing/localization/localizer.hpp"
#include "pathing/geometry/pose.hpp"

namespace pathing::localization {

/*
 * Interface for encoder, probably will need to change
 */
class EncoderSource {
public:
    virtual ~EncoderSource() = default;

    virtual double getPosition() const = 0;
    virtual void reset() = 0;
};


/*
 * Same deal with the imu, probably will need to change
 */
class IMUSource {
public:
    virtual ~IMUSource() = default;

    virtual double getHeading() const = 0;
    virtual void reset() = 0;
};


/*
 * Two tracking wheel + IMU localizer
 */
class TwoWheelLocalizer : public Localizer {

private:

    EncoderSource* forwardEncoder;
    EncoderSource* strafeEncoder;
    IMUSource* imu;


    geometry::Pose startPose;
    geometry::Pose displacementPose;
    geometry::Pose velocityPose;


    double prevForwardTicks;
    double prevStrafeTicks;

    double previousHeading;

    double totalHeading;


    /*
     * robot geometry
     */
    double forwardTicksToDistance;
    double strafeTicksToDistance;

    double forwardPodY;
    double strafePodX;


public:

    TwoWheelLocalizer(
        EncoderSource* forwardEncoder,
        EncoderSource* strafeEncoder,
        IMUSource* imu,
        double forwardTicksToDistance,
        double strafeTicksToDistance,
        double forwardPodY,
        double strafePodX
    );


    void update(double dt) override;


    const geometry::Pose& getPose() const override;


    void setPose(
        const geometry::Pose& pose
    ) override;


    geometry::Pose getVelocity() const;


    double getTotalHeading() const;


    void reset();

};

}