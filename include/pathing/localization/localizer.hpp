#pragma once

#include "pathing/geometry/pose.hpp"

namespace pathing::localization {

/*
 * Base class for any pose estimation system.
 *
 * Examples of implementations:
 *  - Mecanum wheel odometry
 *  - Dead wheel odometry
 *  - IMU fusion
 *  - Simulation pose provider
 */
class Localizer {

public:

    virtual ~Localizer() = default;


    /*
     * Updates internal pose estimate.
     * Called once per control loop.
     */
    virtual void update(double dt) = 0;


    /*
     * Returns current estimated robot pose.
     */
    virtual const pathing::geometry::Pose& getPose() const = 0;


    /*
     * Allows resetting pose estimate.
     */
    virtual void setPose(
        const pathing::geometry::Pose& pose
    ) = 0;

};

}