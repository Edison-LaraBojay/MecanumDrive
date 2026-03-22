#pragma once

#include "pathing/control/pidf_controller.hpp"

namespace pathing::control {

class HeadingController {

private:

    PIDFController headingPID;

    double tolerance;

public:

    HeadingController();

    HeadingController(
        double kP,
        double kI,
        double kD,
        double tolerance
    );


    /*
     * compute angular velocity command
     */
    double update(
        double targetHeading,
        double currentHeading,
        double dt
    );


    bool atTarget() const;


    void reset();

};

}