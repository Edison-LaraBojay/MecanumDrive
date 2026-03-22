#include "pathing/control/heading_controller.hpp"
#include "pathing/control/follower_constants.hpp"

#include "pathing/math/math_functions.hpp"

#include <cmath>

namespace pathing::control {


HeadingController::HeadingController()
    : headingPID(
        FollowerConstants::HEADING_kP,
        FollowerConstants::HEADING_kI,
        FollowerConstants::HEADING_kD
      ),
      tolerance(
        FollowerConstants::HEADING_TOLERANCE
      )
{
}


HeadingController::HeadingController(
    double kP,
    double kI,
    double kD,
    double tolerance
)
    : headingPID(kP, kI, kD),
      tolerance(tolerance)
{
}


double HeadingController::update(
    double targetHeading,
    double currentHeading,
    double dt
)
{
    double error =
        pathing::math::normalizeAngle(
            targetHeading - currentHeading
        );


    double omega =  
        headingPID.update(
            error,
            0.0,
            dt
        );


    /*
     * clamp angular velocity
     */
    if(std::abs(omega) >
        FollowerConstants::MAX_ANGULAR_VELOCITY)
    {
        omega =
            std::copysign(
                FollowerConstants::MAX_ANGULAR_VELOCITY,
                omega
            );
    }

    return omega;
}


bool HeadingController::atTarget() const
{
    return std::abs(
        headingPID.getLastError()
    ) < tolerance;
}


void HeadingController::reset()
{
    headingPID.reset();
}

}