#include "pathing/control/kinematics.hpp"

namespace pathing::control {

Kinematics::WheelVelocities Kinematics::inverse(
    const ChassisVelocity& chassis,
    double track_width,
    double wheel_base
)
{
    const double k = (track_width + wheel_base) / 2.0;

    WheelVelocities wheels;

    wheels.front_left  = chassis.vx - chassis.vy - k * chassis.omega;
    wheels.front_right = chassis.vx + chassis.vy + k * chassis.omega;
    wheels.back_left   = chassis.vx + chassis.vy - k * chassis.omega;
    wheels.back_right  = chassis.vx - chassis.vy + k * chassis.omega;

    return wheels;
}


Kinematics::ChassisVelocity Kinematics::forward(
    const WheelVelocities& wheels,
    double track_width,
    double wheel_base
)
{
    const double k = (track_width + wheel_base) / 2.0;

    ChassisVelocity chassis;

    chassis.vx =
        (wheels.front_left +
         wheels.front_right +
         wheels.back_left +
         wheels.back_right) / 4.0;

    chassis.vy =
        (-wheels.front_left +
          wheels.front_right +
          wheels.back_left -
          wheels.back_right) / 4.0;

    chassis.omega =
        (-wheels.front_left +
          wheels.front_right -
          wheels.back_left +
          wheels.back_right) / (4.0 * k);

    return chassis;
}

}