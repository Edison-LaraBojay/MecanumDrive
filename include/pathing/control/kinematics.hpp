#pragma once

#include "pathing/geometry/vector2.hpp"

namespace pathing::control {

class Kinematics {
public:

    struct ChassisVelocity {
        double vx;     // forward velocity
        double vy;     // lateral velocity (left positive)
        double omega;  // angular velocity
    };

    struct WheelVelocities {
        double front_left;
        double front_right;
        double back_left;
        double back_right;
    };

    // Convert chassis velocity to wheel velocities
    static WheelVelocities inverse(
        const ChassisVelocity& chassis,
        double track_width,
        double wheel_base
    );

    // Convert wheel velocities to chassis velocity
    static ChassisVelocity forward(
        const WheelVelocities& wheels,
        double track_width,
        double wheel_base
    );

};

}