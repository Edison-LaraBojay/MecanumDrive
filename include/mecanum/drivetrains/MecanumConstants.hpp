#pragma once

namespace drivetrains::mecanum {

struct MecanumConstants {
    // Feedforward / scaling 
    double xVelocity = 81.34056;
    double yVelocity = 65.43028;

    // Power limiting
    double maxPower = 1.0;

    // Friction compensation PIDF
    double staticFrictionCoefficient = 0.1;

    // Motor inversion
    bool invertFrontLeft  = false;
    bool invertFrontRight = false;
    bool invertBackLeft   = false;
    bool invertBackRight  = false;
};

}