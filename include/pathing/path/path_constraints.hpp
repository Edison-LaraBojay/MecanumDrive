#pragma once

namespace pathing::path {

class PathConstraints {

public:

    double maxVelocity;
    double maxAcceleration;
    double maxAngularVelocity;

    PathConstraints();
    PathConstraints(
        double maxVel,
        double maxAccel,
        double maxAngVel
    );

};

}