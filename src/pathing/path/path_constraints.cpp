#include "pathing/path/path_constraints.hpp"

namespace pathing::path {

PathConstraints::PathConstraints()
{
    maxVelocity = 1.0;
    maxAcceleration = 1.0;
    maxAngularVelocity = 1.0;
}

PathConstraints::PathConstraints(
    double maxVel,
    double maxAccel,
    double maxAngVel)
{
    maxVelocity = maxVel;
    maxAcceleration = maxAccel;
    maxAngularVelocity = maxAngVel;
}

}