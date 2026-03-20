#pragma once

#include "Pathing/geometry/pose.hpp"

namespace pathing {

class Drivetrain {
public:
    virtual ~Drivetrain() = default;

    virtual void setDrivePower(double x, double y, double heading) = 0;

    virtual geometry::Pose getPose() const = 0;

    virtual void update() = 0;
};

}