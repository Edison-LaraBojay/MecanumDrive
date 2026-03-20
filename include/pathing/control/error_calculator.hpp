#pragma once

#include "pathing/geometry/pose.hpp"

namespace pathing::control {

class ErrorCalculator {
public:
    static double headingError(double target, double current);

    static double xError(const geometry::Pose& target, const geometry::Pose& current);

    static double yError(const geometry::Pose& target, const geometry::Pose& current);
};

}