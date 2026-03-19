#pragma once

#include "pathing/control/low_pass_filter.hpp"

namespace pathing::control {

class FilteredPIDFController {
private:
    double kP, kI, kD, kF;

    double integral;
    double prevError;
    bool firstUpdate;

    LowPassFilter derivativeFilter;

public:
    FilteredPIDFController(double p, double i, double d, double f, double alpha);

    double update(double target, double current, double dt);

    void reset();
};

}