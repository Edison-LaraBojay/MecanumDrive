#pragma once

namespace pathing::control {

class PIDFController {
private:
    double kP, kI, kD, kF;

    double integral;
    double prevError;
    bool firstUpdate;

public:
    PIDFController(double p, double i, double d, double f = 0.0);

    double update(double target, double current, double dt);

    void reset();
};

}