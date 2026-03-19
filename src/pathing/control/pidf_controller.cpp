#include "pathing/control/pidf_controller.hpp"

namespace pathing::control {

PIDFController::PIDFController(double p, double i, double d, double f)
    : kP(p), kI(i), kD(d), kF(f),
      integral(0.0), prevError(0.0), firstUpdate(true) {}

double PIDFController::update(double target, double current, double dt) {
    double error = target - current;

    if (firstUpdate) {
        prevError = error;
        firstUpdate = false;
    }

    integral += error * dt;
    double derivative = (error - prevError) / dt;

    prevError = error;

    return kP * error + kI * integral + kD * derivative + kF * target;
}

void PIDFController::reset() {
    integral = 0.0;
    prevError = 0.0;
    firstUpdate = true;
}

}