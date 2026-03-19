#include "pathing/control/filtered_pidf_controller.hpp"

namespace pathing::control {

FilteredPIDFController::FilteredPIDFController(
    double p, double i, double d, double f, double alpha)
    : kP(p), kI(i), kD(d), kF(f),
      integral(0.0), prevError(0.0),
      firstUpdate(true),
      derivativeFilter(alpha) {}

double FilteredPIDFController::update(double target, double current, double dt) {
    double error = target - current;

    if (firstUpdate) {
        prevError = error;
        firstUpdate = false;
    }

    integral += error * dt;

    double rawDerivative = (error - prevError) / dt;
    double filteredDerivative = derivativeFilter.update(rawDerivative);

    prevError = error;

    return kP * error + kI * integral + kD * filteredDerivative + kF * target;
}

void FilteredPIDFController::reset() {
    integral = 0.0;
    prevError = 0.0;
    firstUpdate = true;
    derivativeFilter.reset();
}

}