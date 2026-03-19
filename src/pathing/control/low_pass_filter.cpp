#include "pathing/control/low_pass_filter.hpp"

namespace pathing::control {

LowPassFilter::LowPassFilter(double alpha)
    : alpha(alpha), state(0.0), initialized(false) {}

double LowPassFilter::update(double input) {
    if (!initialized) {
        state = input;
        initialized = true;
        return state;
    }

    state = alpha * input + (1.0 - alpha) * state;
    return state;
}

void LowPassFilter::reset(double value) {
    state = value;
    initialized = false;
}

}