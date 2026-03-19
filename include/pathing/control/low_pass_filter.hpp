#pragma once

namespace pathing::control {

class LowPassFilter {
private:
    double alpha;   // smoothing factor (0–1)
    double state;
    bool initialized;

public:
    LowPassFilter(double alpha);

    double update(double input);

    void reset(double value = 0.0);
};

}