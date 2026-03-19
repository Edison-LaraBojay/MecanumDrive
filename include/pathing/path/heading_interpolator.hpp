#pragma once

#include <cmath>

namespace pathing {

class HeadingInterpolator {
public:
    // Linearly interpolate between two headings (radians)
    static double interpolate(double start, double end, double t);

    // Normalize angle to [-pi, pi]
    static double normalize(double angle);

    // Smallest angular difference (end - start, wrapped)
    static double shortestAngularDistance(double start, double end);
};

}