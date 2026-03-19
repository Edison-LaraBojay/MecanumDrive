#include "pathing/path/heading_interpolator.hpp"

namespace pathing::path {

double HeadingInterpolator::normalize(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double HeadingInterpolator::shortestAngularDistance(double start, double end) {
    double diff = normalize(end - start);
    return diff;
}

double HeadingInterpolator::interpolate(double start, double end, double t) {
    double diff = shortestAngularDistance(start, end);
    return normalize(start + diff * t);
}

}