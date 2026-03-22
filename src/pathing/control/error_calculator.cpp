#include "pathing/control/error_calculator.hpp"
#include "pathing/math/math_functions.hpp"

#include <cmath>

namespace pathing::control {

double ErrorCalculator::headingError(double target, double current)
{
    return pathing::math::normalizeAngle(target - current);
}


double ErrorCalculator::xError(
    const geometry::Pose& target,
    const geometry::Pose& current
)
{
    const double dx = current.x - target.x;
    const double dy = current.y - target.y;

    const double tangent_x = std::cos(target.heading);
    const double tangent_y = std::sin(target.heading);

    return dx * tangent_x + dy * tangent_y;
}


double ErrorCalculator::yError(
    const geometry::Pose& target,
    const geometry::Pose& current
)
{
    const double dx = current.x - target.x;
    const double dy = current.y - target.y;

    const double normal_x = -std::sin(target.heading);
    const double normal_y =  std::cos(target.heading);

    return dx * normal_x + dy * normal_y;
}

}