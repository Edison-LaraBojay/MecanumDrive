#include "Pathing/control/controller.hpp"
#include <cmath>

namespace pathing::control {

using geometry::Pose;
using geometry::Vector2;

Controller::Controller(
    ErrorCalculator& errorCalculator,
    PIDFController& translationController,
    PIDFController& headingController
)
    : errorCalculator(errorCalculator),
      translationController(translationController),
      headingController(headingController) {}

ControlOutput Controller::update(
    const Pose& currentPose,
    const Pose& targetPose
) {
    ControlOutput output;

    // Find errors
    double errorX = ErrorCalculator::xError(targetPose, currentPose);
    double errorY = ErrorCalculator::yError(targetPose, currentPose);
    double errorHeading = ErrorCalculator::headingError(
        targetPose.getHeading(),
        currentPose.getHeading()
    );

    // Translation magnitude
    double translationError = std::sqrt(errorX * errorX + errorY * errorY);
    
    // dt (temporary fixed timestep)
    double dt = 0.02; // 20ms loop (adjust later if we gotta)

    // PID outputs
    double translationOutput =
        translationController.update(translationError, 0.0, dt);

    double headingOutput =
        headingController.update(errorHeading, 0.0, dt);

    // Convert to direction vector
    Vector2 direction(errorX, errorY);

    if (direction.magnitude() > 0) {
        direction = direction.normalized();
    }

    output.correctivePower = direction * translationOutput;

    // Heading as vector
    output.headingPower = Vector2(headingOutput, 0);

    return output;
}

}