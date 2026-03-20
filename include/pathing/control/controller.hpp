#pragma once

#include "Pathing/geometry/pose.hpp"
#include "Pathing/geometry/vector2.hpp"
#include "Pathing/control/error_calculator.hpp"
#include "Pathing/control/pidf_controller.hpp"
#include "Pathing/control/vector_calculator.hpp"

namespace pathing::control {

struct ControlOutput {
    geometry::Vector2 correctivePower;
    geometry::Vector2 headingPower;
};

class Controller {
public:
    Controller(
        ErrorCalculator& errorCalculator,
        PIDFController& translationController,
        PIDFController& headingController
    );

    ControlOutput update(
        const geometry::Pose& currentPose,
        const geometry::Pose& targetPose
    );

private:
    ErrorCalculator& errorCalculator;

    PIDFController& translationController;
    PIDFController& headingController;

    VectorCalculator vectorCalculator;
};

}