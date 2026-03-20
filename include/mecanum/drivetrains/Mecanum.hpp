#pragma once

#include <array>

#include "Mecanum/drivetrains/mecanumconstants.hpp"
#include "Pathing/drivetrain/drivetrain.hpp"
#include "Pathing/geometry/vector2.hpp"
#include "Pathing/geometry/pose.hpp"

namespace drivetrains::mecanum {

class Mecanum : public pathing::Drivetrain {
public:
    explicit Mecanum(const MecanumConstants& constants);

    // Core drive calculation (Pedro-style simplified)
    std::array<double, 4> calculateDrive(
        const pathing::geometry::Vector2& correctivePower,
        const pathing::geometry::Vector2& headingPower,
        const pathing::geometry::Vector2& pathingPower,
        double robotHeading
    );

    // Drivetrain interface
    void setDrivePower(double x, double y, double heading) override;
    pathing::geometry::Pose getPose() const override;
    void update() override;

private:
    MecanumConstants constants;

    pathing::geometry::Pose pose;

    void normalize(std::array<double, 4>& powers);
};

}