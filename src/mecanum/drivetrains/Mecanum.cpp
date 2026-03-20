#include "Mecanum/drivetrains/mecanum.hpp"

#include <algorithm>
#include <cmath>

namespace drivetrains::mecanum {

using pathing::geometry::Vector2;
using pathing::geometry::Pose;

Mecanum::Mecanum(const MecanumConstants& constants)
    : constants(constants) {}

std::array<double, 4> Mecanum::calculateDrive(
    const Vector2& corrective,
    const Vector2& heading,
    const Vector2& pathing,
    double /*robotHeading*/
) {
    Vector2 c = corrective;
    Vector2 h = heading;
    Vector2 p = pathing;

    // Clamp magnitude
    auto clampMag = [&](Vector2& v) {
        double mag = v.magnitude();
        if (mag > constants.maxPower) {
            v = v.normalized() * constants.maxPower;
        }
    };

    clampMag(c);
    clampMag(h);
    clampMag(p);

    Vector2 left, right;

    if (c.magnitude() >= constants.maxPower) {
        left = c;
        right = c;
    } else {
        Vector2 leftSide = c - h;
        Vector2 rightSide = c + h;

        if (leftSide.magnitude() > constants.maxPower ||
            rightSide.magnitude() > constants.maxPower) {

            double scale = constants.maxPower /
                std::max(leftSide.magnitude(), rightSide.magnitude());

            left = c - h * scale;
            right = c + h * scale;
        } else {
            Vector2 leftWithPath = leftSide + p;
            Vector2 rightWithPath = rightSide + p;

            if (leftWithPath.magnitude() > constants.maxPower ||
                rightWithPath.magnitude() > constants.maxPower) {

                double scale = constants.maxPower /
                    std::max(leftWithPath.magnitude(), rightWithPath.magnitude());

                left = leftSide + p * scale;
                right = rightSide + p * scale;
            } else {
                left = leftWithPath;
                right = rightWithPath;
            }
        }
    }

    // Scale
    left = left * 2.0;
    right = right * 2.0;

    std::array<double, 4> powers{};

    // Mecanum mapping
    powers[0] = left.y + left.x;   // front left
    powers[1] = left.y - left.x;   // front right
    powers[2] = right.y - right.x; // back left
    powers[3] = right.y + right.x; // back right

    normalize(powers);
    return powers;
}

void Mecanum::normalize(std::array<double, 4>& powers) {
    double maxVal = std::max({
        std::abs(powers[0]),
        std::abs(powers[1]),
        std::abs(powers[2]),
        std::abs(powers[3]),
        1.0
    });

    for (auto& p : powers) {
        p /= maxVal;
    }
}

void Mecanum::setDrivePower(double x, double y, double heading) {


    Vector2 pathing(x, y);
    Vector2 corrective(0, 0);

    Vector2 headingVec(heading, 0);

    auto powers = calculateDrive(corrective, headingVec, pathing, 0.0);


}

Pose Mecanum::getPose() const {
    return pose;
}

void Mecanum::update() {

}

}