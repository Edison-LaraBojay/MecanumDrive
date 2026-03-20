#include "pathing/control/vector_calculator.hpp"
#include <cmath>

namespace pathing::control {

using geometry::Vector2;

double VectorCalculator::dot(const Vector2& a, const Vector2& b) {
    return a.x * b.x + a.y * b.y;
}

double VectorCalculator::magnitude(const Vector2& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

Vector2 VectorCalculator::normalize(const Vector2& v) {
    double mag = magnitude(v);
    if (mag == 0) return Vector2(0, 0);
    return Vector2(v.x / mag, v.y / mag);
}

Vector2 VectorCalculator::project(const Vector2& a, const Vector2& b) {
    double denom = dot(b, b);
    if (denom == 0) return Vector2(0, 0);
    double scale = dot(a, b) / denom;
    return Vector2(b.x * scale, b.y * scale);
}

double VectorCalculator::distance(const Vector2& a, const Vector2& b) {
    return magnitude(Vector2(a.x - b.x, a.y - b.y));
}

}