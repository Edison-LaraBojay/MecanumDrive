#pragma once

#include "pathing/geometry/vector2.hpp" 

namespace pathing::control {

class VectorCalculator {
public:
    static double dot(const geometry::Vector2& a, const geometry::Vector2& b);

    static double magnitude(const geometry::Vector2& v);

    static geometry::Vector2 normalize(const geometry::Vector2& v);

    static geometry::Vector2 project(const geometry::Vector2& a, const geometry::Vector2& b);

    static double distance(const geometry::Vector2& a, const geometry::Vector2& b);
};

}