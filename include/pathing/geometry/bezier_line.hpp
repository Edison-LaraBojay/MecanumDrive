#pragma once

#include <vector>
#include "pathing/geometry/vector2.hpp"

namespace pathing::geometry {
    class BezierLine {
        public:
        BezierLine();
        BezierLine(const Vector2& start, const Vector2& end);

        Vector2 getPoint(double t) const;
        Vector2 getDerivative(double t) const;

        private:
        Vector2 start;
        Vector2 end;
    };
}