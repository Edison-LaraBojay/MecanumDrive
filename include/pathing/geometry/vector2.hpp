#pragma once
#include <cmath>
namespace pathing::geometry {

    class Vector2
    {
        public:

        double x;
        double y;

        Vector2();
        Vector2(double x, double y);

        double magnitude() const;
        double magnitudeSquared() const;

        Vector2 normalized() const;

        double dot(const Vector2& other) const;
        double cross(const Vector2& other) const;

        double distanceTo(const Vector2& other) const;

        Vector2 operator+(const Vector2& other) const;
        Vector2 operator-(const Vector2& other) const;

        Vector2 operator*(double scalar) const;
        Vector2 operator/(double scalar) const;
    };
    
}