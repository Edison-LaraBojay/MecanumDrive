#include "pathing/geometry/vector2.hpp"
#include <cmath>
namespace pathing::geometry {

    Vector2::Vector2()
    {
        x = 0;
        y = 0;

    }
Vector2::Vector2(double x, double y)
{
    this->x = x;
    this->y = y;
}

double Vector2::magnitude() const{
    return std::sqrt(x * x + y * y);
}

double Vector2::magnitudeSquared() const
{
    return x * x + y * y;
}

Vector2 Vector2::normalized() const
{
    double mag = magnitude();

    if (mag == 0)
    return Vector2(0,0);

    return Vector2(x / mag, y / mag);
}

double Vector2::dot(const Vector2& other) const
{
    return x * other.x + y * other.y;
}

double Vector2::cross(const Vector2& other) const
{
    return x * other.y - y * other.x;
}

double Vector2::distanceTo(const Vector2& other) const
{
    double dx = x - other.x;
    double dy = y - other.y;

    return std::sqrt(dx*dx + dy*dy);
}

Vector2 Vector2::operator+(const Vector2& other) const
{
    return Vector2(x + other.x, y + other.y);
}

Vector2 Vector2::operator-(const Vector2 & other) const
{
    return Vector2(x - other.x, y - other.y);
}

Vector2 Vector2::operator*(double scalar) const
{
    return Vector2(x * scalar, y * scalar);
}

Vector2 Vector2::operator/(double scalar) const
{
    return Vector2(x / scalar, y / scalar);
}
}
