#include "math/Vector2D.h"

// Constructor implementation
Vector2D::Vector2D(double x_, double y_) : x(x_), y(y_) {}

// Returns vector length using Pythagoras
double Vector2D::magnitude() const {
    return std::sqrt(x*x + y*y);
}

// Normalizes the vector (scales it to length 1)
void Vector2D::normalize() {
    double mag = magnitude();
    if (mag > 0) { x /= mag; y /= mag; }
}

// Vector addition
Vector2D Vector2D::operator+(const Vector2D& other) const {
    return Vector2D(x + other.x, y + other.y);
}

// Scalar multiplication
Vector2D Vector2D::operator*(double scalar) const {
    return Vector2D(x * scalar, y * scalar);
}

Vector2D& Vector2D::operator+=(const Vector2D& other) {
        x += other.x;
        y += other.y;
        return *this;
}

Vector2D Vector2D::rotate(double angle) const {
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);
    return Vector2D(x * cosA - y * sinA, x * sinA + y * cosA);
}

