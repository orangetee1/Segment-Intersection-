#pragma once

#include <optional>
#include <variant>

#include "segment3d.h"

using IntersectionResult = std::variant<std::monostate, Vector3D, Segment3D>;

inline double Dot(const Vector3D &a, const Vector3D &b) {
    return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
}

inline Vector3D Cross(const Vector3D &a, const Vector3D &b) {
    return Vector3D(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
}

inline double ScalarTripleProduct(const Vector3D &a, const Vector3D &b, const Vector3D &c) {
    return a.X * (b.Y * c.Z - b.Z * c.Y) - a.Y * (b.X * c.Z - b.Z * c.X) +
           a.Z * (b.X * c.Y - b.Y * c.X);
}

IntersectionResult Intersect(const Segment3D &a, const Segment3D &b, const double tol = 1e-7);
