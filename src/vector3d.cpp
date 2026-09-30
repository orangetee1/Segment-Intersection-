#include "vector3d.h"

Vector3D Vector3D::operator+(const Vector3D &a) const {
    return Vector3D(X + a.X, Y + a.Y, Z + a.Z);
}

Vector3D Vector3D::operator-(const Vector3D &a) const {
    return Vector3D(X - a.X, Y - a.Y, Z - a.Z);
}

Vector3D Vector3D::operator*(double a) const { return Vector3D(X * a, Y * a, Z * a); }
