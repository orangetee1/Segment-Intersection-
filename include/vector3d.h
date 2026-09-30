#pragma once

struct Vector3D {
    Vector3D(double x, double y, double z) : X(x), Y(y), Z(z) {}

    double X;
    double Y;
    double Z;

    Vector3D operator+(const Vector3D &a) const;
    Vector3D operator-(const Vector3D &a) const;
    Vector3D operator*(double a) const;
};