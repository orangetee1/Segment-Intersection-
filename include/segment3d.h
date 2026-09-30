#pragma once

#include "vector3d.h"

struct Segment3D {
    Segment3D(const Vector3D &a, const Vector3D &b) : start(a), end(b) {}

    Vector3D start;
    Vector3D end;
};