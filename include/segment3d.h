#pragma once

#include "vector3d.h"

struct Segment3D {
    Segment3D(const Vector3D &a, const Vector3D &b) : Start(a), End(b) {}

    Vector3D Start;
    Vector3D End;
};