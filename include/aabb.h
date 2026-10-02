#pragma once

#include <algorithm>
#include <cmath>

#include "vector3d.h"

struct AABB {
    Vector3D min;
    Vector3D max;

    AABB(const Vector3D &p1, const Vector3D &p2, double eps = 1e-9);

    bool Intersects(const AABB &other) const;
};