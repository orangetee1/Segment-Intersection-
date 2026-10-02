#include "aabb.h"

AABB::AABB(const Vector3D &p1, const Vector3D &p2, double eps)
    : min(std::min(p1.X, p2.X) - eps, std::min(p1.Y, p2.Y) - eps, std::min(p1.Z, p2.Z) - eps),
      max(std::max(p1.X, p2.X) + eps, std::max(p1.Y, p2.Y) + eps, std::max(p1.Z, p2.Z) + eps) {}

bool AABB::Intersects(const AABB &other) const {
    if (max.X < other.min.X || min.X > other.max.X)
        return false;
    if (max.Y < other.min.Y || min.Y > other.max.Y)
        return false;
    if (max.Z < other.min.Z || min.Z > other.max.Z)
        return false;
    return true;
}
