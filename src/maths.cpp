#include "maths.h"

#include <algorithm>
#include <cmath>
#include <iostream>

std::optional<Vector3D> Intersect(const Segment3D &x, const Segment3D &y) {
    Vector3D u = x.end - x.start;
    Vector3D v = y.end - y.start;
    Vector3D w = x.start - y.start;

    auto n = Cross(u, v);
    auto n2 = Dot(n, n);

    if (n2 < EPS * EPS) {
        std::cout << "Отрезки параллельны" << std::endl;
        return std::nullopt;
    }

    if (std::abs(ScalarTripleProduct(u, v, w)) > EPS) {
        return std::nullopt;
    }

    auto a = Dot(u, u);
    auto b = Dot(u, v);
    auto c = Dot(v, v);
    auto d = Dot(u, w);
    auto e = Dot(v, w);

    auto D = a * c - b * b;
    if (D < EPS)
        return std::nullopt;

    auto s = (b * e - c * d) / D;
    auto t = (a * e - b * d) / D;

    if (s < -EPS || s > 1.0 + EPS || t < -EPS || t > 1.0 + EPS) {
        return std::nullopt;
    }

    s = std::clamp(s, 0.0, 1.0);
    Vector3D point = x.start + u * s;

    return point;
}
