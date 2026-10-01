#include "maths.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

inline double SquaredDistance(const Vector3D &a, const Vector3D &b) {
    Vector3D r = a - b;
    return Dot(r, r);
}

inline bool IsDegenerate(const Segment3D &s, const double tol) {
    return SquaredDistance(s.start, s.end) <= tol * tol;
}

inline std::pair<Vector3D, double> ClosestPointOnSegment(const Segment3D &seg, const Vector3D &p) {
    Vector3D v = seg.end - seg.start;
    double v_len2 = Dot(v, v);

    double t = std::clamp(Dot(p - seg.start, v) / v_len2, 0.0, 1.0);
    Vector3D projection = seg.start + v * t;
    return {projection, SquaredDistance(p, projection)};
}

std::optional<Vector3D> Intersect(const Segment3D &x, const Segment3D &y, const double tol) {
    const bool x_degen = IsDegenerate(x, tol);
    const bool y_degen = IsDegenerate(y, tol);

    if (x_degen && y_degen) {
        if (SquaredDistance(x.start, y.start) <= tol * tol) {
            return (x.start + y.start) * 0.5;
        }
        return std::nullopt;
    }

    if (x_degen) {
        auto [proj, d] = ClosestPointOnSegment(y, x.start);
        if (d <= tol * tol) {
            return (x.start + proj) * 0.5;
        }
        return std::nullopt;
    }

    if (y_degen) {
        auto [proj, d] = ClosestPointOnSegment(x, y.start);
        if (d <= tol * tol) {
            return (y.start + proj) * 0.5;
        }
        return std::nullopt;
    }

    Vector3D u = x.end - x.start;
    Vector3D v = y.end - y.start;
    Vector3D w = x.start - y.start;

    auto n = Cross(u, v);
    auto n2 = Dot(n, n);

    if (n2 < tol * tol) {
        throw std::runtime_error("отрезки параллельны");
    }

    if (std::abs(ScalarTripleProduct(u, v, w)) > tol) {
        return std::nullopt;
    }

    auto a = Dot(u, u);
    auto b = Dot(u, v);
    auto c = Dot(v, v);
    auto d = Dot(u, w);
    auto e = Dot(v, w);

    auto D = a * c - b * b;
    if (D < tol)
        return std::nullopt;

    auto s = (b * e - c * d) / D;
    auto t = (a * e - b * d) / D;

    if (s < -tol || s > 1.0 + tol || t < -tol || t > 1.0 + tol) {
        return std::nullopt;
    }

    s = std::clamp(s, 0.0, 1.0);
    Vector3D point = x.start + u * s;

    return point;
}
