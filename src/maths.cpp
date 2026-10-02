#include "maths.h"
#include <algorithm>
#include <cmath>
#include <utility>

#include "aabb.h"

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
    if (v_len2 == 0.0) {
        return {seg.start, SquaredDistance(p, seg.start)};
    }
    double t = std::clamp(Dot(p - seg.start, v) / v_len2, 0.0, 1.0);
    Vector3D projection = seg.start + v * t;
    return {projection, SquaredDistance(p, projection)};
}

IntersectionResult IntersectCollinear(const Segment3D &x, const Segment3D &y, const double tol) {
    Vector3D v = y.end - y.start;
    double v_len2 = Dot(v, v);

    Vector3D w = x.start - y.start;
    Vector3D cross_wv = Cross(w, v);
    if (Dot(cross_wv, cross_wv) > tol * tol * v_len2) {
        return std::monostate{};
    }

    double tA = Dot(x.start - y.start, v) / v_len2;
    double tB = Dot(x.end - y.start, v) / v_len2;

    double t_min = std::min(tA, tB);
    double t_max = std::max(tA, tB);

    double t_start = std::max(0.0, t_min);
    double t_end = std::min(1.0, t_max);

    if (t_start > t_end + tol) {
        return std::monostate{};
    }

    if (std::abs(t_end - t_start) <= tol) {
        return y.start + v * t_start;
    }

    return Segment3D(y.start + v * t_start, y.start + v * t_end);
}

IntersectionResult Intersect(const Segment3D &x, const Segment3D &y, const double tol) {
    const AABB x_box{x.start, x.end, tol};
    const AABB y_box{y.start, y.end, tol};

    if (!x_box.intersects(y_box)) {
        return std::monostate{};
    }

    const bool x_degen = IsDegenerate(x, tol);
    const bool y_degen = IsDegenerate(y, tol);

    if (x_degen && y_degen) {
        if (SquaredDistance(x.start, y.start) <= tol * tol) {
            return (x.start + y.start) * 0.5;
        }
        return std::monostate{};
    }

    if (x_degen) {
        auto [proj, d] = ClosestPointOnSegment(y, x.start);
        if (d <= tol * tol)
            return (x.start + proj) * 0.5;
        return std::monostate{};
    }

    if (y_degen) {
        auto [proj, d] = ClosestPointOnSegment(x, y.start);
        if (d <= tol * tol)
            return (y.start + proj) * 0.5;
        return std::monostate{};
    }

    Vector3D u = x.end - x.start;
    Vector3D v = y.end - y.start;
    Vector3D w = x.start - y.start;

    auto n = Cross(u, v);
    auto n2 = Dot(n, n);
    auto u_len2 = Dot(u, u);
    auto v_len2 = Dot(v, v);

    if (n2 <= tol * tol * u_len2 * v_len2) {
        return IntersectCollinear(x, y, tol);
    }

    double triple = Dot(n, w);
    if ((triple * triple) > tol * tol * n2) {
        return std::monostate{};
    }

    auto a = u_len2;
    auto b = Dot(u, v);
    auto c = v_len2;
    auto d = Dot(u, w);
    auto e = Dot(v, w);
    auto D = n2;

    auto s = (b * e - c * d) / D;
    auto t = (a * e - b * d) / D;

    double s_tol = tol / std::sqrt(a);
    double t_tol = tol / std::sqrt(c);

    if (s < -s_tol || s > 1.0 + s_tol || t < -t_tol || t > 1.0 + t_tol) {
        return std::monostate{};
    }

    s = std::clamp(s, 0.0, 1.0);
    t = std::clamp(t, 0.0, 1.0);

    Vector3D pX = x.start + u * s;
    Vector3D pY = y.start + v * t;

    if (SquaredDistance(pX, pY) <= tol * tol) {
        return (pX + pY) * 0.5;
    }

    return std::monostate{};
}