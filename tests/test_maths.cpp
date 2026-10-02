#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <variant>

#include "aabb.h"
#include "maths.h"

namespace {

void ExpectVectorNear(const Vector3D &actual, const Vector3D &expected, double tol = 1e-6) {
    EXPECT_NEAR(actual.X, expected.X, tol);
    EXPECT_NEAR(actual.Y, expected.Y, tol);
    EXPECT_NEAR(actual.Z, expected.Z, tol);
}

void ExpectNoIntersection(const IntersectionResult &result) {
    EXPECT_TRUE(std::holds_alternative<std::monostate>(result));
}

void ExpectPointIntersection(const IntersectionResult &result, const Vector3D &expected,
                             double tol = 1e-6) {
    ASSERT_TRUE(std::holds_alternative<Vector3D>(result))
        << "Expected Vector3D result, but got another alternative";
    const auto &pt = std::get<Vector3D>(result);
    ExpectVectorNear(pt, expected, tol);
}

void ExpectSegmentIntersection(const IntersectionResult &result, const Segment3D &expected,
                               double tol = 1e-6) {
    ASSERT_TRUE(std::holds_alternative<Segment3D>(result))
        << "Expected Segment3D result, but got another alternative";
    const auto &seg = std::get<Segment3D>(result);

    bool forwardMatch = (std::abs(seg.start.X - expected.start.X) <= tol &&
                         std::abs(seg.start.Y - expected.start.Y) <= tol &&
                         std::abs(seg.start.Z - expected.start.Z) <= tol &&
                         std::abs(seg.end.X - expected.end.X) <= tol &&
                         std::abs(seg.end.Y - expected.end.Y) <= tol &&
                         std::abs(seg.end.Z - expected.end.Z) <= tol);

    bool reverseMatch = (std::abs(seg.start.X - expected.end.X) <= tol &&
                         std::abs(seg.start.Y - expected.end.Y) <= tol &&
                         std::abs(seg.start.Z - expected.end.Z) <= tol &&
                         std::abs(seg.end.X - expected.start.X) <= tol &&
                         std::abs(seg.end.Y - expected.start.Y) <= tol &&
                         std::abs(seg.end.Z - expected.start.Z) <= tol);

    EXPECT_TRUE(forwardMatch || reverseMatch)
        << "Expected segment: [(" << expected.start.X << ", " << expected.start.Y << ", "
        << expected.start.Z << ") - (" << expected.end.X << ", " << expected.end.Y << ", "
        << expected.end.Z << ")]\n"
        << "Actual segment: [(" << seg.start.X << ", " << seg.start.Y << ", " << seg.start.Z
        << ") - (" << seg.end.X << ", " << seg.end.Y << ", " << seg.end.Z << ")]";
}

} // namespace

TEST(VectorMathTest, DotProduct) {
    Vector3D a(1.0, 2.0, 3.0);
    Vector3D b(4.0, -5.0, 6.0);
    EXPECT_DOUBLE_EQ(Dot(a, b), 12.0);
}

TEST(VectorMathTest, CrossProductOrthogonal) {
    Vector3D x(1.0, 0.0, 0.0);
    Vector3D y(0.0, 1.0, 0.0);
    Vector3D z = Cross(x, y);

    EXPECT_DOUBLE_EQ(z.X, 0.0);
    EXPECT_DOUBLE_EQ(z.Y, 0.0);
    EXPECT_DOUBLE_EQ(z.Z, 1.0);
}

TEST(VectorMathTest, CrossProductCollinear) {
    Vector3D a(2.0, 3.0, 4.0);
    Vector3D b(4.0, 6.0, 8.0);
    Vector3D c = Cross(a, b);

    EXPECT_NEAR(c.X, 0.0, 1e-12);
    EXPECT_NEAR(c.Y, 0.0, 1e-12);
    EXPECT_NEAR(c.Z, 0.0, 1e-12);
}

TEST(AABBTest, OverlappingBoxes) {
    AABB b1({0.0, 0.0, 0.0}, {2.0, 2.0, 2.0});
    AABB b2({1.0, 1.0, 1.0}, {3.0, 3.0, 3.0});
    EXPECT_TRUE(b1.intersects(b2));
    EXPECT_TRUE(b2.intersects(b1));
}

TEST(AABBTest, DisjointBoxes) {
    AABB b1({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0});
    AABB b2({5.0, 5.0, 5.0}, {6.0, 6.0, 6.0});
    EXPECT_FALSE(b1.intersects(b2));
    EXPECT_FALSE(b2.intersects(b1));
}

TEST(AABBTest, TouchingBoxes) {
    AABB b1({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0});
    AABB b2({1.0, 1.0, 1.0}, {2.0, 2.0, 2.0});
    EXPECT_TRUE(b1.intersects(b2));
}

TEST(IntersectTest, OrthogonalCrossXY) {
    Segment3D s1({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    Segment3D s2({0.0, -1.0, 0.0}, {0.0, 1.0, 0.0});

    auto res = Intersect(s1, s2);
    ExpectPointIntersection(res, {0.0, 0.0, 0.0});
}

TEST(IntersectTest, Arbitrary3DPlane) {
    Segment3D s1({0.0, 0.0, 0.0}, {2.0, 2.0, 2.0});
    Segment3D s2({0.0, 2.0, 0.0}, {2.0, 0.0, 2.0});

    auto res = Intersect(s1, s2);
    ExpectPointIntersection(res, {1.0, 1.0, 1.0});
}

TEST(IntersectTest, CoplanarNonIntersecting) {
    Segment3D s1({1.0, 0.0, 0.0}, {3.0, 0.0, 0.0});
    Segment3D s2({0.0, 1.0, 0.0}, {0.0, 3.0, 0.0});

    auto res = Intersect(s1, s2);
    ExpectNoIntersection(res);
}

// касание вершинами

TEST(IntersectTest, TouchingL_StartStart) {
    Segment3D s1({1.0, 1.0, 1.0}, {3.0, 1.0, 1.0});
    Segment3D s2({1.0, 1.0, 1.0}, {1.0, 3.0, 1.0});

    auto res = Intersect(s1, s2);
    ExpectPointIntersection(res, {1.0, 1.0, 1.0});
}

TEST(IntersectTest, TouchingL_EndStart) {
    Segment3D s1({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    Segment3D s2({2.0, 0.0, 0.0}, {2.0, 2.0, 0.0});

    auto res = Intersect(s1, s2);
    ExpectPointIntersection(res, {2.0, 0.0, 0.0});
}

TEST(IntersectTest, TouchingL_EndEnd) {
    Segment3D s1({0.0, 0.0, 0.0}, {2.0, 2.0, 0.0});
    Segment3D s2({0.0, 4.0, 0.0}, {2.0, 2.0, 0.0});

    auto res = Intersect(s1, s2);
    ExpectPointIntersection(res, {2.0, 2.0, 0.0});
}

TEST(IntersectTest, TouchingT_Midpoint) {
    // Конец одного отрезка лежит на середине другого
    Segment3D s1({0.0, 0.0, 0.0}, {4.0, 0.0, 0.0});
    Segment3D s2({2.0, 0.0, 0.0}, {2.0, 3.0, 0.0});

    auto res = Intersect(s1, s2);
    ExpectPointIntersection(res, {2.0, 0.0, 0.0});
}

// коллинеарные отрезки

TEST(IntersectTest, CollinearDisjoint) {
    Segment3D s1({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    Segment3D s2({2.0, 0.0, 0.0}, {3.0, 0.0, 0.0});

    auto res = Intersect(s1, s2);
    ExpectNoIntersection(res);
}

TEST(IntersectTest, CollinearTouchingVertex) {
    Segment3D s1({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    Segment3D s2({2.0, 0.0, 0.0}, {4.0, 0.0, 0.0});

    auto res = Intersect(s1, s2);
    ExpectPointIntersection(res, {2.0, 0.0, 0.0});
}

TEST(IntersectTest, CollinearPartialOverlap) {
    Segment3D s1({0.0, 0.0, 0.0}, {3.0, 0.0, 0.0});
    Segment3D s2({2.0, 0.0, 0.0}, {5.0, 0.0, 0.0});

    auto res = Intersect(s1, s2);
    ExpectSegmentIntersection(res, {{2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}});
}

TEST(IntersectTest, CollinearOneInsideAnother) {
    Segment3D s1({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0});
    Segment3D s2({2.0, 0.0, 0.0}, {6.0, 0.0, 0.0});

    auto res = Intersect(s1, s2);
    ExpectSegmentIntersection(res, {{2.0, 0.0, 0.0}, {6.0, 0.0, 0.0}});
}

TEST(IntersectTest, CollinearIdentical) {
    Segment3D s1({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});
    Segment3D s2({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});

    auto res = Intersect(s1, s2);
    ExpectSegmentIntersection(res, {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
}

TEST(IntersectTest, CollinearOppositeDirections) {
    Segment3D s1({0.0, 0.0, 0.0}, {4.0, 0.0, 0.0});
    Segment3D s2({5.0, 0.0, 0.0}, {2.0, 0.0, 0.0});

    auto res = Intersect(s1, s2);
    ExpectSegmentIntersection(res, {{2.0, 0.0, 0.0}, {4.0, 0.0, 0.0}});
}

TEST(IntersectTest, Collinear3D_Diagonal) {
    Segment3D s1({1.0, 1.0, 1.0}, {6.0, 6.0, 6.0});
    Segment3D s2({1.0, 1.0, 1.0}, {3.0, 3.0, 3.0});

    auto res = Intersect(s1, s2);
    ExpectSegmentIntersection(res, {{1.0, 1.0, 1.0}, {3.0, 3.0, 3.0}});
}

TEST(IntersectTest, ParallelDisjoint) {
    Segment3D s1({0.0, 0.0, 0.0}, {5.0, 0.0, 0.0});
    Segment3D s2({0.0, 2.0, 0.0}, {5.0, 2.0, 0.0});

    auto res = Intersect(s1, s2);
    ExpectNoIntersection(res);
}

TEST(IntersectTest, ParallelCloseOutsideTolerance) {
    Segment3D s1({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    Segment3D s2({0.0, 1e-4, 0.0}, {1.0, 1e-4, 0.0});

    auto res = Intersect(s1, s2, 1e-6);
    ExpectNoIntersection(res);
}

// Скрещивающиеся отрезки (Skew lines)

TEST(IntersectTest, SkewWithinTolerance) {
    const double tol = 1e-6;
    // Отрезки скрещиваются на расстоянии 0.5 * tol
    Segment3D s1({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    Segment3D s2({0.0, -1.0, 0.5 * tol}, {0.0, 1.0, 0.5 * tol});

    auto res = Intersect(s1, s2, tol);
    ExpectPointIntersection(res, {0.0, 0.0, 0.25 * tol}, tol);
}

TEST(IntersectTest, SkewOutsideTolerance) {
    const double tol = 1e-6;
    // Отрезки скрещиваются на расстоянии 2.0 * tol
    Segment3D s1({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    Segment3D s2({0.0, -1.0, 2.0 * tol}, {0.0, 1.0, 2.0 * tol});

    auto res = Intersect(s1, s2, tol);
    ExpectNoIntersection(res);
}

TEST(IntersectTest, SkewFarApart) {
    Segment3D s1({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    Segment3D s2({1.0, 1.0, 5.0}, {1.0, 3.0, 5.0});

    auto res = Intersect(s1, s2);
    ExpectNoIntersection(res);
}

// Вырожденные случаи (точки)

TEST(IntersectTest, DegenerateBothPointsCoincide) {
    Segment3D s1({2.0, 3.0, 4.0}, {2.0, 3.0, 4.0});
    Segment3D s2({2.0, 3.0, 4.0}, {2.0, 3.0, 4.0});

    auto res = Intersect(s1, s2);
    ExpectPointIntersection(res, {2.0, 3.0, 4.0});
}

TEST(IntersectTest, DegenerateBothPointsDisjoint) {
    Segment3D s1({2.0, 3.0, 4.0}, {2.0, 3.0, 4.0});
    Segment3D s2({2.0, 3.0, 5.0}, {2.0, 3.0, 5.0});

    auto res = Intersect(s1, s2);
    ExpectNoIntersection(res);
}

TEST(IntersectTest, DegeneratePointOnSegmentMiddle) {
    Segment3D pt({1.0, 1.0, 1.0}, {1.0, 1.0, 1.0});
    Segment3D seg({0.0, 0.0, 0.0}, {2.0, 2.0, 2.0});

    auto res = Intersect(pt, seg);
    ExpectPointIntersection(res, {1.0, 1.0, 1.0});
}

TEST(IntersectTest, DegeneratePointOnSegmentEndpoint) {
    Segment3D pt({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    Segment3D seg({0.0, 0.0, 0.0}, {5.0, 0.0, 0.0});

    auto res = Intersect(seg, pt);
    ExpectPointIntersection(res, {0.0, 0.0, 0.0});
}

TEST(IntersectTest, DegeneratePointOffSegmentCollinear) {
    // Точка лежит на продолжении прямой, но вне отрезка
    Segment3D pt({6.0, 0.0, 0.0}, {6.0, 0.0, 0.0});
    Segment3D seg({0.0, 0.0, 0.0}, {5.0, 0.0, 0.0});

    auto res = Intersect(pt, seg);
    ExpectNoIntersection(res);
}

TEST(IntersectTest, DegeneratePointOffSegmentSpatial) {
    Segment3D pt({1.0, 5.0, 0.0}, {1.0, 5.0, 0.0});
    Segment3D seg({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});

    auto res = Intersect(pt, seg);
    ExpectNoIntersection(res);
}

// микро- и макро-координаты

TEST(IntersectTest, ScaleMacroCoordinates) {
    Segment3D s1({0.0, 0.0, 0.0}, {1e6, 0.0, 0.0});
    Segment3D s2({5e5, -1e6, 0.0}, {5e5, 1e6, 0.0});

    auto res = Intersect(s1, s2, 1e-4);
    ExpectPointIntersection(res, {5e5, 0.0, 0.0}, 1e-4);
}

TEST(IntersectTest, ScaleMicroCoordinates) {
    const double tol = 1e-9;
    Segment3D s1({-1e-5, 0.0, 0.0}, {1e-5, 0.0, 0.0});
    Segment3D s2({0.0, -1e-5, 0.0}, {0.0, 1e-5, 0.0});

    auto res = Intersect(s1, s2, tol);
    ExpectPointIntersection(res, {0.0, 0.0, 0.0}, tol);
}

// Стресс-тест

TEST(IntersectStressTest, RandomizedGuaranteedIntersections) {
    std::mt19937_64 rng(1337);
    std::uniform_real_distribution<double> coordDist(-50.0, 50.0);
    std::uniform_real_distribution<double> paramDist(0.1, 0.9);
    const double tol = 1e-6;

    for (int i = 0; i < 2000; ++i) {
        Vector3D p1(coordDist(rng), coordDist(rng), coordDist(rng));
        Vector3D dir1(coordDist(rng), coordDist(rng), coordDist(rng));
        Vector3D p2 = p1 + dir1;
        Segment3D s1(p1, p2);

        // Точка на первом отрезке
        double t1 = paramDist(rng);
        Vector3D sharedPt = p1 + dir1 * t1;

        // Второй отрезок, проходящий через sharedPt
        Vector3D dir2(coordDist(rng), coordDist(rng), coordDist(rng));
        double t2 = paramDist(rng);
        Vector3D q1 = sharedPt - dir2 * t2;
        Vector3D q2 = sharedPt + dir2 * (1.0 - t2);
        Segment3D s2(q1, q2);

        auto res = Intersect(s1, s2, tol);

        // пересечение гарантированно должно быть найдено
        ASSERT_FALSE(std::holds_alternative<std::monostate>(res)) << "Failed at iteration " << i;

        if (std::holds_alternative<Vector3D>(res)) {
            const auto &pt = std::get<Vector3D>(res);
            double dist2 = (pt.X - sharedPt.X) * (pt.X - sharedPt.X) +
                           (pt.Y - sharedPt.Y) * (pt.Y - sharedPt.Y) +
                           (pt.Z - sharedPt.Z) * (pt.Z - sharedPt.Z);
            EXPECT_LE(dist2, tol * tol * 100.0);
        }
    }
}
