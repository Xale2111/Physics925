//
// Created by alexk on 03.11.2026.
//

#include <gtest/gtest.h>

#include "gpr925/collider.h"
#include "gpr925/collision.h"

namespace gpr925 {
namespace {

Vec2 V(int x, int y) { return Vec2{Ratio(x, 1), Ratio(y, 1)}; }

Collider Circle(int radius, bool is_trigger = false) {
  return MakeCircleCollider(Ratio(radius, 1), is_trigger);
}

// ============================================================
// Collider
// ============================================================

TEST(ColliderTest, DefaultColliderShouldBeAnEmptySolidCircle) {
  const Collider collider;
  ASSERT_TRUE(std::holds_alternative<CircleShape>(collider.shape));
  EXPECT_EQ(std::get<CircleShape>(collider.shape).radius, Scalar{});
  EXPECT_EQ(collider.offset, Vec2{});
  EXPECT_FALSE(collider.is_trigger);
}

TEST(ColliderTest, MakeCircleColliderShouldStoreItsArguments) {
  const Collider collider =
      MakeCircleCollider(Ratio(3, 2), true, Vec2{Ratio(1, 1), Ratio(2, 1)});
  ASSERT_TRUE(std::holds_alternative<CircleShape>(collider.shape));
  EXPECT_EQ(std::get<CircleShape>(collider.shape).radius, Ratio(3, 2));
  EXPECT_EQ(collider.offset, V(1, 2));
  EXPECT_TRUE(collider.is_trigger);
}

TEST(ColliderTest, ShouldBeAbleToHoldABox) {
  Collider collider;
  collider.shape = BoxShape{V(2, 1)};
  ASSERT_TRUE(std::holds_alternative<BoxShape>(collider.shape));
  EXPECT_EQ(std::get<BoxShape>(collider.shape).half_size, V(2, 1));
}

// ============================================================
// Circle vs circle
// ============================================================

TEST(OverlapCircleTest, SeparatedCirclesShouldNotOverlap) {
  EXPECT_FALSE(Overlap(Circle(1), V(0, 0), Circle(1), V(3, 0)));
  EXPECT_FALSE(Overlap(Circle(1), V(0, 0), Circle(1), V(0, -3)));
}

TEST(OverlapCircleTest, TouchingCirclesShouldOverlap) {
  // Distance 2 = 1 + 1: touching counts as overlapping.
  EXPECT_TRUE(Overlap(Circle(1), V(0, 0), Circle(1), V(2, 0)));
  EXPECT_TRUE(Overlap(Circle(1), V(0, 0), Circle(1), V(0, 2)));
}

TEST(OverlapCircleTest, CirclesJustOutsideShouldNotOverlap) {
  const Vec2 just_outside{Ratio(33, 16), Ratio(0, 1)};  // 2.0625 > 2
  EXPECT_FALSE(Overlap(Circle(1), V(0, 0), Circle(1), just_outside));
}

TEST(OverlapCircleTest, CirclesJustInsideShouldOverlap) {
  const Vec2 just_inside{Ratio(31, 16), Ratio(0, 1)};  // 1.9375 < 2
  EXPECT_TRUE(Overlap(Circle(1), V(0, 0), Circle(1), just_inside));
}

TEST(OverlapCircleTest, OverlappingCirclesShouldOverlap) {
  EXPECT_TRUE(Overlap(Circle(2), V(0, 0), Circle(2), V(3, 0)));
}

TEST(OverlapCircleTest, ConcentricCirclesShouldOverlap) {
  EXPECT_TRUE(Overlap(Circle(1), V(4, 4), Circle(1), V(4, 4)));
}

TEST(OverlapCircleTest, CircleInsideABiggerOneShouldOverlap) {
  EXPECT_TRUE(Overlap(Circle(5), V(0, 0), Circle(1), V(1, 1)));
}

TEST(OverlapCircleTest, DifferentRadiiShouldBeAdded) {
  // Distance 5, radii 2 + 3 = 5: touching. With 2 + 2 = 4: apart.
  EXPECT_TRUE(Overlap(Circle(2), V(0, 0), Circle(3), V(5, 0)));
  EXPECT_FALSE(Overlap(Circle(2), V(0, 0), Circle(2), V(5, 0)));
}

TEST(OverlapCircleTest, DiagonalDistanceShouldBeMeasuredOnBothAxes) {
  // Centers (1, 1) apart: distance about 1.414.
  const Collider small = MakeCircleCollider(Ratio(1, 2));   // radii sum 1.0
  const Collider big = MakeCircleCollider(Ratio(3, 4));     // radii sum 1.5
  EXPECT_FALSE(Overlap(small, V(0, 0), small, V(1, 1)));
  EXPECT_TRUE(Overlap(big, V(0, 0), big, V(1, 1)));
}

TEST(OverlapCircleTest, ShouldWorkWithNegativeCoordinates) {
  EXPECT_TRUE(Overlap(Circle(1), V(-5, -5), Circle(1), V(-6, -5)));
  EXPECT_FALSE(Overlap(Circle(1), V(-5, -5), Circle(1), V(5, 5)));
}

TEST(OverlapCircleTest, ZeroRadiusCirclesShouldOverlapOnlyWhenTheyCoincide) {
  EXPECT_TRUE(Overlap(Circle(0), V(2, 2), Circle(0), V(2, 2)));
  EXPECT_FALSE(Overlap(Circle(0), V(2, 2), Circle(0), V(3, 2)));
}

TEST(OverlapCircleTest, ResultShouldNotDependOnTheOrderOfTheArguments) {
  const Vec2 positions[] = {V(0, 0), V(2, 0), V(3, 0), V(1, 1), V(-4, 2)};
  for (const Vec2& pa : positions) {
    for (const Vec2& pb : positions) {
      EXPECT_EQ(Overlap(Circle(1), pa, Circle(2), pb),
                Overlap(Circle(2), pb, Circle(1), pa));
    }
  }
}

TEST(OverlapCircleTest, TriggerFlagShouldNotChangeTheGeometry) {
  EXPECT_EQ(Overlap(Circle(1, false), V(0, 0), Circle(1, false), V(2, 0)),
            Overlap(Circle(1, true), V(0, 0), Circle(1, true), V(2, 0)));
  EXPECT_EQ(Overlap(Circle(1, false), V(0, 0), Circle(1, false), V(5, 0)),
            Overlap(Circle(1, true), V(0, 0), Circle(1, false), V(5, 0)));
}

// ============================================================
// Offset
// ============================================================

TEST(OverlapCircleTest, OffsetShouldMoveTheShapeAwayFromTheBody) {
  // The body of `a` is at the origin but its circle is centered on (3, 0).
  const Collider a = MakeCircleCollider(Ratio(1, 1), false, V(3, 0));
  EXPECT_TRUE(Overlap(a, V(0, 0), Circle(1), V(3, 0)));
  EXPECT_FALSE(Overlap(a, V(0, 0), Circle(1), V(0, 0)));
}

TEST(OverlapCircleTest, OffsetsOfBothColliderShouldBeApplied) {
  const Collider a = MakeCircleCollider(Ratio(1, 1), false, V(2, 0));
  const Collider b = MakeCircleCollider(Ratio(1, 1), false, V(-2, 0));
  // Centers: (2, 0) and (4 - 2, 0) = (2, 0): they coincide.
  EXPECT_TRUE(Overlap(a, V(0, 0), b, V(4, 0)));
  EXPECT_FALSE(Overlap(a, V(0, 0), b, V(10, 0)));
}

}  // namespace
}  // namespace gpr925