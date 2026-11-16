//
// Created by alexk on 16.11.2026.
//

#include <gtest/gtest.h>

#include <cmath>

#include "gpr925/circle_collider.h"
#include "gpr925/collision.h"
#include "gpr925/oriented_box_collider.h"
#include "gpr925/trigonometry.h"

namespace gpr925 {
namespace {

// Whole numbers only for the inputs; the results go through sin/cos, so they
// are compared with a tolerance.
constexpr Scalar S(int value) { return static_cast<Scalar>(value); }
Vec2 V(int x, int y) { return Vec2{S(x), S(y)}; }
bool Near(Scalar value, int expected) {
  return std::fabs(ToFloat(value) - static_cast<float>(expected)) < 2e-3F;
}

// A 4 x 2 box (half size 2 x 1), centered on the body.
OrientedBoxCollider MakeBox() { return OrientedBoxCollider(V(-2, -1), V(2, 1), false); }

TEST(OrientedBox, CornersWithoutRotation) {
  const auto corners = MakeBox().GetCorners(V(0, 0), S(0));
  EXPECT_TRUE(Near(corners[0].x(), -2) && Near(corners[0].y(), -1));
  EXPECT_TRUE(Near(corners[1].x(), 2) && Near(corners[1].y(), -1));
  EXPECT_TRUE(Near(corners[2].x(), 2) && Near(corners[2].y(), 1));
  EXPECT_TRUE(Near(corners[3].x(), -2) && Near(corners[3].y(), 1));
}

// A quarter turn: (x, y) becomes (-y, x).
TEST(OrientedBox, CornersAfterAQuarterTurn) {
  const auto corners = MakeBox().GetCorners(V(0, 0), kHalfPi);
  EXPECT_TRUE(Near(corners[0].x(), 1) && Near(corners[0].y(), -2));
  EXPECT_TRUE(Near(corners[2].x(), -1) && Near(corners[2].y(), 2));
}

TEST(OrientedBox, OffsetTurnsWithTheBody) {
  const OrientedBoxCollider box(V(-1, -1), V(1, 1), V(1, 0), false);  // center 1 to the right
  const Vec2 center = box.GetWorldCenter(V(10, 5), kHalfPi);          // ... which becomes 1 down
  EXPECT_TRUE(Near(center.x(), 10) && Near(center.y(), 6));
}

TEST(OrientedBox, AabbOfATurnedBoxIsBigger) {
  const OrientedBoxCollider box = MakeBox();
  const Rect flat = ComputeAabb(box, V(10, 5), S(0));
  EXPECT_TRUE(Near(flat.min_bound.x(), 8) && Near(flat.max_bound.y(), 6));
  const Rect turned = ComputeAabb(box, V(10, 5), kHalfPi);  // 2 wide, 4 tall
  EXPECT_TRUE(Near(turned.min_bound.x(), 9) && Near(turned.max_bound.x(), 11));
  EXPECT_TRUE(Near(turned.min_bound.y(), 3) && Near(turned.max_bound.y(), 7));
}

// The box turned by a quarter turn is 2 wide, 4 tall: its right face is at x = 1.
TEST(CircleOrientedBox, HitOnTheSideFace) {
  const CircleCollider circle(S(2), V(0, 0), false);
  const auto contact = CircleOrientedBoxContact(circle, V(2, 0), MakeBox(), V(0, 0), kHalfPi);
  EXPECT_TRUE(contact.has_value());
  EXPECT_TRUE(Near(contact->normal.x(), -1) && Near(contact->normal.y(), 0));  // circle -> box
  EXPECT_TRUE(Near(contact->penetration, 1));
  EXPECT_TRUE(Near(contact->point.x(), 1) && Near(contact->point.y(), 0));
}

// ... and its top face is at y = -2 (y points down).
TEST(CircleOrientedBox, HitOnTheTopFace) {
  const CircleCollider circle(S(2), V(0, 0), false);
  const auto contact = CircleOrientedBoxContact(circle, V(0, -3), MakeBox(), V(0, 0), kHalfPi);
  EXPECT_TRUE(contact.has_value());
  EXPECT_TRUE(Near(contact->normal.x(), 0) && Near(contact->normal.y(), 1));
  EXPECT_TRUE(Near(contact->penetration, 1));
  EXPECT_TRUE(Near(contact->point.x(), 0) && Near(contact->point.y(), -2));
}

TEST(CircleOrientedBox, TooFarGivesNoContact) {
  const CircleCollider circle(S(1), V(0, 0), false);
  EXPECT_FALSE(CircleOrientedBoxContact(circle, V(5, 0), MakeBox(), V(0, 0), kHalfPi).has_value());
}

// Center inside the box: pushed out through the nearest face of the TURNED box.
TEST(CircleOrientedBox, CenterInsideLeavesByTheNearestFace) {
  const CircleCollider circle(S(1), V(0, 0), false);
  const auto contact = CircleOrientedBoxContact(circle, V(0, 0), MakeBox(), V(0, 0), kHalfPi);
  EXPECT_TRUE(contact.has_value());
  EXPECT_TRUE(Near(contact->normal.x(), -1) && Near(contact->normal.y(), 0));
  EXPECT_TRUE(Near(contact->penetration, 2));  // radius 1 + 1 to the face
}

// Angle 0: the result is the one of the AABB version.
TEST(CircleOrientedBox, WithoutRotationItIsTheAabbResult) {
  const CircleCollider circle(S(2), V(0, 0), false);
  const BoxCollider aabb(V(-5, 0), V(5, 4), false);
  const OrientedBoxCollider obb(V(-5, 0), V(5, 4), false);
  const auto expected = CircleBoxContact(circle, V(1, -1), aabb, V(0, 0));
  const auto got = CircleOrientedBoxContact(circle, V(1, -1), obb, V(0, 0), S(0));
  EXPECT_TRUE(expected.has_value() && got.has_value());
  EXPECT_TRUE(Near(got->normal.x(), 0) && Near(got->normal.y(), 1));
  EXPECT_TRUE(std::fabs(ToFloat(got->penetration) - ToFloat(expected->penetration)) < 2e-3F);
  EXPECT_TRUE(std::fabs(ToFloat(got->point.x()) - ToFloat(expected->point.x())) < 2e-3F);
  EXPECT_TRUE(std::fabs(ToFloat(got->point.y()) - ToFloat(expected->point.y())) < 2e-3F);
}

TEST(GenerateContactOriented, BothOrdersFlipTheNormal) {
  const CircleCollider circle(S(2), V(0, 0), false);
  const OrientedBoxCollider box = MakeBox();
  const auto circle_first = GenerateContact(circle, V(2, 0), S(0), box, V(0, 0), kHalfPi);
  const auto box_first = GenerateContact(box, V(0, 0), kHalfPi, circle, V(2, 0), S(0));
  EXPECT_TRUE(circle_first.has_value() && box_first.has_value());
  EXPECT_TRUE(Near(circle_first->normal.x(), -1));  // from A (circle) to B (box)
  EXPECT_TRUE(Near(box_first->normal.x(), 1));      // from A (box) to B (circle)
  EXPECT_TRUE(Overlap(circle, V(2, 0), S(0), box, V(0, 0), kHalfPi));
  EXPECT_FALSE(Overlap(circle, V(9, 0), S(0), box, V(0, 0), kHalfPi));
}

}  // namespace
}  // namespace gpr925