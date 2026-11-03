//
// Created by alexk on 03.11.2026.
//

#include "gpr925/collision.h"

namespace gpr925 {

namespace {

//circle with circle overlap
//using the square distance to avoid sqrt
bool OverlapShapes(const CircleShape& a, const Vec2& center_a,
                   const CircleShape& b, const Vec2& center_b) {
  const Scalar sum_of_radius = a.radius + b.radius;
  return (center_b - center_a).SqrMagnitude() <= sum_of_radius * sum_of_radius;
}

//TODO : Implement for other shapes
bool OverlapShapes(const auto&, const Vec2&, const auto&, const Vec2&) {
  return false;
}

}  // namespace

bool Overlap(const Collider& a, const Vec2& position_a, const Collider& b,
             const Vec2& position_b) {
  const Vec2 center_a = position_a + a.offset;
  const Vec2 center_b = position_b + b.offset;
  // visit calls OverlapShapes with the two real shape types.
  return std::visit(
      [&](const auto& shape_a, const auto& shape_b) {
        return OverlapShapes(shape_a, center_a, shape_b, center_b);
      },
      a.shape, b.shape);
}

}  // namespace gpr925
