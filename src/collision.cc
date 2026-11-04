//
// Created by alexk on 03.11.2026.
//

#include "gpr925/collision.h"

#include <cassert>

#include "gpr925/circle_collider.h"

namespace gpr925 {

namespace {

// Circle vs circle: compare the squared distance between the centers with the
// squared sum of the radii, so no square root is needed. `<=`: touching counts.
bool OverlapCircleCircle(const CircleCollider& a, const Vec2& position_a,
                         const CircleCollider& b, const Vec2& position_b) {
  const Vec2 center_a = position_a + a.GetOffset();
  const Vec2 center_b = position_b + b.GetOffset();
  const Scalar sum_of_radii = a.GetRadius() + b.GetRadius();
  return (center_b - center_a).SqrMagnitude() <= sum_of_radii * sum_of_radii;
}

}  // namespace

bool Overlap(const Collider& a, const Vec2& position_a, const Collider& b,
             const Vec2& position_b) {
  // No RTTI on the DS (-fno-rtti): no dynamic_cast. GetType() tells us the real
  // type, so the static_cast down to it is safe.
  if (a.GetType() == ColliderType::kCircle &&
      b.GetType() == ColliderType::kCircle) {
    return OverlapCircleCircle(static_cast<const CircleCollider&>(a),
                               position_a,
                               static_cast<const CircleCollider&>(b),
                               position_b);
  }
  assert(false && "this pair of colliders has no overlap test yet");
  return false;
}

}  // namespace gpr925