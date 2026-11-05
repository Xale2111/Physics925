//
// Created by alexk on 03.11.2026.
//

#include "gpr925/collision.h"

#include <algorithm>
#include <cassert>

#include "gpr925/box_collider.h"
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

bool OverlapAABB(const BoxCollider& a, const Vec2& position_a,
                 const BoxCollider& b, const Vec2& position_b) {
  const Vec2 min_a_position = position_a + a.GetOffset() + a.GetMin(); //Current position of min of a
  const Vec2 max_a_position = position_a + a.GetOffset() + a.GetMax(); //Current position of max of a
  const Vec2 min_b_position = position_b + b.GetOffset() + b.GetMin();
  const Vec2 max_b_position = position_b + b.GetOffset() + b.GetMax();

  const bool min_a_x_in_b_x = min_a_position.x() <= max_b_position.x();
  const bool min_a_y_in_b_y = min_a_position.y() <= max_b_position.y();
  const bool max_a_x_in_b_x = max_a_position.x() >= min_b_position.x();
  const bool max_a_y_in_b_y = max_a_position.y() >= min_b_position.y();

  //Check for overlap in x and y
  const bool overlap_x = min_a_x_in_b_x && max_a_x_in_b_x;
  const bool overlap_y = min_a_y_in_b_y && max_a_y_in_b_y;

  //if both x and y overlap, return true
  return overlap_x && overlap_y;
}

bool OverlapCircleAABB(const CircleCollider& circle, const Vec2& circle_position,
                       const BoxCollider& box, const Vec2& box_position) {
  const Vec2 center = circle_position + circle.GetOffset();
  const Vec2 box_min = box_position + box.GetOffset() + box.GetMin();
  const Vec2 box_max = box_position + box.GetOffset() + box.GetMax();

  const Vec2 closest_point{std::clamp(center.x(), box_min.x(), box_max.x()),
                           std::clamp(center.y(), box_min.y(), box_max.y())};

  const Scalar radius = circle.GetRadius();
  return (center - closest_point).SqrMagnitude() <= radius * radius;
}

}  // namespace

bool Overlap(const Collider& a, const Vec2& position_a, const Collider& b,
             const Vec2& position_b) {
  if (a.GetType() == ColliderType::kCircle &&
      b.GetType() == ColliderType::kCircle) {
    return OverlapCircleCircle(static_cast<const CircleCollider&>(a),
                               position_a,
                               static_cast<const CircleCollider&>(b),
                               position_b);
  }
  if (a.GetType() == ColliderType::kBox && b.GetType() == ColliderType::kBox) {
    return OverlapAABB(static_cast<const BoxCollider&>(a), position_a,
                       static_cast<const BoxCollider&>(b), position_b);
  }

  if (a.GetType() == ColliderType::kCircle &&
      b.GetType() == ColliderType::kBox) {
    return OverlapCircleAABB(static_cast<const CircleCollider&>(a), position_a,
                             static_cast<const BoxCollider&>(b), position_b);
  }
  if (a.GetType() == ColliderType::kBox &&
      b.GetType() == ColliderType::kCircle) {

    return OverlapCircleAABB(static_cast<const CircleCollider&>(b), position_b,
                             static_cast<const BoxCollider&>(a), position_a);
  }
  assert(false && "this pair of colliders has no overlap test yet");
  return false;
}

}  // namespace gpr925