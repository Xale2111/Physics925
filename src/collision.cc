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
  const Vec2 min_a_position = position_a + a.GetOffset() + a.GetMin();  // Current position of min of a
  const Vec2 max_a_position = position_a + a.GetOffset() + a.GetMax();  // Current position of max of a
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

bool OverlapCircleAABB(const CircleCollider& circle,const Vec2& circle_position, const BoxCollider& box,const Vec2& box_position) {
  const Vec2 center = circle_position + circle.GetOffset();
  const Vec2 box_min = box_position + box.GetOffset() + box.GetMin();
  const Vec2 box_max = box_position + box.GetOffset() + box.GetMax();

  const Vec2 closest_point{std::clamp(center.x(), box_min.x(), box_max.x()),std::clamp(center.y(), box_min.y(), box_max.y())};

  const Scalar radius = circle.GetRadius();
  return (center - closest_point).SqrMagnitude() <= radius * radius;
}

}  // namespace

bool Overlap(const Collider& a, const Vec2& position_a, const Collider& b,const Vec2& position_b) {
  if (a.GetType() == ColliderType::kCircle &&b.GetType() == ColliderType::kCircle)
  {
    return OverlapCircleCircle(static_cast<const CircleCollider&>(a), position_a,static_cast<const CircleCollider&>(b), position_b);
  }
  if (a.GetType() == ColliderType::kBox && b.GetType() == ColliderType::kBox) {
    return OverlapAABB(static_cast<const BoxCollider&>(a), position_a,static_cast<const BoxCollider&>(b), position_b);
  }

  if (a.GetType() == ColliderType::kCircle &&
      b.GetType() == ColliderType::kBox) {
    return OverlapCircleAABB(static_cast<const CircleCollider&>(a), position_a,static_cast<const BoxCollider&>(b), position_b);
  }
  if (a.GetType() == ColliderType::kBox &&
      b.GetType() == ColliderType::kCircle) {
    return OverlapCircleAABB(static_cast<const CircleCollider&>(b), position_b,static_cast<const BoxCollider&>(a), position_a);
  }
  return false;
}

Scalar CombineBounciness(const PhysicsMaterial& a, const PhysicsMaterial& b) {
  return (a.bounciness + b.bounciness) / Scalar{2};  // average of the bounciness
}

std::optional<Contact> CircleCircleContact(const CircleCollider& a,const Vec2& position_a,const CircleCollider& b,const Vec2& position_b) {
  const Vec2 delta = (position_b + b.GetOffset()) - (position_a + a.GetOffset()); //From A to B
  const Scalar radius_sum = a.GetRadius() + b.GetRadius();
  const Scalar distance_sqr = delta.SqrMagnitude();

  if (distance_sqr > radius_sum * radius_sum)
    return std::nullopt;  //no contact

  //centers are perfectly aligned, no direction given
  if (distance_sqr == Scalar{}) {
    return Contact{Vec2{Scalar{}, Scalar{1}}, radius_sum};
  }
  const Scalar distance = maths::NumberTraits<Scalar>::Sqrt(distance_sqr);
  return Contact{Vec2{delta.x() / distance,
                      delta.y() / distance},          //length 1 vector
               radius_sum - distance};         //penetration which each other
}

std::optional<Contact> BoxBoxContact(const BoxCollider& a, const Vec2& position_a,const BoxCollider& b, const Vec2& position_b) {
  const Vec2 min_a = position_a + a.GetOffset() + a.GetMin();
  const Vec2 max_a = position_a + a.GetOffset() + a.GetMax();
  const Vec2 min_b = position_b + b.GetOffset() + b.GetMin();
  const Vec2 max_b = position_b + b.GetOffset() + b.GetMax();

  //Zero or negative means the intervals do not cross: no contact.
  const Scalar overlap_x = std::min(max_a.x(), max_b.x()) - std::max(min_a.x(), min_b.x());
  const Scalar overlap_y = std::min(max_a.y(), max_b.y()) - std::max(min_a.y(), min_b.y());
  if (overlap_x <= Scalar{} || overlap_y <= Scalar{}) {
    return std::nullopt;
  }

  //The normal goes from A to B. B is "after" A on an axis when the sum
  //min+max (twice the center) of B is bigger than the one of A.
  if (overlap_x < overlap_y) {
    const bool b_is_right = (min_b.x() + max_b.x()) >= (min_a.x() + max_a.x());
    return Contact{Vec2{b_is_right ? Scalar{1} : -Scalar{1}, Scalar{}}, overlap_x};
  }
  const bool b_is_below = (min_b.y() + max_b.y()) >= (min_a.y() + max_a.y());
  return Contact{Vec2{Scalar{}, b_is_below ? Scalar{1} : -Scalar{1}}, overlap_y};
}

std::optional<Contact> CircleBoxContact(const CircleCollider& circle, const Vec2& circle_position,const BoxCollider& box, const Vec2& box_position) {
  const Vec2 center = circle_position + circle.GetOffset();
  const Vec2 box_min = box_position + box.GetOffset() + box.GetMin();
  const Vec2 box_max = box_position + box.GetOffset() + box.GetMax();
  const Scalar radius = circle.GetRadius();

  // The point of the box closest to the center (the course's clamp).
  const Vec2 closest{std::clamp(center.x(), box_min.x(), box_max.x()),std::clamp(center.y(), box_min.y(), box_max.y())};
  const Vec2 delta = closest - center;  // from the circle toward the box
  const Scalar distance_sqr = delta.SqrMagnitude();

  if (distance_sqr > radius * radius) return std::nullopt;  // too far

  if (distance_sqr > Scalar{}) {
    //Center outside the box: same as circle vs circle, the "other circle" being the closest point.
    const Scalar distance = maths::NumberTraits<Scalar>::Sqrt(distance_sqr);
    return Contact{Vec2{delta.x() / distance, delta.y() / distance}, radius - distance};
  }

  // Degenerate case: the center is INSIDE the box, so closest == center and
  // delta is zero (no direction). Push out through the nearest face.
  const Scalar to_left = center.x() - box_min.x();
  const Scalar to_right = box_max.x() - center.x();
  const Scalar to_top = center.y() - box_min.y();
  const Scalar to_bottom = box_max.y() - center.y();

  // The circle leaves along -normal (A moves along -n), so the normal is the
  // opposite of the direction of exit.
  Scalar nearest = to_left;
  Vec2 normal{Scalar{1}, Scalar{}};          // exit left  -> n = (+1, 0)
  if (to_right < nearest)  { nearest = to_right;  normal = Vec2{-Scalar{1}, Scalar{}}; }
  if (to_top < nearest)    { nearest = to_top;    normal = Vec2{Scalar{}, Scalar{1}}; }
  if (to_bottom < nearest) { nearest = to_bottom; normal = Vec2{Scalar{}, -Scalar{1}}; }
  return Contact{normal, radius + nearest};  // the whole circle must leave: r + d
}

std::optional<Contact> GenerateContact(const Collider& a, const Vec2& position_a,const Collider& b, const Vec2& position_b) {
  if (a.GetType() == ColliderType::kCircle && b.GetType() == ColliderType::kCircle) {
    return CircleCircleContact(static_cast<const CircleCollider&>(a), position_a,static_cast<const CircleCollider&>(b), position_b);
  }
  if (a.GetType() == ColliderType::kBox && b.GetType() == ColliderType::kBox) {
    return BoxBoxContact(static_cast<const BoxCollider&>(a), position_a,static_cast<const BoxCollider&>(b), position_b);
  }
  if (a.GetType() == ColliderType::kCircle && b.GetType() == ColliderType::kBox) {
    return CircleBoxContact(static_cast<const CircleCollider&>(a), position_a,static_cast<const BoxCollider&>(b), position_b);
  }
  if (a.GetType() == ColliderType::kBox && b.GetType() == ColliderType::kCircle) {
    //Same as circle with box but the normal is inverted
    const auto contact = CircleBoxContact(static_cast<const CircleCollider&>(b), position_b,static_cast<const BoxCollider&>(a), position_a);
    if (!contact) return std::nullopt;
    return Contact{Vec2{-contact->normal.x(), -contact->normal.y()}, contact->penetration};
  }
  return std::nullopt;
}

Rect ComputeAabb(const Collider& collider, const Vec2& position) {
  Rect aabb;
  if (collider.GetType() == ColliderType::kCircle) {
    const CircleCollider& circle = static_cast<const CircleCollider&>(collider);
    const Vec2 center = position + circle.GetOffset();
    const Vec2 radius{circle.GetRadius(), circle.GetRadius()};
    aabb.min_bound = center - radius;
    aabb.max_bound = center + radius;
  } else {
    const BoxCollider& box = static_cast<const BoxCollider&>(collider);
    aabb.min_bound = position + box.GetOffset() + box.GetMin();
    aabb.max_bound = position + box.GetOffset() + box.GetMax();
  }
  return aabb;
}

}  // namespace gpr925