//
// Created by alexk on 03.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_COLLISION_H_
#define GPR925PHYSICS_INCLUDE_GPR925_COLLISION_H_

#include <optional>
#include "box_collider.h"
#include "circle_collider.h"
#include "oriented_box_collider.h"
#include "vec2.h"

namespace gpr925 {

struct Contact {
  Vec2 normal;         // unit vector from A toward B
  Scalar penetration;  // how deep the two shapes overlap
  Vec2 point;          //where the contact touch (A touch B or B touch A) in world coordinates
};

[[nodiscard]] std::optional<Contact> CircleCircleContact(
    const CircleCollider& a, const Vec2& position_a,
    const CircleCollider& b, const Vec2& position_b);

[[nodiscard]] std::optional<Contact> BoxBoxContact(
    const BoxCollider& a, const Vec2& position_a,
    const BoxCollider& b, const Vec2& position_b);

// Normal goes from the circle toward the box.
[[nodiscard]] std::optional<Contact> CircleBoxContact(
    const CircleCollider& circle, const Vec2& circle_position,
    const BoxCollider& box, const Vec2& box_position);

[[nodiscard]] std::optional<Contact> CircleOrientedBoxContact(
    const CircleCollider& circle, const Vec2& circle_position,
    const OrientedBoxCollider& box, const Vec2& box_position, Scalar box_angle);

// Contact between any two colliders, nullopt if they do not touch
// (or if this pair of shapes is not supported yet).
[[nodiscard]] std::optional<Contact> GenerateContact(
    const Collider& a, const Vec2& position_a, Scalar angle_a,
    const Collider& b, const Vec2& position_b, Scalar angle_b);

// Same, for bodies that do not turn (angles are 0).
[[nodiscard]] std::optional<Contact> GenerateContact(
    const Collider& a, const Vec2& position_a,
    const Collider& b, const Vec2& position_b);

[[nodiscard]] bool Overlap(const Collider& a, const Vec2& position_a, const Collider& b, const Vec2& position_b);

[[nodiscard]] bool Overlap(const Collider& a, const Vec2& position_a, Scalar angle_a, const Collider& b, const Vec2& position_b, Scalar angle_b);

[[nodiscard]] Scalar CombineBounciness(const PhysicsMaterial& a, const PhysicsMaterial& b);

//If one of the surface friction is 0, then it's slippery no matter what the other is
[[nodiscard]] Scalar CombineFriction(const PhysicsMaterial& a, const PhysicsMaterial& b);

[[nodiscard]] Rect ComputeAabb(const Collider& collider, const Vec2& position);
// The AABB of a collider whose body is turned by `angle`: for a turned box it is
// the box around the four corners (bigger than the box itself).
[[nodiscard]] Rect ComputeAabb(const Collider& collider, const Vec2& position, Scalar angle);

}  // namespace gpr925

#endif  // GPR925PHYSICS_INCLUDE_GPR925_COLLISION_H_
