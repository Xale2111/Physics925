//
// Created by alexk on 16.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_ORIENTED_BOX_COLLIDER_H_
#define GPR925PHYSICS_INCLUDE_GPR925_ORIENTED_BOX_COLLIDER_H_
#include <array>

#include "collider.h"
#include "physics_material.h"
#include "scalar.h"
#include "vec2.h"

namespace gpr925 {
class OrientedBoxCollider final : public Collider {
 public:
  // min and max are relative to the body position (then shifted by offset).
  OrientedBoxCollider(Vec2 min, Vec2 max, Vec2 offset, bool is_trigger, PhysicsMaterial material = {}) : Collider(ColliderType::kOrientedBox, offset, is_trigger, material), min_(min), max_(max) {}
  OrientedBoxCollider(Vec2 min, Vec2 max, bool is_trigger, PhysicsMaterial material = {}) : OrientedBoxCollider(min, max, Vec2{}, is_trigger, material) {}

  [[nodiscard]] const Vec2& GetMin() const { return min_; }
  [[nodiscard]] const Vec2& GetMax() const { return max_; }

  [[nodiscard]] Vec2 GetLocalCenter() const;
  [[nodiscard]] Vec2 GetHalfSize() const;
  [[nodiscard]] Vec2 GetWorldCenter(const Vec2& position, Scalar angle) const;

  [[nodiscard]] std::array<Vec2, 4> GetCorners(const Vec2& position, Scalar angle) const;

 private:
  Vec2 min_{};
  Vec2 max_{};
};
}
#endif  // GPR925PHYSICS_INCLUDE_GPR925_ORIENTED_BOX_COLLIDER_H_
