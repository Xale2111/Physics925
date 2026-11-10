//
// Created by alexk on 04.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_CIRCLE_COLLIDER_H_
#define GPR925PHYSICS_INCLUDE_GPR925_CIRCLE_COLLIDER_H_

#include "collider.h"
#include "scalar.h"
#include "vec2.h"

namespace gpr925 {

class CircleCollider final : public Collider {
 public:
  CircleCollider(Scalar radius, Vec2 offset, bool is_trigger, PhysicsMaterial material = {})
      : Collider(ColliderType::kCircle, offset, is_trigger, material), radius_(radius) {}
  CircleCollider(Scalar radius, bool is_trigger, PhysicsMaterial material = {})
      : CircleCollider(radius, Vec2{}, is_trigger, material) {}

  [[nodiscard]] Scalar GetRadius() const { return radius_; }

 private:
  Scalar radius_;
};

}  // namespace gpr925
#endif  // GPR925PHYSICS_INCLUDE_GPR925_CIRCLE_COLLIDER_H_