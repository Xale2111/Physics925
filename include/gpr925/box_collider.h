//
// Created by alexk on 04.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_BOX_COLLIDER_H_
#define GPR925PHYSICS_INCLUDE_GPR925_BOX_COLLIDER_H_

#include "collider.h"
#include "scalar.h"
#include "vec2.h"

namespace gpr925 {

class BoxCollider final : public Collider {
 public:
  // min and max are relative to the body position (then shifted by offset).
  BoxCollider(Vec2 min, Vec2 max, Vec2 offset, bool is_trigger)
      : Collider(ColliderType::kBox, offset, is_trigger), min_(min), max_(max) {}
  BoxCollider(Vec2 min, Vec2 max, bool is_trigger) : BoxCollider(min, max, Vec2{}, is_trigger) {}

  [[nodiscard]] const Vec2& GetMin() const { return min_; }
  [[nodiscard]] const Vec2& GetMax() const { return max_; }

 private:
  Vec2 min_{};
  Vec2 max_{};
};

}  // namespace gpr925
#endif  // GPR925PHYSICS_INCLUDE_GPR925_BOX_COLLIDER_H_