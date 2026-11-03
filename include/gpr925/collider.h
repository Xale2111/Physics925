//
// Created by alexk on 03.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_H_
#define GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_H_
#include <variant>

#include "scalar.h"
#include "vec2.h"

namespace gpr925 {

// The shapes a collider can have
struct CircleShape {
  Scalar radius{};
};

struct BoxShape {
  Vec2 min{};
  Vec2 max{};
};

// I can now use Shape with either a Circle or a Box
using Shape = std::variant<CircleShape, BoxShape>;

struct Collider {
  Shape shape{CircleShape{}};
  Vec2 offset{};
  // Works like Unity IsTrigger (true -> sensor, false -> physical collider)
  bool is_trigger = false;
};

[[nodiscard]] inline Collider MakeCircleCollider(Scalar radius,
                                                 bool is_trigger = false,
                                                 Vec2 offset = {}) {
  return Collider{CircleShape{radius}, offset, is_trigger};
}

}  // namespace gpr925
#endif  // GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_H_
