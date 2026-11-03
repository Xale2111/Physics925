//
// Created by alexk on 29.10.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_BODYDEF_H_
#define GPR925PHYSICS_INCLUDE_GPR925_BODYDEF_H_
#include <optional>

#include "scalar.h"
#include "vec2.h"
#include "body_utils.h"
#include "collider.h"

namespace gpr925 {
// Everything needed to create a body.
struct BodyDef {
  Vec2 position{};
  Vec2 velocity{};
  Scalar mass{};
  BodyType type = BodyType::kDynamic;
  //if there's no collider, then the body can't have contact with anything (same logic as in Unity)
  std::optional<Collider> collider{};

};
}  // namespace gpr925
#endif  // GPR925PHYSICS_INCLUDE_GPR925_BODYDEF_H_
