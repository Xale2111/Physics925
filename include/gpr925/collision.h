//
// Created by alexk on 03.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_COLLISION_H_
#define GPR925PHYSICS_INCLUDE_GPR925_COLLISION_H_

#include "variant"

#include "circle_collider.h"
#include "box_collider.h"
#include "vec2.h"

namespace gpr925 {
[[nodiscard]] bool Overlap(const Collider& a, const Vec2& position_a, const Collider& b, const Vec2& position_b);


}  // namespace gpr925

#endif  // GPR925PHYSICS_INCLUDE_GPR925_COLLISION_H_
