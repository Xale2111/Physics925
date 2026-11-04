//
// Created by alexk on 04.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_UTILS_H_
#define GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_UTILS_H_
#include <cstddef>

namespace gpr925 {
// Using size_t to have access to the Colliders by index.
enum class ColliderId : std::size_t;
enum class ColliderType : std::uint8_t { kCircle, kBox };

}
#endif  // GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_UTILS_H_
