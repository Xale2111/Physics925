//
// Created by alexk on 29.10.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_BODY_UTILS_H_
#define GPR925PHYSICS_INCLUDE_GPR925_BODY_UTILS_H_

#include <cstddef>

namespace gpr925{
// Using size_t to have access to the bodies by index.
enum class BodyId : std::size_t;


enum class BodyType {
  kStatic,   // the object does not move
  kDynamic,  // forces are applied to it
};
}

#endif  // GPR925PHYSICS_INCLUDE_GPR925_BODY_UTILS_H_
