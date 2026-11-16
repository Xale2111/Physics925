//
// Created by alexk on 16.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_INERTIA_H_
#define GPR925PHYSICS_INCLUDE_GPR925_INERTIA_H_
#include "scalar.h"

namespace gpr925 {

//Disc rotation = 1/2 * mass * radius^2
[[nodiscard]] constexpr Scalar DiscInertia(Scalar mass, Scalar radius)
{
  return mass * radius*radius / Scalar{2};
}

//Box Rotation = m(w*w + h*h)/12
[[nodiscard]] constexpr Scalar DiscInertia(Scalar mass, Scalar width, Scalar height)
{
  return (mass * (width*width + height *height)) / Scalar{12};
}

// Full rectangle width x height about its center: I = m * (w^2 + h^2) / 12.
[[nodiscard]] constexpr Scalar BoxInertia(Scalar mass, Scalar width, Scalar height) {
  return mass * (width * width + height * height) / Scalar{12};
}

}  // namespace gpr925
#endif  // GPR925PHYSICS_INCLUDE_GPR925_INERTIA_H_
