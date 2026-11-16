//
// Created by alexk on 16.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_TRIGONOMETRY_H_
#define GPR925PHYSICS_INCLUDE_GPR925_TRIGONOMETRY_H_
#include "scalar.h"
#include "vec2.h"

namespace gpr925{

inline constexpr Scalar kPi = Scalar{3.14159265359f};
inline constexpr Scalar kTwoPi = Scalar{6.28318530718f};
inline constexpr Scalar kHalfPi = Scalar{1.57079632679f};

[[nodiscard]] Scalar WrapAngle(Scalar angle);

[[nodiscard]] Scalar Sin(Scalar angle);
[[nodiscard]] Scalar Cos(Scalar angle);

// The vector turned by `angle` around the origin:
//   x' = x*cos - y*sin
//   y' = x*sin + y*cos
[[nodiscard]] Vec2 Rotate(const Vec2& v, Scalar angle);

}

#endif  // GPR925PHYSICS_INCLUDE_GPR925_TRIGONOMETRY_H_
