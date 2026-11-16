//
// Created by alexk on 16.11.2026.
//
#include "gpr925/trigonometry.h"

#include <array>
#include <cmath>
#include <cstddef>

namespace gpr925 {

namespace {
// 1 / (2 pi): turns per radian.
constexpr Scalar kInverseTwoPi = Scalar{0.15915494309f};
}  // namespace

Scalar WrapAngle(Scalar angle) {
  if (angle >= -kPi && angle <= kPi) {
    return angle;
  }
  // ToInt truncates toward zero: this removes the whole turns...
  const int turns = ToInt(angle * kInverseTwoPi);
  angle -= ToScalar(turns) * kTwoPi;
  // ...and what is left is in (-2 pi, 2 pi): one more correction at most.
  if (angle > kPi) {
    angle -= kTwoPi;
  } else if (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

#if defined(GPR925_SCALAR_FIXED)

namespace {

constexpr int kTableSize = 1024;  // entries over one turn (2 pi)

// kTableSize + 1 entries: the last one repeats the first, so the interpolation
// between entry 1023 and "entry 1024" needs no wrap-around.
// The values are in sin_table.inc (see why there). `constexpr`: the compiler
// computes everything, the table ends up in the ROM.
constexpr std::array<Scalar, kTableSize + 1> kSinTable = {{
#include "sin_table.inc"
}};

// Table steps per radian: 1024 / (2 pi).
constexpr Scalar kStepsPerRadian = Scalar{162.974662f};

}  // namespace

Scalar Sin(Scalar angle) {
  angle = WrapAngle(angle);  // [-pi, pi]: the position below is in [-512, 512]
  const Scalar position = angle * kStepsPerRadian;
  int index = ToInt(position);                    // toward zero
  Scalar fraction = position - ToScalar(index);   // same sign as position
  if (fraction < Scalar{}) {                      // negative angle: go one entry back
    --index;
    fraction += Scalar{1};
  }
  index = ((index % kTableSize) + kTableSize) % kTableSize;  // 0 .. kTableSize-1
  const std::size_t i = static_cast<std::size_t>(index);
  // Linear interpolation between two entries.
  return kSinTable[i] + (kSinTable[i + 1] - kSinTable[i]) * fraction;
}

#else  // FLOAT

Scalar Sin(Scalar angle) { return std::sin(angle); }

#endif

Scalar Cos(Scalar angle) { return Sin(angle + kHalfPi); }  // cos(x) = sin(x + pi/2)

Vec2 Rotate(const Vec2& v, Scalar angle) {
  const Scalar c = Cos(angle);
  const Scalar s = Sin(angle);
  return Vec2{v.x() * c - v.y() * s, v.x() * s + v.y() * c};
}

}  // namespace gpr925