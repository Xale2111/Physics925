#ifndef GPR925_SCALAR_H_
#define GPR925_SCALAR_H_

#include <cmath>

#include "gpr925/fixed_point.h"

namespace gpr925 {

/// The real number type of the engine and the samples.
///
/// It is chosen by the CMake cache variable `GPR925_SCALAR`:
/// - `FLOAT` (Windows and Linux presets): plain `float`;
/// - `FIXED` (`nds` preset, and `debug-fixed` to debug the DS behaviour on
///   PC): @ref Fixed, the 20.12 fixed-point class.
///
/// Code that only uses `Scalar`, the four operators and the helpers of this
/// header (@ref Ratio, `Sqrt`, `Abs`, `ToInt`, `ToFloat`) builds on every
/// platform. What does not port:
/// - float literals in run-time code: write `Ratio(1, 2)` or a `constexpr
///   Scalar{0.5f}` constant instead of `0.5f * x`;
/// - `std::sqrt`, `std::abs`: use `Sqrt` and `Abs`;
/// - `static_cast<Scalar>(some_float)`: there is no run-time float on the DS.
///
/// The simulation is not bit-identical between `FLOAT` and `FIXED`.
#if defined(GPR925_SCALAR_FIXED)
using Scalar = Fixed;
#else
using Scalar = float;
#endif

/// Returns the square root of a float (`Sqrt(Fixed)` is in fixed.h).
[[nodiscard]] inline float Sqrt(float value) noexcept {
  return std::sqrt(value);
}

/// Returns the absolute value of a float.
[[nodiscard]] constexpr float Abs(float value) noexcept {
  return value < 0.0f ? -value : value;
}

/// Returns the float itself: the float side of `ToFloat(Fixed)`.
[[nodiscard]] constexpr float ToFloat(float value) noexcept { return value; }

/// Converts a float to an integer, truncating toward zero.
[[nodiscard]] constexpr int ToInt(float value) noexcept {
  return static_cast<int>(value);
}

/// Converts an integer to @ref Scalar.
[[nodiscard]] constexpr Scalar ToScalar(int value) noexcept {
  return static_cast<Scalar>(value);
}

/// Returns `num / den` computed in @ref Scalar.
///
/// Write constants with it instead of float literals: `Ratio(981, 100)`
/// rather than `9.81f`. With @ref Fixed the division happens in fixed point,
/// so no float arithmetic ends up in the DS ROM.
///
/// @param num Numerator.
/// @param den Denominator, not zero.
[[nodiscard]] constexpr Scalar Ratio(int num, int den) noexcept {
  return ToScalar(num) / ToScalar(den);
}

}  // namespace gpr925

#endif  // GPR925_SCALAR_H_
