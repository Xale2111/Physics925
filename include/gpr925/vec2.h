#ifndef GPR925_VEC2_H_
#define GPR925_VEC2_H_

#include "gpr925/scalar.h"
#include "vector2.h"

namespace gpr925 {

/// 2D vector of @ref Scalar, with the same members as EngineLib's
/// `maths::Vector2`.
///
/// Why not `maths::Vector2<Scalar>` directly? On the DS, @ref Scalar is
/// @ref Fixed, a class, and EngineLib's `Number` concept only accepts built-in
/// arithmetic types: `maths::Vector2<gpr925::Fixed>` does not compile. Once
/// EngineLib accepts user-defined number types, replace this struct with
/// @code
/// using Vec2 = maths::Vector2<Scalar>;
/// @endcode
/// The members below mirror `maths::Vector2` so that the swap is that one
/// line.
/*
struct Vec2 {
  Scalar x{};  ///< Horizontal component, to the right.
  Scalar y{};  ///< Vertical component, pointing down on screen.

  /// Returns the squared length: no square root, use it to compare distances.
  [[nodiscard]] constexpr Scalar SqrMagnitude() const noexcept {
    return x * x + y * y;
  }
  /// Returns the length.
  [[nodiscard]] Scalar Magnitude() const noexcept {
    return Sqrt(SqrMagnitude());
  }

  /// Compares two vectors component by component.
  constexpr bool operator==(const Vec2& v) const noexcept = default;

  /// Adds two vectors.
  constexpr Vec2 operator+(const Vec2& v) const noexcept {
    return {x + v.x, y + v.y};
  }
  /// Subtracts two vectors.
  constexpr Vec2 operator-(const Vec2& v) const noexcept {
    return {x - v.x, y - v.y};
  }
  /// Multiplies two vectors component by component.
  constexpr Vec2 operator*(const Vec2& v) const noexcept {
    return {x * v.x, y * v.y};
  }

  /// Adds a vector in place.
  constexpr void operator+=(const Vec2& v) noexcept {
    x += v.x;
    y += v.y;
  }
  /// Subtracts a vector in place.
  constexpr void operator-=(const Vec2& v) noexcept {
    x -= v.x;
    y -= v.y;
  }
  /// Multiplies component by component in place.
  constexpr void operator*=(const Vec2& v) noexcept {
    x *= v.x;
    y *= v.y;
  }
};*/

using Vec2 = maths::Vector2<Scalar>;

/// Axis-aligned rectangle, from its top-left to its bottom-right corner.
struct Rect {
  Vec2 min_bound{};  ///< Smallest x and y: top-left on screen.
  Vec2 max_bound{};  ///< Largest x and y: bottom-right on screen.

  /// Returns the width and the height.
  [[nodiscard]] constexpr Vec2 Size() const noexcept {
    return max_bound - min_bound;
  }
  /// Returns the middle point.
  [[nodiscard]] constexpr Vec2 Center() const noexcept {
    return {(min_bound.x() + max_bound.x()) / 2, (min_bound.y() + max_bound.y()) / 2};
  }
};

}  // namespace gpr925

#endif  // GPR925_VEC2_H_
