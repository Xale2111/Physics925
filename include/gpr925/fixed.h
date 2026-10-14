#ifndef GPR925_FIXED_H_
#define GPR925_FIXED_H_

#include <compare>
#include <cstdint>

#if defined(GPR925_PLATFORM_NDS)
#include <nds/arm9/math.h>
#endif

namespace gpr925 {

namespace internal {
// Portable versions of libnds' mulf32, divf32 and sqrtf32 (nds/arm9/math.h),
// used on PC and for constant evaluation. They give the same bits as the DS.

[[nodiscard]] constexpr std::int32_t MulF32Portable(std::int32_t a,
                                                    std::int32_t b) noexcept {
  return static_cast<std::int32_t>((std::int64_t{a} * std::int64_t{b}) >> 12);
}

// den must not be 0. Truncates toward zero, like the DS divider.
[[nodiscard]] constexpr std::int32_t DivF32Portable(std::int32_t num,
                                                    std::int32_t den) noexcept {
  return static_cast<std::int32_t>((std::int64_t{num} * 4096) /
                                   std::int64_t{den});
}

// Floor of the square root, computed bit by bit. A negative value gives 0.
[[nodiscard]] constexpr std::int32_t SqrtF32Portable(std::int32_t a) noexcept {
  if (a <= 0) {
    return 0;
  }
  std::uint64_t value = static_cast<std::uint64_t>(a) << 12;
  std::uint64_t result = 0;
  std::uint64_t bit = std::uint64_t{1} << 62;
  while (bit > value) {
    bit >>= 2;
  }
  while (bit != 0) {
    if (value >= result + bit) {
      value -= result + bit;
      result = (result >> 1) + bit;
    } else {
      result >>= 1;
    }
    bit >>= 2;
  }
  return static_cast<std::int32_t>(result);
}

// On the DS, the run-time path calls libnds. divf32 and sqrtf32 use the
// hardware divider and square root unit: these are shared memory-mapped
// registers, so Fixed division and Sqrt must not run inside an interrupt
// handler (an interrupted operation would return the handler's result).

[[nodiscard]] constexpr std::int32_t MulF32(std::int32_t a,
                                            std::int32_t b) noexcept {
#if defined(GPR925_PLATFORM_NDS)
  if !consteval {
    return mulf32(a, b);
  }
#endif
  return MulF32Portable(a, b);
}

[[nodiscard]] constexpr std::int32_t DivF32(std::int32_t num,
                                            std::int32_t den) noexcept {
#if defined(GPR925_PLATFORM_NDS)
  if !consteval {
    return divf32(num, den);
  }
#endif
  return DivF32Portable(num, den);
}

[[nodiscard]] constexpr std::int32_t SqrtF32(std::int32_t a) noexcept {
#if defined(GPR925_PLATFORM_NDS)
  if !consteval {
    return a <= 0 ? 0 : sqrtf32(a);
  }
#endif
  return SqrtF32Portable(a);
}
}  // namespace internal

/// A 20.12 fixed-point number: libnds' `f32` wrapped in a class.
///
/// The Nintendo DS CPU (ARM946E-S) has no FPU: every `float` operation is a
/// call to a software routine, tens of times slower than integer math. A
/// fixed-point number is an integer `raw()` that means `raw() / 4096`: the top
/// 20 bits hold the integer part and the low 12 bits the fraction. `1.0` is
/// stored as `4096`, the smallest step is `1 / 4096` (about 0.00024) and the
/// range is about -524288 to 524287.99.
///
/// libnds already has this format (`nds/arm9/math.h`), but as a plain `s32`
/// with functions: `mulf32(a, b)`, `divf32(a, b)`, `sqrtf32(a)`. Nothing
/// stops you from writing `a * b` on two such integers and getting a result
/// 4096 times too big. This class makes the compiler do the right thing:
/// `*` calls `mulf32`, `/` calls `divf32` and @ref Sqrt calls `sqrtf32`.
///
/// @code
/// using gpr925::Fixed;
/// constexpr Fixed three{3};          // from int
/// constexpr Fixed half{0.5f};        // from a float constant, at compile time
/// Fixed x = three * half;            // 1.5
/// x += Fixed{1} / Fixed{3};
/// const int pixels = static_cast<int>(x * 20);
/// @endcode
///
/// Rules to remember:
/// - multiplication and division truncate (the last bit is not rounded);
/// - dividing by zero is an error (don't);
/// - overflow is not checked: keep values and intermediate products well
///   inside the range. A squared length overflows past about 724 units;
/// - division and @ref Sqrt use the DS hardware units, so never call them
///   from an interrupt handler.
///
/// @note In samples and engine code, write @ref Scalar instead: it is `Fixed`
/// on the DS and `float` on PC.
class Fixed {
 public:
  /// Number of fractional bits.
  static constexpr int kFractionBits = 12;
  /// Raw value of 1.0.
  static constexpr std::int32_t kOne = std::int32_t{1} << kFractionBits;

  /// Constructs zero.
  constexpr Fixed() = default;
  /// Converts an integer (libnds' `inttof32`).
  ///
  /// @param value Integer value, within the range of the type.
  constexpr explicit Fixed(int value) noexcept
      : raw_(std::int32_t{value} * kOne) {}
  /// Converts a float constant, at compile time only (libnds' `floattof32`,
  /// rounded to nearest).
  ///
  /// It is `consteval`: `Fixed{0.5f}` compiles, but converting a float
  /// variable does not, so no soft-float code can sneak into the DS ROM this
  /// way.
  ///
  /// @param value Constant to convert.
  consteval explicit Fixed(float value) noexcept
      : raw_(static_cast<std::int32_t>(value * 4096.0f +
                                       (value < 0.0f ? -0.5f : 0.5f))) {}

  /// Builds a value from its raw 20.12 integer, such as a libnds `f32`.
  ///
  /// @param raw The value multiplied by 4096.
  /// @return The fixed-point number `raw / 4096`.
  [[nodiscard]] static constexpr Fixed FromRaw(std::int32_t raw) noexcept {
    Fixed result;
    result.raw_ = raw;
    return result;
  }
  /// Returns the raw 20.12 integer, to pass to libnds functions.
  [[nodiscard]] constexpr std::int32_t raw() const noexcept { return raw_; }

  /// Converts to an integer, truncating toward zero (libnds' `f32toint`).
  constexpr explicit operator int() const noexcept { return int{raw_ / kOne}; }
  /// Converts to float (libnds' `f32tofloat`).
  ///
  /// For tests and debug display on PC. On the DS it pulls in soft-float.
  constexpr explicit operator float() const noexcept {
    return static_cast<float>(raw_) / 4096.0f;
  }

  /// Compares two values.
  friend constexpr bool operator==(Fixed, Fixed) noexcept = default;
  /// Orders two values.
  friend constexpr std::strong_ordering operator<=>(Fixed,
                                                    Fixed) noexcept = default;

  /// Returns the opposite value.
  constexpr Fixed operator-() const noexcept { return FromRaw(-raw_); }

  /// Adds two values.
  friend constexpr Fixed operator+(Fixed a, Fixed b) noexcept {
    return FromRaw(a.raw_ + b.raw_);
  }
  /// Subtracts two values.
  friend constexpr Fixed operator-(Fixed a, Fixed b) noexcept {
    return FromRaw(a.raw_ - b.raw_);
  }
  /// Multiplies two values (libnds' `mulf32`).
  friend constexpr Fixed operator*(Fixed a, Fixed b) noexcept {
    return FromRaw(internal::MulF32(a.raw_, b.raw_));
  }
  /// Divides two values (libnds' `divf32`, the DS hardware divider).
  ///
  /// @param a Numerator.
  /// @param b Denominator, not zero.
  friend constexpr Fixed operator/(Fixed a, Fixed b) noexcept {
    return FromRaw(internal::DivF32(a.raw_, b.raw_));
  }

  /// Multiplies by an integer: a plain integer multiply, cheaper than
  /// `a * Fixed{b}`.
  friend constexpr Fixed operator*(Fixed a, int b) noexcept {
    return FromRaw(a.raw_ * std::int32_t{b});
  }
  /// Multiplies an integer by a value.
  friend constexpr Fixed operator*(int a, Fixed b) noexcept { return b * a; }
  /// Divides by an integer, not zero.
  friend constexpr Fixed operator/(Fixed a, int b) noexcept {
    return FromRaw(a.raw_ / std::int32_t{b});
  }

  /// Adds a value in place.
  constexpr Fixed& operator+=(Fixed other) noexcept {
    raw_ += other.raw_;
    return *this;
  }
  /// Subtracts a value in place.
  constexpr Fixed& operator-=(Fixed other) noexcept {
    raw_ -= other.raw_;
    return *this;
  }
  /// Multiplies in place.
  constexpr Fixed& operator*=(Fixed other) noexcept {
    return *this = *this * other;
  }
  /// Divides in place.
  constexpr Fixed& operator/=(Fixed other) noexcept {
    return *this = *this / other;
  }

 private:
  std::int32_t raw_ = 0;
};

/// Returns the square root of a value (libnds' `sqrtf32`, the DS hardware
/// square root unit).
///
/// @param value Value to take the root of; zero or less gives zero.
/// @return The root, rounded down to the 20.12 step.
[[nodiscard]] constexpr Fixed Sqrt(Fixed value) noexcept {
  return Fixed::FromRaw(internal::SqrtF32(value.raw()));
}

/// Returns the absolute value.
[[nodiscard]] constexpr Fixed Abs(Fixed value) noexcept {
  return value < Fixed{} ? -value : value;
}

/// Converts to float, for tests and debug display (soft-float on the DS).
[[nodiscard]] constexpr float ToFloat(Fixed value) noexcept {
  return static_cast<float>(value);
}

/// Converts to an integer, truncating toward zero.
[[nodiscard]] constexpr int ToInt(Fixed value) noexcept {
  return static_cast<int>(value);
}

}  // namespace gpr925

#endif  // GPR925_FIXED_H_
