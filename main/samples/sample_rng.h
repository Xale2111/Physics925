#ifndef MAIN_SAMPLES_SAMPLE_RNG_H_
#define MAIN_SAMPLES_SAMPLE_RNG_H_

#include <cstdint>

#include "gpr925/scalar.h"

#include "main/samples/draw_list.h"

namespace gpr925 {

/// Seeded xorshift32 random generator for the sample scenes.
///
/// Use it in @ref Sample::Begin with a constant seed, one per sample, to
/// scatter objects, speeds and colors. @ref Scalar ranges are built from
/// integer ratios, with no float and no `std::random_device`, so a restart
/// gives the same scene again. Never seed it from the time.
///
/// @code
/// SampleRng rng{kSeed};
/// position = {rng.Range(bounds.min_bound.x, bounds.max_bound.x),
///             rng.Range(bounds.min_bound.y, bounds.max_bound.y)};
/// const ColorId color = rng.RandomColor();
/// @endcode
///
/// @note It is not a good general-purpose generator (@ref Range uses a modulo),
/// just a small, fast and reproducible one.
class SampleRng {
 public:
  /// Creates a generator from a seed.
  ///
  /// @param seed Initial state; 0 (which xorshift can't leave) is replaced by
  /// a fixed non-zero default.
  explicit constexpr SampleRng(std::uint32_t seed) noexcept
      : state_(seed != 0 ? seed : kDefaultSeed) {}

  /// Advances the state and returns it.
  ///
  /// @return A 32-bit pseudo-random value, never 0.
  constexpr std::uint32_t Next() noexcept {
    state_ ^= state_ << 13;
    state_ ^= state_ >> 17;
    state_ ^= state_ << 5;
    return state_;
  }

  /// Returns an integer in [`min_value`, `max_value`].
  ///
  /// @param min_value Lower bound, inclusive.
  /// @param max_value Upper bound, inclusive, not less than `min_value`.
  constexpr int Range(int min_value, int max_value) noexcept {
    const auto span = static_cast<std::uint32_t>(max_value - min_value) + 1u;
    return min_value + static_cast<int>(Next() % span);
  }

  /// Returns a @ref Scalar in [`min_value`, `max_value`].
  ///
  /// The range is split in 4096 steps and computed with @ref Ratio.
  ///
  /// @param min_value Lower bound, inclusive.
  /// @param max_value Upper bound, inclusive.
  constexpr Scalar Range(Scalar min_value, Scalar max_value) noexcept {
    return min_value +
           (max_value - min_value) * Ratio(Range(0, kResolution), kResolution);
  }

  /// Returns one of the non-grey palette colors.
  ///
  /// @return A color between @ref sample_color::kFirstBright and
  /// @ref sample_color::kLastBright.
  constexpr ColorId RandomColor() noexcept {
    return static_cast<ColorId>(
        Range(sample_color::kFirstBright, sample_color::kLastBright));
  }

 private:
  static constexpr std::uint32_t kDefaultSeed = 0x9E3779B9u;
  static constexpr int kResolution = 4096;
  std::uint32_t state_;
};

}  // namespace gpr925

#endif  // MAIN_SAMPLES_SAMPLE_RNG_H_
