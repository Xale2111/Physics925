#ifndef MAIN_SAMPLES_SAMPLE_H_
#define MAIN_SAMPLES_SAMPLE_H_

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "gpr925/vec2.h"

#include "main/samples/draw_list.h"
#include "main/samples/input.h"

namespace gpr925 {

/// What the host platform gives a sample when it begins.
///
/// The host passes it to @ref SampleRegistry::Select, which forwards it to
/// @ref Sample::Begin. Both hosts use @ref sample_config::kWorldBounds.
struct SampleContext {
  /// World rectangle shown on screen, in meters with y pointing down.
  ///
  /// Samples place their objects inside it, so the same scene fits the PC
  /// window and the DS top screen.
  Rect world_bounds{};
};

/// Scene shown by a host: the base class of every sample.
///
/// A sample knows nothing about the platform (no SDL, ImGui or libnds): it
/// simulates in @ref FixedUpdate and describes itself in a @ref DrawList. The
/// same code runs in the SDL window on PC (`main/sdl/main.cc`), in the DS ROM
/// (`nds/main.cc`) and in the unit tests (`tests/samples_test.cc`).
///
/// Lifecycle, driven by @ref SampleRegistry:
///
/// - @ref Begin builds the scene when the sample is selected or restarted.
/// - @ref FixedUpdate advances it by exactly @ref kFixedDeltaTime. The PC host
///   calls it zero or more times per frame from an accumulator; the DS calls it
///   once per VBlank (60 Hz).
/// - @ref Draw fills the draw list once per rendered frame. It is `const`:
///   never change the simulation there, or the result would depend on the
///   frame rate.
/// - @ref End tears the scene down so that the next @ref Begin starts clean.
///
/// Rules that keep a sample portable:
///
/// - Use @ref Scalar and @ref Vec2, never `float` directly: on the DS,
///   @ref Scalar is the fixed-point @ref Fixed. Write constants with @ref Ratio
///   or as `constexpr Scalar` values.
/// - Read sizes and object counts from @ref sample_config, never hard-code a
///   screen size.
/// - Simulate only in @ref FixedUpdate, never with the frame time.
/// - Allocate in @ref Begin, not per step.
///
/// How to add a new sample:
///
/// 1. Copy `main/samples/01_hello.h` and `.cc` to `02_my_sample.h` and `.cc`
///    and rename the class. Every file of `main/samples/` is compiled into
///    the `Gpr925Samples` library, so the PC host and the DS ROM both get it.
/// 2. Put its per-platform sizes in @ref sample_config and check that the
///    @ref DrawList capacities (@ref sample_config::kMaxDrawCircles...) fit.
/// 3. Register it in the @ref SampleRegistry constructor in
///    `main/samples/sample.cc`, with the name shown in the menus. Both hosts
///    build their menus from the registry: there is nothing to change in
///    `main/sdl/` or `nds/`.
/// 4. Add it to `tests/samples_test.cc`.
///
/// @see @ref SampleRegistry
/// @see @ref DrawList
class Sample {
 public:
  /// Destroys the sample.
  virtual ~Sample() = default;
  /// Builds the scene.
  ///
  /// Called each time the sample is selected or restarted, always after
  /// @ref End of the previous run, so it must produce the same initial state
  /// every time.
  ///
  /// @param context What the host gives the sample, such as the world bounds.
  virtual void Begin(const SampleContext& context) = 0;
  /// Advances the simulation by one @ref kFixedDeltaTime step.
  ///
  /// All simulation and game logic goes here. Never use wall-clock time.
  ///
  /// @param input State of the virtual pad for this step.
  virtual void FixedUpdate(const Input& input) = 0;
  /// Describes the current state of the scene for the host to render.
  ///
  /// The registry has already cleared @p draw_list and set its camera; the
  /// sample only appends circles, boxes and lines in world space.
  ///
  /// @param draw_list The list to append the frame's shapes to.
  virtual void Draw(DrawList& draw_list) const = 0;
  /// Tears the scene down.
  virtual void End() = 0;

  /// Duration of one @ref FixedUpdate step: 1/60 s, one DS frame.
  ///
  /// With @ref Fixed it is 68/4096 s, 0.4 % shorter than 1/60 s.
  static constexpr Scalar kFixedDeltaTime = Ratio(1, 60);
};

/// Owns every sample and runs the selected one.
///
/// The list of samples lives in the constructor (`main/samples/sample.cc`):
/// that is the only place where a new sample is registered. Each host builds
/// its own menu on top of it (ImGui combo box on PC, L/R on the DS) and
/// forwards its fixed steps and frames to it:
/// @code
/// SampleRegistry registry;
/// const SampleContext context{sample_config::kWorldBounds};
/// registry.Select(0, context);
/// DrawList draw_list;
/// draw_list.Reserve(sample_config::kMaxDrawCircles,
///                   sample_config::kMaxDrawBoxes,
///                   sample_config::kMaxDrawLines);
/// while (running) {
///   // as many times as kFixedDeltaTime fits in the elapsed time
///   registry.FixedUpdate(input);
///   registry.Draw(draw_list);
///   drawer.Render(draw_list);
/// }
/// @endcode
class SampleRegistry {
 public:
  /// Index meaning "no sample selected".
  static constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();

  /// Creates every sample, with none selected.
  SampleRegistry();
  /// Not copyable: it owns the samples.
  SampleRegistry(const SampleRegistry&) = delete;
  /// Not copyable: it owns the samples.
  SampleRegistry& operator=(const SampleRegistry&) = delete;
  /// Ends the current sample, then destroys every sample.
  ~SampleRegistry();

  /// Returns the number of registered samples.
  [[nodiscard]] std::size_t size() const { return samples_.size(); }
  /// Returns the menu name of a sample, such as `"01 Hello"`.
  ///
  /// @param index Sample index, less than @ref size.
  /// @return A string literal, valid for the registry's lifetime.
  [[nodiscard]] const char* name(std::size_t index) const {
    return samples_[index].name;
  }
  /// Returns the index of the selected sample, or @ref kNone.
  [[nodiscard]] std::size_t current_index() const { return current_index_; }
  /// Returns the selected sample, or `nullptr` if none is selected.
  [[nodiscard]] Sample* current() const {
    return current_index_ == kNone ? nullptr
                                   : samples_[current_index_].sample.get();
  }

  /// Ends the current sample and begins another one.
  ///
  /// Selecting the current index restarts it from its initial state.
  ///
  /// @param index Sample to begin; @ref kNone or any index not less than
  /// @ref size selects no sample.
  /// @param context Passed to @ref Sample::Begin and kept for @ref Draw.
  void Select(std::size_t index, const SampleContext& context);
  /// Advances the selected sample by one fixed step, if any.
  ///
  /// @param input State of the virtual pad for this step.
  void FixedUpdate(const Input& input);
  /// Fills a draw list with the selected sample's frame.
  ///
  /// Clears the list, sets its camera to the context's world bounds, then
  /// lets the selected sample (if any) append its shapes.
  ///
  /// @param draw_list The list to fill; its capacity is kept.
  void Draw(DrawList& draw_list) const;

 private:
  struct Entry {
    const char* name;
    std::unique_ptr<Sample> sample;
  };
  std::vector<Entry> samples_;
  std::size_t current_index_ = kNone;
  SampleContext context_{};
};

}  // namespace gpr925

#endif  // MAIN_SAMPLES_SAMPLE_H_
