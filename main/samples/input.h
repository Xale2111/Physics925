#ifndef MAIN_SAMPLES_INPUT_H_
#define MAIN_SAMPLES_INPUT_H_

#include <cstdint>

namespace gpr925 {

/// Button of the virtual pad that every host provides.
///
/// | Button | DS         | PC keyboard | PC gamepad |
/// |--------|------------|-------------|------------|
/// | D-pad  | D-pad      | arrow keys  | D-pad      |
/// | `kA`   | A (right)  | F           | east       |
/// | `kB`   | B (bottom) | D           | south      |
/// | `kX`   | X (top)    | E           | north      |
/// | `kY`   | Y (left)   | S           | west       |
///
/// On PC the four action buttons keep the diamond of the console: E above
/// S, D, F on the keyboard, and the same positions on a gamepad whatever
/// letter it prints (an Xbox pad prints A on the south button, which is B
/// here). E, S, D and F are at the same place with the same label on US,
/// Swiss and French keyboards, unlike W, A, Z, Q or Y.
///
/// L, R and START on the DS belong to the sample menu, not to the samples.
/// The eight buttons use the eight bits of the mask: one more needs a wider
/// type in @ref Input.
enum class Button : std::uint8_t {
  kUp = 1 << 0,     ///< D-pad up.
  kDown = 1 << 1,   ///< D-pad down.
  kLeft = 1 << 2,   ///< D-pad left.
  kRight = 1 << 3,  ///< D-pad right.
  kA = 1 << 4,      ///< Main action button (right of the diamond).
  kB = 1 << 5,      ///< Second action button (bottom of the diamond).
  kX = 1 << 6,      ///< Third action button (top of the diamond).
  kY = 1 << 7,      ///< Fourth action button (left of the diamond).
};

/// Returns the bit of a button in @ref Input::held and @ref Input::pressed.
[[nodiscard]] constexpr std::uint8_t ButtonMask(Button button) noexcept {
  return static_cast<std::uint8_t>(button);
}

/// State of the virtual pad for one @ref Sample::FixedUpdate step.
///
/// The host fills it and passes it to the step, so a sample never talks to
/// SDL or libnds:
/// @code
/// void MySample::FixedUpdate(const Input& input) {
///   if (input.IsHeld(Button::kRight)) { /* move while the key is down */ }
///   if (input.WasPressed(Button::kA)) { /* once per key press */ }
/// }
/// @endcode
struct Input {
  std::uint8_t held = 0;  ///< Buttons that are down, as @ref ButtonMask bits.
  std::uint8_t pressed = 0;  ///< Buttons that went down on this step.

  /// Builds the input of a step from the buttons down now and on the
  /// previous step.
  ///
  /// @param held Buttons down now.
  /// @param previous_held Buttons that were down on the previous step.
  [[nodiscard]] static constexpr Input FromHeld(
      std::uint8_t held, std::uint8_t previous_held) noexcept {
    return {held, static_cast<std::uint8_t>(held & ~previous_held)};
  }

  /// Returns true while the button is down.
  [[nodiscard]] constexpr bool IsHeld(Button button) const noexcept {
    return (held & ButtonMask(button)) != 0;
  }
  /// Returns true on the one step where the button goes down.
  [[nodiscard]] constexpr bool WasPressed(Button button) const noexcept {
    return (pressed & ButtonMask(button)) != 0;
  }
};

}  // namespace gpr925

#endif  // MAIN_SAMPLES_INPUT_H_
