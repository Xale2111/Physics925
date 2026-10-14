#ifndef MAIN_SAMPLES_01_HELLO_H_
#define MAIN_SAMPLES_01_HELLO_H_

#include <vector>

#include "gpr925/vec2.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace gpr925 {

/// Sample 01: the three shapes of a @ref DrawList and the virtual pad.
///
/// It uses no physics engine, only @ref Scalar and @ref Vec2: copy it to start
/// your own sample.
///
/// - Balls move in straight lines at a random velocity and bounce on the
///   world bounds (circles). There are @ref sample_config::kHelloBallCount of
///   them: fewer on the DS than on PC.
/// - The D-pad moves the player, a bigger circle.
/// - A changes the player's color (@ref Input::WasPressed: once per press).
/// - B freezes the balls while it is down (@ref Input::IsHeld).
/// - A box sits in the top-left corner and four lines frame the world.
class HelloSample : public Sample {
 public:
  /// Scatters the balls from a fixed seed and centers the player.
  void Begin(const SampleContext& context) override;
  /// Moves the balls and the player by one step.
  void FixedUpdate(const Input& input) override;
  /// Draws the balls, the player, the box and the frame.
  void Draw(DrawList& draw_list) const override;
  /// Removes the balls.
  void End() override;

  /// Returns the player's position, for the tests.
  [[nodiscard]] Vec2 player_position() const { return player_position_; }

 private:
  struct Ball {
    Vec2 position{};
    Vec2 velocity{};
    ColorId color = sample_color::kWhite;
  };

  std::vector<Ball> balls_;
  Rect bounds_{};
  Vec2 player_position_{};
  ColorId player_color_ = sample_color::kWhite;
};

}  // namespace gpr925

#endif  // MAIN_SAMPLES_01_HELLO_H_
