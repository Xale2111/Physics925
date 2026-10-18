//
// Created by alexk on 14.10.2026.
//

#ifndef GPR925PHYSICS_MAIN_SAMPLES_02_PLANETS_H_
#define GPR925PHYSICS_MAIN_SAMPLES_02_PLANETS_H_

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

#include "vector2.h"
#include "number_concepts.h"

namespace gpr925 {

class PlanetSample : public Sample {
 public :
  /// Scatters the balls from a fixed seed and centers the player.
  void Begin(const SampleContext& context) override;
  /// Moves the balls and the player by one step.
  void FixedUpdate(const Input& input) override;
  /// Draws the balls, the player, the box and the frame.
  void Draw(DrawList& draw_list) const override;
  /// Removes the balls.
  void End() override;


 private:

  struct Ball {
    maths::Vector2<Scalar> position{};
    maths::Vector2<Scalar>  velocity{};
    ColorId color = sample_color::kWhite;
  };

  std::vector<Ball> balls_;
  Rect bounds_{};
  maths::Vector2<Scalar>  center_{};
  Ball center_ball_;
};
}

#endif  // GPR925PHYSICS_MAIN_SAMPLES_02_PLANETS_H_
