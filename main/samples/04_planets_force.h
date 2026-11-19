//
// Created by alexk on 21.10.2026.
//

#ifndef GPR925PHYSICS_MAIN_SAMPLES_04_PLANETS_FORCE_H_
#define GPR925PHYSICS_MAIN_SAMPLES_04_PLANETS_FORCE_H_
#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

#include "vector2.h"
#include "number_concepts.h"
#include "gpr925/physics_world.h"

namespace gpr925 {

class PlanetForceSample : public Sample {
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
  PhysicsWorld world_;
  BodyId center_id_{};
  // all the planet ids
  std::vector<BodyId> planet_ids_;
  //since color isn't part of the physics, keep it outside
  std::vector<ColorId> planet_colors_;
  Rect bounds_{};

};
}  // namespace gpr925

#endif  // GPR925PHYSICS_MAIN_SAMPLES_04_PLANETS_FORCE_H_
