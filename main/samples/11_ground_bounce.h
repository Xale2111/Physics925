#ifndef GPR925PHYSICS_MAIN_SAMPLES_11__GROUND__BOUNCE_H_
#define GPR925PHYSICS_MAIN_SAMPLES_11__GROUND__BOUNCE_H_

#include <vector>

#include "gpr925/vec2.h"
#include "gpr925/body.h"
#include "gpr925/physics_world.h"
#include "gpr925/collider_utils.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace gpr925 {

class GroundBounce : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate(const Input& input) override;
  void Draw(DrawList& draw_list) const override;
  void End() override;

 private:
  PhysicsWorld world_;
  BodyId ball_id_;
  BodyId box_id_;
  BodyId ground_id_;
  std::vector<ColliderId> colliders_ids_;
  //since color isn't part of the physics, keep it outside
  std::vector<ColorId> bodies_colors_;
  Rect bounds_{};
};

}  // namespace gpr925

#endif  // GPR925PHYSICS_MAIN_SAMPLES_11__GROUND__BOUNCE_H_
