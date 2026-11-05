#ifndef GPR925PHYSICS_MAIN_SAMPLES_05_TRIGGER_COLLIDER_SHAPES_H_
#define GPR925PHYSICS_MAIN_SAMPLES_05_TRIGGER_COLLIDER_SHAPES_H_

#include <vector>

#include "gpr925/vec2.h"
#include "gpr925/body.h"
#include "gpr925/physics_world.h"
#include "gpr925/collider_utils.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace gpr925 {

class TriggerColliderShapes : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate(const Input& input) override;
  void Draw(DrawList& draw_list) const override;
  void End() override;

 private:
  PhysicsWorld world_;
  // all the planet ids
  std::vector<BodyId> bodies_ids_;
  std::vector<ColliderId> colliders_ids_;
  //since color isn't part of the physics, keep it outside
  std::vector<ColorId> bodies_colors_;
  //how many shapes are overlapping with each other
  std::vector<int> overlap_counts_;
  Rect bounds_{};
};

}  // namespace gpr925

#endif  // GPR925PHYSICS_MAIN_SAMPLES_05_TRIGGER_COLLIDER_SHAPES_H_
