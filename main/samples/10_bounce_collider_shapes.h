#ifndef GPR925PHYSICS_MAIN_SAMPLES_10_BOUNCE__COLLIDER_SHAPES_H_
#define GPR925PHYSICS_MAIN_SAMPLES_10_BOUNCE__COLLIDER_SHAPES_H_

#include <vector>

#include "gpr925/vec2.h"
#include "gpr925/body.h"
#include "gpr925/physics_world.h"
#include "gpr925/collider_utils.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace gpr925 {

class BounceColliderShapes : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate(const Input& input) override;
  void Draw(DrawList& draw_list) const override;
  void End() override;

 private:
  PhysicsWorld world_;
  std::vector<BodyId> bodies_ids_;
  std::vector<ColliderId> colliders_ids_;
  std::vector<ColorId> bodies_colors_;
  Rect bounds_{};
};

}  // namespace gpr925

#endif  // GPR925PHYSICS_MAIN_SAMPLES_10_BOUNCE__COLLIDER_SHAPES_H_
