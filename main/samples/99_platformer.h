#ifndef GPR925PHYSICS_MAIN_SAMPLES_99_PLATFORMER_H_
#define GPR925PHYSICS_MAIN_SAMPLES_99_PLATFORMER_H_

#include <vector>

#include "gpr925/vec2.h"
#include "gpr925/body.h"
#include "gpr925/physics_world.h"
#include "gpr925/collider_utils.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace gpr925 {

enum class Tag { kDefault, kPlayer, kGround, kGroundCheck };

struct GameObject{
  BodyId body_id;
  Tag tag;
  ColorId color;
};

class Platformer : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate(const Input& input) override;
  void Draw(DrawList& draw_list) const override;
  void End() override;

 private:
  void UpdateGrounded();               // reads the events of the last Step
  Tag TagOf(BodyId id) const;

  PhysicsWorld world_;
  std::vector<GameObject> game_objects_;
  BodyId player_id;
  BodyId player_groundcheck_id;
  BodyId ground_id_;



  bool grounded_ = false;
  bool up_before_ = false;
  int ground_contacts_ = 0;            // replaces overlap_counts_

  std::vector<ColliderId> colliders_ids_;
  //since color isn't part of the physics, keep it outside
  std::vector<ColorId> bodies_colors_;
  Rect bounds_{};
};

}  // namespace gpr925

#endif  // GPR925PHYSICS_MAIN_SAMPLES_99_PLATFORMER_H_
