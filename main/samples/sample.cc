#include "main/samples/sample.h"

#include "main/samples/01_hello.h"
#include "main/samples/02_planets.h"
#include "main/samples/03_planets_aosoa.h"
#include "main/samples/04_planets_force.h"
#include "main/samples/05_trigger_collider.h"
#include "main/samples/06_trigger_collider_box.h"
#include "main/samples/07_trigger_collider_shapes.h"
#include "main/samples/08_bounce_collider_circles.h"
#include "main/samples/09_bounce_collider_boxes.h"
#include "main/samples/10_bounce_collider_shapes.h"
#include "main/samples/11_ground_bounce.h"

namespace gpr925 {

SampleRegistry::SampleRegistry() {
  // Register your samples here: bump the reserve and add a push_back
  samples_.reserve(11);
  samples_.push_back({"01 Hello", std::make_unique<HelloSample>()});
  samples_.push_back({"02 Planets", std::make_unique<PlanetSample>()});
  samples_.push_back({"03 Planets AoSoA", std::make_unique<PlanetAoSoaSample>()});
  samples_.push_back({"04 Planets Force", std::make_unique<PlanetForceSample>()});
  samples_.push_back({"05 Circle Trigger", std::make_unique<TriggerCollider>()});
  samples_.push_back({"06 Box Trigger", std::make_unique<TriggerColliderBox>()});
  samples_.push_back({"07 All Shapes Trigger", std::make_unique<TriggerColliderShapes>()});
  samples_.push_back({"08 Bounce Collider Circles", std::make_unique<BounceColliderCircles>()});
  samples_.push_back({"09 Bounce Collider Boxes", std::make_unique<BounceColliderBoxes>()});
  samples_.push_back({"10 Bounce Collider Shapes", std::make_unique<BounceColliderShapes>()});
  samples_.push_back({"11 Ground Bounce", std::make_unique<GroundBounce>()});

}

SampleRegistry::~SampleRegistry() { Select(kNone, context_); }

void SampleRegistry::Select(std::size_t index, const SampleContext& context) {
  if (current_index_ != kNone) {
    samples_[current_index_].sample->End();
  }
  current_index_ = index < samples_.size() ? index : kNone;
  context_ = context;
  if (current_index_ != kNone) {
    samples_[current_index_].sample->Begin(context_);
  }
}

void SampleRegistry::FixedUpdate(const Input& input) {
  if (current_index_ != kNone) {
    samples_[current_index_].sample->FixedUpdate(input);
  }
}

void SampleRegistry::Draw(DrawList& draw_list) const {
  draw_list.Clear();
  draw_list.camera = context_.world_bounds;
  if (current_index_ != kNone) {
    samples_[current_index_].sample->Draw(draw_list);
  }
}

}  // namespace gpr925
