#include "main/samples/sample.h"

#include "main/samples/01_hello.h"
#include "main/samples/02_planets.h"
#include "main/samples/03_planets_aosoa.h"
#include "main/samples/04_planets_force.h"

namespace gpr925 {

SampleRegistry::SampleRegistry() {
  // Register your samples here: bump the reserve and add a push_back
  samples_.reserve(4);
  samples_.push_back({"01 Hello", std::make_unique<HelloSample>()});
  samples_.push_back({"02 Planets", std::make_unique<PlanetSample>()});
  samples_.push_back({"03 Planets AoSoA", std::make_unique<PlanetAoSoaSample>()});
  samples_.push_back({"04 Planets Force", std::make_unique<PlanetForceSample>()});
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
