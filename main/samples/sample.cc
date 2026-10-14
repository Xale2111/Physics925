#include "main/samples/sample.h"

#include "main/samples/01_hello.h"

namespace gpr925 {

SampleRegistry::SampleRegistry() {
  // Register your samples here: bump the reserve and add a push_back
  samples_.reserve(1);
  samples_.push_back({"01 Hello", std::make_unique<HelloSample>()});
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
