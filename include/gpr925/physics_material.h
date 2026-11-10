//
// Created by alexk on 10.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_MATERIAL_H_
#define GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_MATERIAL_H_
#include <algorithm>
#include "gpr925/scalar.h"

namespace gpr925 {

// Surface properties of a collider, like Unity's PhysicsMaterial.
// It is a small plain value (two scalars), so we copy it instead of sharing
// pointers: no ownership, no lifetime issue, nothing to allocate on the DS.
struct PhysicsMaterial {
  Scalar bounciness{};  // restitution coefficient e, kept in [0, 1]
  Scalar friction{};    // stored for later, not used by the solver yet

  constexpr PhysicsMaterial() = default;
  constexpr PhysicsMaterial(Scalar bounciness_in, Scalar friction_in = Scalar{})
      : bounciness(std::clamp(bounciness_in, Scalar{}, Scalar{1})),
        friction(friction_in < Scalar{} ? Scalar{} : friction_in) {}
};

}  // namespace gpr925
#endif //GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_MATERIAL_H_
