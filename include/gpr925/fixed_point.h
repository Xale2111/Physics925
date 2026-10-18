//
// Created by alexk on 15.10.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_FIXED_POINT_H_
#define GPR925PHYSICS_INCLUDE_GPR925_FIXED_POINT_H_

#include "number_concepts.h"
#include "gpr925/fixed.h"

template <>
struct maths::NumberTraits<gpr925::Fixed> {
  // * and / round, as with float: compare with an epsilon.
  static constexpr bool kIsExact = false;
  // Two steps of 1/4096: 1e-5 is below what 20.12 can hold.
  static constexpr gpr925::Fixed kEpsilon = gpr925::Fixed::FromRaw(2);
  static gpr925::Fixed Sqrt(gpr925::Fixed value) noexcept {
    return gpr925::Sqrt(value);
  }
  static gpr925::Fixed Abs(gpr925::Fixed value) noexcept {
    return gpr925::Abs(value);
  }
};
#endif  // GPR925PHYSICS_INCLUDE_GPR925_FIXED_POINT_H_
