//
// Created by alexk on 03.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_PAIR_H_
#define GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_PAIR_H_
#include <compare>

#include "body_utils.h"

namespace gpr925 {

//(A,B) and (B,A) are the same
class ColliderPair {
 public:
  constexpr ColliderPair(BodyId a, BodyId b)
      : first_(b < a ? b : a), second_(b < a ? a : b) {}

  [[nodiscard]] constexpr BodyId first() const { return first_; }
  [[nodiscard]] constexpr BodyId second() const { return second_; }

  friend constexpr auto operator<=>(const ColliderPair&,
                                    const ColliderPair&) = default;

 private:
  //First is the one with the smaller id (and so, second is the one with the higher id)
  BodyId first_;
  BodyId second_;
};

}  // namespace gpr925


#endif  // GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_PAIR_H_
