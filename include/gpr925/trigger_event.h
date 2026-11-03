//
// Created by alexk on 03.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_TRIGGER_EVENT_H_
#define GPR925PHYSICS_INCLUDE_GPR925_TRIGGER_EVENT_H_
#include "body_utils.h"

namespace gpr925 {

//Slide 28 du cours collisions_02 (gestion des triggers)
enum class TriggerEventType {
  kEnter,  //Now overlapping, wasn't overlapping last frame/step
  kExit,   //Now not overlapping, was overlapping last frame/step
};

struct TriggerEvent {
  TriggerEventType type;
  //Same then collider pair. a is the body with the smaller Id and b the one with the higher id
  BodyId a;
  BodyId b;

  friend constexpr bool operator==(const TriggerEvent&,
                                   const TriggerEvent&) = default;
};

}  // namespace gpr925


#endif  // GPR925PHYSICS_INCLUDE_GPR925_TRIGGER_EVENT_H_
