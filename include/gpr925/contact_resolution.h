//
// Created by alexk on 16.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_CONTACT_RESOLUTION_H_
#define GPR925PHYSICS_INCLUDE_GPR925_CONTACT_RESOLUTION_H_
#include "gpr925/body.h"
#include "gpr925/collision.h"
#include "scalar.h"

namespace gpr925 {
  void ResolveContact(Body& a, Body& b, const Contact& contact, Scalar restitution, Scalar friction);

}
#endif  // GPR925PHYSICS_INCLUDE_GPR925_CONTACT_RESOLUTION_H_
