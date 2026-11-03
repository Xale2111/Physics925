//
// Created by alexk on 27.10.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_WORLD_H_
#define GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_WORLD_H_
#include <cstddef>
#include <span>
#include <vector>

#include "scalar.h"
#include "vec2.h"
#include "body_utils.h"
#include "body_def.h"
#include "body.h"
#include "collider_pair.h"
#include "trigger_event.h"

namespace gpr925 {

class PhysicsWorld {
 public:
  PhysicsWorld() = default;

  //gravitational_constant: the G of F = G*m1*m2/r^2
  //min_distance: below this distance the attraction stops growing
  PhysicsWorld(Scalar gravitational_constant, Scalar min_distance, Rect bounds, size_t size_to_reserve = 0);

  BodyId AddBody(const BodyDef& def);
  //void RemoveBody(BodyId id);

  void Clear() { bodies_.clear(); }
  void ReserveBodyVectorSize(size_t size_to_reserve) {
    bodies_.reserve(size_to_reserve);
    previous_overlaps_.clear();
    events_.clear();

  }
  [[nodiscard]] Body& GetBody(BodyId id) { return bodies_[static_cast<size_t>(id)]; }
  [[nodiscard]] const Body& GetBody(BodyId id) const { return bodies_[static_cast<size_t>(id)]; }
  [[nodiscard]] std::size_t CountBody() const { return bodies_.size(); }

  [[nodiscard]] std::span<const TriggerEvent> TriggerEvents() const {
    return events_;
  }


  // One fixed step: gather the forces, then integrate.
  void Step(Scalar dt);
  //Step but with no gravity. Balls with bounce on the "walls" to stay in bounds (like the 01_hello.cc)
  void StepFreeRoam(Scalar dt);

 private:
  void ApplyGravity();
  void Integrate(Scalar dt);
  //Checks the pairs that start or stop overlapping
  void UpdateTriggers();

  std::vector<Body> bodies_;
  Scalar gravitational_constant_ = {};
  Scalar min_distance_sqr_ = {};
  Rect bounds_;

  //last step overlapping bodies
  std::vector<ColliderPair> previous_overlaps_;
  std::vector<TriggerEvent> events_;

};

}  // namespace gpr925

#endif  // GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_WORLD_H_
