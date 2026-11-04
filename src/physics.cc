// Your physics engine starts here: declare it in include/gpr925/ and implement
// it in src/. Every .cc file of this folder is compiled into the Gpr925Physics
// library, for PC and for the DS.

#include <algorithm>
#include <cassert>
#include <iterator>

#include "gpr925/collision.h"
#include "gpr925/physics_world.h"

namespace gpr925 {

PhysicsWorld::PhysicsWorld(Scalar gravitational_constant, Scalar min_distance,
                           Rect bounds, size_t size_to_reserve) {
  gravitational_constant_ = gravitational_constant;
  min_distance_sqr_ = min_distance * min_distance;
  bounds_ = bounds;
  bodies_.reserve(size_to_reserve);
  // ApplyGravity divides by the squared distance: with gravity on, the minimum
  // distance must be big enough for its square to be more than zero (with a
  // fixed point scalar, 0.01 squared is already rounded down to 0).
  assert((gravitational_constant == Scalar{} || min_distance_sqr_ > Scalar{}) &&
         "with gravity on, min_distance must be strictly positive");
}

// the mass_ of the body must be strictly positive
// return expected bodyID, (imply enum class for error)
BodyId PhysicsWorld::AddBody(const BodyDef& def) {
  // assert(def.mass > Scalar{} && "a body needs a strictly positive mass_");
  // //<- do not use
  Body body(def);
  bodies_.push_back(body);
  return static_cast<BodyId>(bodies_.size() - 1);
}
/*
void PhysicsWorld::RemoveBody(BodyId id) {
  //Implement logic to remove a body by setting his mass_ to the next valid one
  auto temp = id;
}*/

void PhysicsWorld::Step(Scalar dt, bool stay_in_bounds) {
  events_.clear();
  ApplyGravity();  // sum the forces
  Integrate(dt);   // velocity_, then position_
  if (stay_in_bounds) {
    BounceInBounds();  // looks at the new position_, reverses velocity_
  }
  UpdateTriggers();
}

void PhysicsWorld::ApplyGravity() {
  if (gravitational_constant_ == Scalar{}) {
    return;  // no gravity: no force to sum, and no O(n^2) loop for nothing
  }
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    for (std::size_t j = i + 1; j < bodies_.size(); ++j) {
      Body& a = bodies_[i];
      Body& b = bodies_[j];
      if (a.GetBodyType() == BodyType::kStatic &&
          b.GetBodyType() == BodyType::kStatic) {
        continue;  // neither can move: the force_ would be discarded
      }

      const Vec2 delta = b.GetPosition() - a.GetPosition();  // from a to b
      Scalar sqr_distance = delta.SqrMagnitude();
      // check distance between bodies. If it is too small, set it to the
      // minimum distance
      if (sqr_distance < min_distance_sqr_) {
        sqr_distance = min_distance_sqr_;
      }

      const Scalar distance = maths::NumberTraits<Scalar>::Sqrt(sqr_distance);

      // Gravitation law -> F = (G * m1 * m2) / r^2
      const Scalar magnitude =
          gravitational_constant_ * a.GetMass() * b.GetMass() / sqr_distance;
      const Vec2 force_on_a = delta * (magnitude / distance);

      // third law: equal and opposite
      a.ApplyForce(force_on_a);
      b.ApplyForce(-force_on_a);
    }
  }
}

void PhysicsWorld::Integrate(Scalar dt) {
  for (Body& body : bodies_) {
    if (body.GetBodyType() == BodyType::kStatic) {
      continue;
    }
    // Second law: a = F / m
    const Vec2 acceleration{body.GetForce().x() / body.GetMass(),
                            body.GetForce().y() / body.GetMass()};
    // semi implicit Euler integration
    body.SetVelocity(body.GetVelocity() +
                     acceleration * dt);  // velocity_ first
    body.SetPosition(body.GetPosition() +
                     body.GetVelocity() * dt);  // then position_, new velocity_

    body.SetForce(Vec2{});  // reset forces
  }
}

void PhysicsWorld::UpdateTriggers() {
  // 1. A collider knows its bodies: walk the colliders and gather one entry
  //    per (body, collider) link.
  entries_.clear();
  for (const auto& collider : colliders_) {
    for (const BodyId body_id : collider->GetBodies()) {
      assert(static_cast<std::size_t>(body_id) < bodies_.size() &&
             "a collider is linked to a body that does not exist");
      entries_.push_back({body_id, collider.get()});
    }
  }

  // 2. The pairs overlapping now.
  std::vector<ColliderPair> current;
  const std::size_t count = entries_.size();
  for (std::size_t i = 0; i < count; ++i) {
    const ColliderEntry& entry_a = entries_[i];
    const Collider& collider_a = *entry_a.collider;
    // Read once per i: Overlap and emplace_back are calls the compiler cannot
    // see into, it would reload them at every j.
    const bool a_is_trigger = collider_a.IsTrigger();
    const Vec2 position_a =
        bodies_[static_cast<std::size_t>(entry_a.body_id)].GetPosition();
    for (std::size_t j = i + 1; j < count; ++j) {
      const ColliderEntry& entry_b = entries_[j];
      if (entry_a.body_id == entry_b.body_id) {
        continue;  // the same body linked twice: not a contact
      }
      // Like Unity: events only if at least one of the two is a trigger.
      if (!a_is_trigger && !entry_b.collider->IsTrigger()) {
        continue;
      }
      const Vec2& position_b =
          bodies_[static_cast<std::size_t>(entry_b.body_id)].GetPosition();
      if (Overlap(collider_a, position_a, *entry_b.collider, position_b)) {
        current.emplace_back(entry_a.body_id, entry_b.body_id);
      }
    }
  }
  // The entries follow the order of the colliders, not the order of the ids:
  // `current` is not sorted by construction anymore. Sort it (it only holds the
  // overlapping pairs, a short list), and drop a pair found through two
  // different colliders.
  std::ranges::sort(current);
  const auto duplicates = std::ranges::unique(current);
  current.erase(duplicates.begin(), duplicates.end());

  // 3. What changed since the previous step. Both lists are sorted, so one
  //    pass over each finds every change.
  std::vector<ColliderPair> entered;
  std::vector<ColliderPair> exited;
  std::ranges::set_difference(current, previous_overlaps_,
                              std::back_inserter(entered));
  std::ranges::set_difference(previous_overlaps_, current,
                              std::back_inserter(exited));

  for (const ColliderPair& pair : entered) {
    events_.push_back({TriggerEventType::kEnter, pair.first(), pair.second()});
  }
  for (const ColliderPair& pair : exited) {
    events_.push_back({TriggerEventType::kExit, pair.first(), pair.second()});
  }

  previous_overlaps_ = std::move(current);
}
void PhysicsWorld::BounceInBounds() {
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    if ((bodies_[i].GetVelocity().x() > Scalar{} &&
         bodies_[i].GetPosition().x() > bounds_.max_bound.x()) ||
        (bodies_[i].GetVelocity().x() < Scalar{} &&
         bodies_[i].GetPosition().x() < bounds_.min_bound.x())) {
      bodies_[i].SetVelocity(
          Vec2(-bodies_[i].GetVelocity().x(), bodies_[i].GetVelocity().y()));
    }
    if ((bodies_[i].GetVelocity().y() > Scalar{} &&
         bodies_[i].GetPosition().y() > bounds_.max_bound.y()) ||
        (bodies_[i].GetVelocity().y() < Scalar{} &&
         bodies_[i].GetPosition().y() < bounds_.min_bound.y())) {
      bodies_[i].SetVelocity(
          Vec2(bodies_[i].GetVelocity().x(), -bodies_[i].GetVelocity().y()));
    }
  }
}

}  // namespace gpr925