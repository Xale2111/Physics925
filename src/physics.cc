// Your physics engine starts here: declare it in include/gpr925/ and implement
// it in src/. Every .cc file of this folder is compiled into the Gpr925Physics
// library, for PC and for the DS.

#include <algorithm>
#include <iterator>

#include "gpr925/collision.h"
#include "gpr925/physics_world.h"

namespace gpr925 {

PhysicsWorld::PhysicsWorld(Scalar gravitational_constant, Scalar min_distance,Rect bounds,
                           size_t size_to_reserve) {
  gravitational_constant_ = gravitational_constant;
  min_distance_sqr_ = min_distance * min_distance;
  bounds_ = bounds;
  bodies_.reserve(size_to_reserve);
}

// the mass_ of the body must be strictly positive
// return expected bodyID, (imply enum class for error)
BodyId PhysicsWorld::AddBody(const BodyDef& def) {
  //assert(def.mass > Scalar{} && "a body needs a strictly positive mass_");  //<- do not use
  Body body(def);
  bodies_.push_back(body);
  return static_cast<BodyId>(bodies_.size() - 1);
}
/*
void PhysicsWorld::RemoveBody(BodyId id) {
  //Implement logic to remove a body by setting his mass_ to the next valid one
  auto temp = id;
}*/

void PhysicsWorld::Step(Scalar dt) {
  events_.clear();
  ApplyGravity();  // sum the forces
  Integrate(dt);   // velocity_, then position_
  UpdateTriggers();
}

void PhysicsWorld::StepFreeRoam(Scalar dt) {
  events_.clear();
  //Make the bodies stay in bounds
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    bodies_[i].SetPosition(bodies_[i].GetPosition() + bodies_[i].GetVelocity() * dt);

    if ((bodies_[i].GetVelocity().x() > Scalar{} && bodies_[i].GetPosition().x() > bounds_.max_bound.x()) ||
        (bodies_[i].GetVelocity().x() < Scalar{} && bodies_[i].GetPosition().x() < bounds_.min_bound.x())) {
      bodies_[i].SetVelocity(Vec2(-bodies_[i].GetVelocity().x(), bodies_[i].GetVelocity().y()));
    }
    if ((bodies_[i].GetVelocity().y() > Scalar{} && bodies_[i].GetPosition().y() > bounds_.max_bound.y()) ||
        (bodies_[i].GetVelocity().y() < Scalar{} && bodies_[i].GetPosition().y() < bounds_.min_bound.y())) {
      bodies_[i].SetVelocity(Vec2(bodies_[i].GetVelocity().x(), -bodies_[i].GetVelocity().y()));
    }
  }
  UpdateTriggers();
}


void PhysicsWorld::ApplyGravity() {
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
    body.SetVelocity(body.GetVelocity() + acceleration * dt);  // velocity_ first
    body.SetPosition(body.GetPosition() +
                     body.GetVelocity() * dt);  // then position_, new velocity_

    body.SetForce(Vec2{});  // reset forces
  }
}

void PhysicsWorld::UpdateTriggers() {
  // 1. The pairs overlapping now. The loops go through (i, j) with i < j in
  //    order, so `current` is already sorted: no sort needed.
  std::vector<ColliderPair> current;
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    const auto& collider_a = bodies_[i].GetCollider();
    if (!collider_a) {
      continue;  // no collider: ignored
    }
    for (std::size_t j = i + 1; j < bodies_.size(); ++j) {
      const auto& collider_b = bodies_[j].GetCollider();
      if (!collider_b) {
        continue;
      }
      // Like Unity: events only if at least one of the two is a trigger.
      if (!collider_a->is_trigger && !collider_b->is_trigger) {
        continue;
      }
      if (Overlap(*collider_a, bodies_[i].GetPosition(), *collider_b,
                  bodies_[j].GetPosition())) {
        current.emplace_back(static_cast<BodyId>(i), static_cast<BodyId>(j));
      }
    }
  }

  // 2. What changed since the previous step. Both lists are sorted, so one
  //    pass over each finds every change.
  std::vector<ColliderPair> entered;
  std::vector<ColliderPair> exited;
  std::ranges::set_difference(current, previous_overlaps_,
                              std::back_inserter(entered));
  std::ranges::set_difference(previous_overlaps_, current,
                              std::back_inserter(exited));

  for (const ColliderPair& pair : entered) {
    events_.push_back(
        {TriggerEventType::kEnter, pair.first(), pair.second()});
  }
  for (const ColliderPair& pair : exited) {
    events_.push_back({TriggerEventType::kExit, pair.first(), pair.second()});
  }

  previous_overlaps_ = std::move(current);
}


}  // namespace gpr925