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

std::optional<ColliderType> PhysicsWorld::GetColliderTypeWithBodyId(BodyId id) const {
  const Collider* collider = GetColliderWithBodyId(id);
  if (collider == nullptr) {
    return std::nullopt;
  }
  return collider->GetType();
}

const Collider* PhysicsWorld::GetColliderWithBodyId(BodyId id) const {
  for (const auto& collider : colliders_) {
    const std::vector<BodyId>& bodies = collider->GetBodies();
    if (std::ranges::find(bodies, id) != bodies.end()) {
      return collider.get();
    }
  }
  return nullptr;  // no collider is linked to this body
}

void PhysicsWorld::Step(Scalar dt, bool stay_in_bounds) {
  events_.clear();
  ApplyGravity();  // sum the forces
  Integrate(dt);   // velocity_, then position_
  if (stay_in_bounds) {
    BounceInBounds();  // looks at the new position_, reverses velocity_
  }
  BuildEntries();
  BuildBroadphase();
  ResolveCollisions();
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
  std::vector<ColliderPair> current;
  for (const BroadphasePair& pair : candidate_pairs_) {
    const ColliderEntry& entry_a = entries_[pair.first];
    const ColliderEntry& entry_b = entries_[pair.second];
    if (entry_a.body_id == entry_b.body_id) continue;
    if (!entry_a.collider->IsTrigger() && !entry_b.collider->IsTrigger()) continue;
    const Vec2& position_a = bodies_[static_cast<std::size_t>(entry_a.body_id)].GetPosition();
    const Vec2& position_b = bodies_[static_cast<std::size_t>(entry_b.body_id)].GetPosition();
    if (Overlap(*entry_a.collider, position_a, *entry_b.collider, position_b)) {
      current.emplace_back(entry_a.body_id, entry_b.body_id);
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

void PhysicsWorld::BuildEntries() {
  entries_.clear();
  for (const auto& collider : colliders_) {
    for (const BodyId body_id : collider->GetBodies()) {
      entries_.push_back({body_id, collider.get()});
    }
  }
}

void PhysicsWorld::ResolveCollisions() {
  for (const BroadphasePair& pair : candidate_pairs_) {
    const ColliderEntry& ea = entries_[pair.first];
    const ColliderEntry& eb = entries_[pair.second];
    if (ea.body_id == eb.body_id) continue;
    if (ea.collider->IsTrigger() || eb.collider->IsTrigger()) continue;

    Body& a = GetBody(ea.body_id);
    Body& b = GetBody(eb.body_id);
    const auto contact = GenerateContact(*ea.collider, a.GetPosition(), *eb.collider, b.GetPosition());
    if (contact) {
      const Scalar restitution = CombineBounciness(ea.collider->GetMaterial(), eb.collider->GetMaterial());
      ResolveContact(a, b, *contact, restitution);
    }
  }
}

void PhysicsWorld::ResolveContact(Body& a, Body& b, const Contact& contact, Scalar restitution) {
  const bool a_static = a.GetBodyType() == BodyType::kStatic;
  const bool b_static = b.GetBodyType() == BodyType::kStatic;
  if (a_static && b_static) return;  // nothing can move

  //Check if one of the bodies is static, if so, the other one will get restitution of 1
  //if none are static, we calculate the restitution each should have depending on the mass of the two bodies
  Scalar share_a = Scalar{1};
  Scalar share_b = Scalar{1};
  if (a_static) {
    share_a = Scalar{};
  } else if (b_static) {
    share_b = Scalar{};
  } else {
    const Scalar total_mass = a.GetMass() + b.GetMass();
    share_a = b.GetMass() / total_mass;
    share_b = a.GetMass() / total_mass;
  }

  //Setting the velocity of the two bodies depending on the current velocity + the restitution * the normal of the contact
  const Scalar separating_speed = Vec2::Dot(b.GetVelocity() - a.GetVelocity(), contact.normal);
  if (separating_speed < Scalar{}) {
    const Scalar delta_velocity = (Scalar{1} + restitution) * separating_speed;
    a.SetVelocity(a.GetVelocity() + contact.normal * (delta_velocity * share_a));
    b.SetVelocity(b.GetVelocity() - contact.normal * (delta_velocity * share_b));
  }

  //Setting the position of the bodies so they are not overlapping anymore
  a.SetPosition(a.GetPosition() - contact.normal * (contact.penetration * share_a));
  b.SetPosition(b.GetPosition() + contact.normal * (contact.penetration * share_b));
}

void PhysicsWorld::BuildBroadphase() {
  // Rebuilt from scratch every step: the bodies moved. (Naive: this allocates.)
  quadtree_ = QuadTree(bounds_, 0);
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const ColliderEntry& entry = entries_[i];
    const Vec2& position = bodies_[static_cast<std::size_t>(entry.body_id)].GetPosition();
    quadtree_.Insert({i, ComputeAabb(*entry.collider, position)});  // id = index in entries_
  }
  candidate_pairs_.clear();  // keeps its capacity: no allocation after the first steps
  quadtree_.GeneratePairs(candidate_pairs_);
}

}  // namespace gpr925