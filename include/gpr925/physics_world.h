//
// Created by alexk on 27.10.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_WORLD_H_
#define GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_WORLD_H_
#include <cstddef>
#include <memory>
#include <span>
#include <type_traits>
#include <utility>
#include <vector>

#include "scalar.h"
#include "vec2.h"
#include "body_utils.h"
#include "body_def.h"
#include "body.h"
#include "collider.h"
#include "collider_pair.h"
#include "collision.h"
#include "trigger_event.h"

namespace gpr925 {

class PhysicsWorld {
 public:
  PhysicsWorld() = default;

  PhysicsWorld(Scalar gravitational_constant, Scalar min_distance, Rect bounds, size_t size_to_reserve = 0);

  BodyId AddBody(const BodyDef& def);
  //void RemoveBody(BodyId id);

  template <class T, class... Args>
  T& CreateCollider(Args&&... args) {
    static_assert(std::is_base_of_v<Collider, T>,"CreateCollider needs a class derived from Collider");
    auto collider = std::make_unique<T>(std::forward<Args>(args)...);
    T& reference = *collider;
    colliders_.push_back(std::move(collider));
    return reference;
  }

  void Clear() {
    bodies_.clear();
    colliders_.clear();  // they point to bodies that no longer exist
    previous_overlaps_.clear();
    events_.clear();
  }

  void ReserveBodyVectorSize(size_t size_to_reserve) {
    bodies_.reserve(size_to_reserve);
  }

  [[nodiscard]] Body& GetBody(BodyId id) { return bodies_[static_cast<size_t>(id)]; }
  [[nodiscard]] const Body& GetBody(BodyId id) const { return bodies_[static_cast<size_t>(id)]; }
  [[nodiscard]] std::optional<ColliderType> GetColliderTypeWithBodyId(BodyId id) const;
  [[nodiscard]] const Collider* GetColliderWithBodyId(BodyId id) const;
  [[nodiscard]] std::size_t CountBody() const { return bodies_.size(); }
  [[nodiscard]] std::size_t CountCollider() const { return colliders_.size();}

  [[nodiscard]] std::span<const TriggerEvent> TriggerEvents() const {
    return events_;
  }

  // One fixed step: gather the forces, then integrate.
  void Step(Scalar dt, bool stay_in_bounds = false);

 private:
  struct ColliderEntry {
    BodyId body_id;
    const Collider* collider;
  };

  void ApplyGravity();
  void Integrate(Scalar dt);
  //Checks the pairs that start or stop overlapping
  void UpdateTriggers();
  void BounceInBounds();

  void BuildEntries();
  void ResolveCollisions();
  void ResolveContact(Body& a, Body& b, const Contact& contact, Scalar restitution);

  std::vector<Body> bodies_;
  Scalar gravitational_constant_{};
  Scalar min_distance_sqr_{};
  Rect bounds_;

  std::vector<std::unique_ptr<Collider>> colliders_;
  std::vector<ColliderEntry> entries_;

  //last step overlapping bodies
  std::vector<ColliderPair> previous_overlaps_;
  std::vector<TriggerEvent> events_;

};

}  // namespace gpr925

#endif  // GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_WORLD_H_