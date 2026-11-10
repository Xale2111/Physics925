//
// Created by alexk on 04.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_H_
#define GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_H_
#include <cstdint>
#include <vector>

#include "body_utils.h"
#include "collider_utils.h"
#include "gpr925/physics_material.h"
#include "scalar.h"
#include "vec2.h"

namespace gpr925 {


class Collider {
 public:
  // virtual: a collider is deleted through a Collider* (or unique_ptr<Collider>)
  virtual ~Collider() = default;

  [[nodiscard]] ColliderType GetType() const { return type_; }
  // Shift of the shape relative to the position of each body.
  [[nodiscard]] const Vec2& GetOffset() const { return offset_; }
  // Works like Unity IsTrigger (true -> sensor, false -> physical collider)
  [[nodiscard]] bool IsTrigger() const { return is_trigger_; }
  [[nodiscard]] const PhysicsMaterial& GetMaterial() const { return material_; }
  void SetMaterial(const PhysicsMaterial& material) { material_ = material; }

  void AddBody(BodyId body_id) { bodies_ids_.push_back(body_id); }

  void RemoveBody(BodyId body_id) { std::erase(bodies_ids_, body_id); }
  void ClearBodies() { bodies_ids_.clear(); }
  [[nodiscard]] bool HasBody() const { return !bodies_ids_.empty(); }
  [[nodiscard]] const std::vector<BodyId>& GetBodies() const {
    return bodies_ids_;
  }

 protected:
  Collider(const Collider&) = default;
  Collider(Collider&&) = default;
  Collider& operator=(const Collider&) = default;
  Collider& operator=(Collider&&) = default;
  Collider(ColliderType type, Vec2 offset, bool is_trigger, PhysicsMaterial material = {}) : type_(type), offset_(offset), is_trigger_(is_trigger), material_(material) {}

 private:
  ColliderType type_ : 2;
  Vec2 offset_;
  std::vector<BodyId> bodies_ids_;
  bool is_trigger_;           //THIS WILL ALIGN WITH 8 BYTES
  PhysicsMaterial material_;
};

}  // namespace gpr925
#endif  // GPR925PHYSICS_INCLUDE_GPR925_COLLIDER_H_
