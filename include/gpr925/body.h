//
// Created by alexk on 29.10.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_BODY_H_
#define GPR925PHYSICS_INCLUDE_GPR925_BODY_H_
#include "scalar.h"
#include "vec2.h"
#include "body_def.h"
#include "body_utils.h"

namespace gpr925 {

class Body{
 public:
  Body() = default;

  Body(BodyDef def) {
    position = def.position;
    velocity = def.velocity;
    mass = def.mass;
    type = def.type;
  }

  [[nodiscard]] Vec2 const& GetPosition() const { return position; }
  void SetPosition(const Vec2& new_position) { position = new_position; }
  [[nodiscard]] Vec2 const& GetVelocity() const { return velocity; }
  void SetVelocity(const Vec2& new_velocity) { velocity = new_velocity; }
  [[nodiscard]] Scalar const& GetMass() const { return mass; }
  [[nodiscard]] BodyType const& GetBodyType() const { return type;}
  [[nodiscard]] Vec2 const& GetForce() const { return force;}
  void SetForce(const Vec2& new_forces) { force = new_forces;}


  // A force acts during the whole step: it changes the velocity by (F/m)*dt.
  void ApplyForce(Vec2 applied_force) { force += applied_force; }

 private:
  Vec2 position{};
  Vec2 velocity{};
  Scalar mass{};
  BodyType type = BodyType::kDynamic;
  // Sum of the forces applied since the last step. Cleared by the world.
  Vec2 force{};
};

} // namespace gpr925

#endif  // GPR925PHYSICS_INCLUDE_GPR925_BODY_H_
