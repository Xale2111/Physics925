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
    position_ = def.position;
    velocity_ = def.velocity;
    mass_ = def.mass;
    type_ = def.type;
    angle_ = def.angle;
    angular_velocity_ = def.angular_velocity;
    inertia_ = def.inertia;
  }

  [[nodiscard]] Vec2 const& GetPosition() const { return position_; }
  void SetPosition(const Vec2& new_position) { position_ = new_position; }
  [[nodiscard]] Vec2 const& GetVelocity() const { return velocity_; }
  void SetVelocity(const Vec2& new_velocity) { velocity_ = new_velocity; }
  [[nodiscard]] Scalar const& GetMass() const { return mass_; }
  [[nodiscard]] BodyType const& GetBodyType() const { return type_;}
  [[nodiscard]] Vec2 const& GetForce() const { return force_;}
  void SetForce(const Vec2& new_forces) { force_ = new_forces;}

  // A force_ acts during the whole step: it changes the velocity_ by (F/m)*dt.
  void ApplyForce(Vec2 applied_force) { force_ += applied_force; }

  [[nodiscard]] Scalar GetAngle() const {return angle_;}
  void SetAngle(Scalar new_angle){angle_ = new_angle;}
  [[nodiscard]] Scalar GetAngularVelocity() const {return angular_velocity_;}
  void SetAngularVelocity(Scalar new_angular_velocity){angular_velocity_ = new_angular_velocity;}
  [[nodiscard]] Scalar GetInertia() const {return inertia_;}
  [[nodiscard]] Scalar GetTorque() const {return torque_;}
  void SetTorque(Scalar new_torque){torque_ = new_torque;}

  [[nodiscard]] Scalar GetInverseMass() const{
    return (type_ == BodyType::kStatic || mass_ <= Scalar{}) ? Scalar{} : Scalar{1}/mass_;
  }
  [[nodiscard]] Scalar GetInverseInertia() const{
    return (type_ == BodyType::kStatic || inertia_ <= Scalar{}) ? Scalar{} : Scalar{1}/inertia_;
  }

  void ApplyTorque(Scalar applied_torque){torque_ += applied_torque;}
  void ApplyForceAtPoint(const Vec2& applied_force, const Vec2& world_point){
    force_ += applied_force;
    torque_ += Vec2::Cross(world_point - position_, applied_force);
  }

 private:
  Vec2 position_{};
  Vec2 velocity_{};
  Scalar mass_{};
  // Sum of the forces applied since the last step. Cleared by the world.
  Vec2 force_{};
  BodyType type_{};

  Scalar angle_{};            //radians
  Scalar angular_velocity_{}; //radians/second
  Scalar inertia_{};          //moment of inertia, 0 -> not rotation
  Scalar torque_{};           //sum of the torques applied since the last step
};

} // namespace gpr925

#endif  // GPR925PHYSICS_INCLUDE_GPR925_BODY_H_
