//
// Created by alexk on 27.10.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_WORLD_H_
#define GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_WORLD_H_
#include <cstddef>
#include <vector>

#include "scalar.h"
#include "vec2.h"

namespace gpr925 {

// Using size_t to have access to the bodies by index.
using BodyId = std::size_t;

enum class BodyType {
  kStatic,   // the object does not move
  kDynamic,  // forces are applied to it
};

// Everything needed to create a body.
struct BodyDef {
  Vec2 position{};
  Vec2 velocity{};
  Scalar mass{};
  BodyType type = BodyType::kDynamic;
};

struct Body {
  Vec2 position{};
  Vec2 velocity{};
  Scalar mass{};
  BodyType type = BodyType::kDynamic;
  // Sum of the forces applied since the last step. Cleared by the world.
  Vec2 force{};

  // A force acts during the whole step: it changes the velocity by (F/m)*dt.
  void ApplyForce(const Vec2& applied_force) { force += applied_force; }
};

class PhysicsWorld {
 public:

  PhysicsWorld() = default;

  // gravitational_constant: the G of F = G*m1*m2/r^2.
  // min_distance: below this distance the attraction stops growing.
  PhysicsWorld(Scalar gravitational_constant, Scalar min_distance);

  BodyId CreateBody(const BodyDef& def);

  void Clear(){bodies_.clear();}
  void ReserveBodyVectorSize (size_t size_to_reserve){bodies_.reserve(size_to_reserve);}
  [[nodiscard]] Body& GetBody(BodyId id) { return bodies_[id]; }
  [[nodiscard]] const Body& GetBody(BodyId id) const { return bodies_[id]; }
  [[nodiscard]] std::size_t CountBody() const { return bodies_.size(); }

  // One fixed step: gather the forces, then integrate.
  void Step(Scalar dt);

 private:
  void ApplyGravity();
  void Integrate(Scalar dt);

  std::vector<Body> bodies_;
  Scalar gravitational_constant_ = {};
  Scalar min_distance_sqr_ = {};
};

}  // namespace gpr925

#endif  // GPR925PHYSICS_INCLUDE_GPR925_PHYSICS_WORLD_H_
