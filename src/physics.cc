// Your physics engine starts here: declare it in include/gpr925/ and implement
// it in src/. Every .cc file of this folder is compiled into the Gpr925Physics
// library, for PC and for the DS.

#include "gpr925/physics_world.h"

#include <cassert>

namespace gpr925 {

PhysicsWorld::PhysicsWorld(Scalar gravitational_constant, Scalar min_distance)
    : gravitational_constant_(gravitational_constant),
      min_distance_sqr_(min_distance * min_distance) {}

// the mass of the body must be strictly positive
BodyId PhysicsWorld::CreateBody(const BodyDef& def) {
  assert(def.mass > Scalar{} && "a body needs a strictly positive mass");
  Body body;
  body.position = def.position;
  body.velocity = def.velocity;
  body.mass = def.mass;
  body.type = def.type;
  bodies_.push_back(body);
  return bodies_.size() - 1;
}

void PhysicsWorld::Step(Scalar dt) {
  ApplyGravity();  // sum the forces
  Integrate(dt);   // velocity, then position
}


void PhysicsWorld::ApplyGravity() {
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    for (std::size_t j = i + 1; j < bodies_.size(); ++j) {
      Body& a = bodies_[i];
      Body& b = bodies_[j];
      if (a.type == BodyType::kStatic && b.type == BodyType::kStatic) {
        continue;  // neither can move: the force would be discarded
      }

      const Vec2 delta = b.position - a.position;  // from a to b
      Scalar sqr_distance = delta.SqrMagnitude();
      //check distance between bodies. If it is too small, set it to the minimum distance
      if (sqr_distance < min_distance_sqr_) {
        sqr_distance = min_distance_sqr_;
      }

      const Scalar distance = maths::NumberTraits<Scalar>::Sqrt(sqr_distance);

      // Gravitation law -> F = (G * m1 * m2) / r^2
      const Scalar magnitude = gravitational_constant_ * a.mass * b.mass / sqr_distance;
      const Vec2 force_on_a = delta * (magnitude / distance);

      // third law: equal and opposite
      a.ApplyForce(force_on_a);
      b.ApplyForce(-force_on_a);
    }
  }
}

void PhysicsWorld::Integrate(Scalar dt) {
  for (Body& body : bodies_) {
    if (body.type == BodyType::kDynamic) {
      //Second law: a = F / m
      const Vec2 acceleration{body.force.x() / body.mass,
                              body.force.y() / body.mass};
      //semi implicit Euler integration
      body.velocity += acceleration * dt;  // velocity first
      body.position += body.velocity * dt;  //then position, new velocity
    }
    body.force = Vec2{};  //reset forces
  }
}

}  // namespace gpr925