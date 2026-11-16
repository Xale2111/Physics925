//
// Created by alexk on 16.11.2026.
//

#include <gtest/gtest.h>

#include "gpr925/body_def.h"
#include "gpr925/inertia.h"
#include "gpr925/physics_world.h"

namespace gpr925 {
namespace {

// Whole numbers only: exact in float AND in Fixed 20.12.
constexpr Scalar S(int value) { return static_cast<Scalar>(value); }

BodyDef MakeDef(int mass, int inertia) {
  BodyDef def;
  def.mass = S(mass);
  def.inertia = S(inertia);
  return def;
}

TEST(Cross, UnitAxesAndParallelVectors) {
  EXPECT_EQ(Vec2::Cross(Vec2{S(1), S(0)}, Vec2{S(0), S(1)}), S(1));
  EXPECT_EQ(Vec2::Cross(Vec2{S(0), S(1)}, Vec2{S(1), S(0)}), S(-1));  // order matters
  EXPECT_EQ(Vec2::Cross(Vec2{S(2), S(3)}, Vec2{S(4), S(6)}), S(0));   // parallel
}

TEST(Inertia, DiscAndBox) {
  EXPECT_EQ(DiscInertia(S(2), S(3)), S(9));         // 2 * 3^2 / 2
  EXPECT_EQ(BoxInertia(S(12), S(3), S(4)), S(25));  // 12 * (9 + 16) / 12
}

TEST(Rotation, TorqueSpinsTheBody) {
  PhysicsWorld world;
  const BodyId id = world.AddBody(MakeDef(1, 5));
  world.GetBody(id).ApplyTorque(S(10));
  world.Step(S(1));
  EXPECT_EQ(world.GetBody(id).GetAngularVelocity(), S(2));  // 10 / 5 * 1
  EXPECT_EQ(world.GetBody(id).GetAngle(), S(2));            // omega * dt
  // The torque was cleared: no new acceleration, the body keeps its speed.
  world.Step(S(1));
  EXPECT_EQ(world.GetBody(id).GetAngularVelocity(), S(2));
  EXPECT_EQ(world.GetBody(id).GetAngle(), S(4));
}

TEST(Rotation, ForceOffCenterMovesAndTurns) {
  PhysicsWorld world;
  const BodyId id = world.AddBody(MakeDef(1, 1));
  // Push down (0,1) at the point (2,0): lever arm (2,0), torque 2*1 - 0*0 = 2.
  world.GetBody(id).ApplyForceAtPoint(Vec2{S(0), S(1)}, Vec2{S(2), S(0)});
  world.Step(S(1));
  EXPECT_EQ(world.GetBody(id).GetAngularVelocity(), S(2));
  EXPECT_EQ(world.GetBody(id).GetVelocity().y(), S(1));  // translation as usual
}

TEST(Rotation, ForceThroughTheCenterDoesNotTurn) {
  PhysicsWorld world;
  const BodyId id = world.AddBody(MakeDef(1, 1));
  // The point is on the line of the force: r and F are parallel, r x F = 0.
  world.GetBody(id).ApplyForceAtPoint(Vec2{S(1), S(0)}, Vec2{S(3), S(0)});
  world.Step(S(1));
  EXPECT_EQ(world.GetBody(id).GetAngularVelocity(), S(0));
  EXPECT_EQ(world.GetBody(id).GetVelocity().x(), S(1));
}

TEST(Rotation, ZeroInertiaNeverRotates) {
  PhysicsWorld world;
  const BodyId id = world.AddBody(MakeDef(1, 0));
  world.GetBody(id).ApplyTorque(S(10));
  world.Step(S(1));
  EXPECT_EQ(world.GetBody(id).GetAngularVelocity(), S(0));
  EXPECT_EQ(world.GetBody(id).GetAngle(), S(0));
}

TEST(Rotation, StaticBodyDoesNotRotate) {
  PhysicsWorld world;
  BodyDef def = MakeDef(1, 5);
  def.type = BodyType::kStatic;
  const BodyId id = world.AddBody(def);
  world.GetBody(id).ApplyTorque(S(10));
  world.Step(S(1));
  EXPECT_EQ(world.GetBody(id).GetAngle(), S(0));
}

TEST(Rotation, InitialAngularVelocityKeepsSpinning) {
  PhysicsWorld world;
  BodyDef def = MakeDef(1, 5);
  def.angular_velocity = S(3);
  const BodyId id = world.AddBody(def);
  world.Step(S(2));  // no torque: constant angular velocity
  EXPECT_EQ(world.GetBody(id).GetAngularVelocity(), S(3));
  EXPECT_EQ(world.GetBody(id).GetAngle(), S(6));
}

}  // namespace
}  // namespace gpr925