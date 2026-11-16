//
// Created by alexk on 16.11.2026.
//
#include <gtest/gtest.h>

#include <cmath>

#include "gpr925/body.h"
#include "gpr925/box_collider.h"
#include "gpr925/circle_collider.h"
#include "gpr925/collision.h"
#include "gpr925/contact_resolution.h"

namespace gpr925 {
namespace {

// Whole numbers only: exact in float AND in Fixed 20.12.
constexpr Scalar S(int value) { return static_cast<Scalar>(value); }
Vec2 V(int x, int y) { return Vec2{S(x), S(y)}; }

Body MakeBody(Vec2 position, Vec2 velocity, int mass, int inertia, BodyType type = BodyType::kDynamic) {
  BodyDef def;
  def.position = position;
  def.velocity = velocity;
  def.mass = S(mass);
  def.inertia = S(inertia);
  def.type = type;
  return Body(def);
}

TEST(ResolveContact, HeadOnEqualMassesSwapVelocities) {
  Body a = MakeBody(V(0, 0), V(1, 0), 1, 0);
  Body b = MakeBody(V(2, 0), V(-1, 0), 1, 0);
  ResolveContact(a, b, Contact{V(1, 0), S(0), V(1, 0)}, S(1), S(0));  // e = 1
  EXPECT_EQ(a.GetVelocity().x(), S(-1));
  EXPECT_EQ(b.GetVelocity().x(), S(1));
}

TEST(ResolveContact, StaticGroundStopsTheBallWithoutRestitution) {
  Body ball = MakeBody(V(0, 0), V(0, 3), 1, 0);
  Body ground = MakeBody(V(0, 2), V(0, 0), 1, 0, BodyType::kStatic);
  ResolveContact(ball, ground, Contact{V(0, 1), S(0), V(0, 1)}, S(0), S(0));
  EXPECT_EQ(ball.GetVelocity().y(), S(0));
  EXPECT_EQ(ground.GetVelocity().y(), S(0));
}

TEST(ResolveContact, StaticGroundBouncesTheBall) {
  Body ball = MakeBody(V(0, 0), V(0, 3), 1, 0);
  Body ground = MakeBody(V(0, 2), V(0, 0), 1, 0, BodyType::kStatic);
  ResolveContact(ball, ground, Contact{V(0, 1), S(0), V(0, 1)}, S(1), S(0));
  EXPECT_EQ(ball.GetVelocity().y(), S(-3));
}

// B is hit off-center: the impulse also spins it.
TEST(ResolveContact, OffCenterHitSpinsTheBody) {
  Body wall = MakeBody(V(0, -2), V(0, 0), 1, 0, BodyType::kStatic);
  Body b = MakeBody(V(0, 0), V(0, -1), 1, 1);  // inertia 1, moving toward the wall
  // Contact at (1, -1): lever arm r_b = (1, -1), normal (0, 1): r x n = 1.
  ResolveContact(wall, b, Contact{V(0, 1), S(0), V(1, -1)}, S(1), S(0));
  EXPECT_EQ(b.GetVelocity().y(), S(0));        // 2 / (1 + 1) = j = 1: v = -1 + 1
  EXPECT_EQ(b.GetAngularVelocity(), S(1));     // delta_omega = (r x J) / I = 1
}

TEST(ResolveContact, ZeroInertiaGivesNoSpin) {
  Body wall = MakeBody(V(0, -2), V(0, 0), 1, 0, BodyType::kStatic);
  Body b = MakeBody(V(0, 0), V(0, -1), 1, 0);  // same hit, no inertia
  ResolveContact(wall, b, Contact{V(0, 1), S(0), V(1, -1)}, S(1), S(0));
  EXPECT_EQ(b.GetVelocity().y(), S(1));  // j = 2 / 1
  EXPECT_EQ(b.GetAngularVelocity(), S(0));
}

// Energy check with e = 1: the kinetic energy is the same before and after.
TEST(ResolveContact, ElasticHitKeepsKineticEnergy) {
  Body wall = MakeBody(V(0, -2), V(0, 0), 1, 0, BodyType::kStatic);
  Body b = MakeBody(V(0, 0), V(0, -1), 1, 1);
  const Scalar before = b.GetMass() * Vec2::Dot(b.GetVelocity(), b.GetVelocity()) / S(2);
  ResolveContact(wall, b, Contact{V(0, 1), S(0), V(1, -1)}, S(1), S(0));
  const Scalar after = b.GetMass() * Vec2::Dot(b.GetVelocity(), b.GetVelocity()) / S(2) +
                       b.GetInertia() * b.GetAngularVelocity() * b.GetAngularVelocity() / S(2);
  EXPECT_EQ(before, after);
}

TEST(ResolveContact, SeparatingBodiesGetNoImpulse) {
  Body a = MakeBody(V(0, 0), V(-1, 0), 1, 0);
  Body b = MakeBody(V(2, 0), V(1, 0), 1, 0);
  ResolveContact(a, b, Contact{V(1, 0), S(0), V(1, 0)}, S(1), S(0));
  EXPECT_EQ(a.GetVelocity().x(), S(-1));
  EXPECT_EQ(b.GetVelocity().x(), S(1));
}

TEST(ResolveContact, PenetrationIsSplitByInverseMass) {
  Body a = MakeBody(V(0, 0), V(0, 0), 1, 0);
  Body b = MakeBody(V(1, 0), V(0, 0), 3, 0);
  ResolveContact(a, b, Contact{V(1, 0), S(4), V(0, 0)}, S(0), S(0));
  // 1/3 is not exact in Fixed 20.12: compare with a tolerance, not with ==.
  EXPECT_TRUE(std::fabs(ToFloat(a.GetPosition().x()) - (-3.0F)) < 5e-3F);  // the light body moves 3/4 of it
  EXPECT_TRUE(std::fabs(ToFloat(b.GetPosition().x()) - 2.0F) < 5e-3F);     // the heavy one 1/4
}

TEST(ResolveContact, TwoStaticBodiesDoNothing) {
  Body a = MakeBody(V(0, 0), V(0, 0), 1, 0, BodyType::kStatic);
  Body b = MakeBody(V(1, 0), V(0, 0), 1, 0, BodyType::kStatic);
  ResolveContact(a, b, Contact{V(1, 0), S(2), V(0, 0)}, S(1), S(0));
  EXPECT_EQ(a.GetPosition().x(), S(0));
  EXPECT_EQ(b.GetPosition().x(), S(1));
}

// A ball of radius 1 slides on the ground (below it, normal (0, 1)). The contact
// point is under the ball: r_a = (0, 1). Inertia 1, mass 1.
Body MakeSlidingBall() { return MakeBody(V(0, 0), V(4, 1), 1, 1); }
Body MakeGround() { return MakeBody(V(0, 2), V(0, 0), 1, 0, BodyType::kStatic); }
const Contact kGroundContact{V(0, 1), S(0), V(0, 1)};

TEST(Friction, ZeroFrictionDoesNotSlowOrSpin) {
  Body ball = MakeSlidingBall();
  Body ground = MakeGround();
  ResolveContact(ball, ground, kGroundContact, S(0), S(0));
  EXPECT_EQ(ball.GetVelocity().x(), S(4));
  EXPECT_EQ(ball.GetAngularVelocity(), S(0));
}

// Normal impulse j = 1. Friction wanted: 2 (it would stop the sliding), limit
// mu * j = 1: the ball slows from 4 to 3 and starts to spin.
TEST(Friction, CoulombLimitCapsTheImpulse) {
  Body ball = MakeSlidingBall();
  Body ground = MakeGround();
  ResolveContact(ball, ground, kGroundContact, S(0), S(1));
  EXPECT_EQ(ball.GetVelocity().x(), S(3));
  EXPECT_EQ(ball.GetAngularVelocity(), S(1));
}

// Plenty of friction: the sliding stops exactly, the ball rolls without slipping:
// speed of the contact point = v - omega * r = 2 - 2 * 1 = 0.
TEST(Friction, StrongFrictionMakesTheBallRoll) {
  Body ball = MakeSlidingBall();
  Body ground = MakeGround();
  ResolveContact(ball, ground, kGroundContact, S(0), S(10));
  EXPECT_EQ(ball.GetVelocity().x(), S(2));
  EXPECT_EQ(ball.GetAngularVelocity(), S(2));
}

// No normal impulse (the bodies separate): nothing presses them, no friction.
TEST(Friction, NoPressureNoFriction) {
  Body ball = MakeBody(V(0, 0), V(4, -1), 1, 1);
  Body ground = MakeGround();
  ResolveContact(ball, ground, kGroundContact, S(0), S(10));
  EXPECT_EQ(ball.GetVelocity().x(), S(4));
  EXPECT_EQ(ball.GetAngularVelocity(), S(0));
}

TEST(Friction, CombineIsTheGeometricMean) {
  EXPECT_EQ(CombineFriction(PhysicsMaterial{S(0), S(4)}, PhysicsMaterial{S(0), S(9)}), S(6));
  EXPECT_EQ(CombineFriction(PhysicsMaterial{S(0), S(0)}, PhysicsMaterial{S(0), S(9)}), S(0));
}

TEST(ContactPoint, CircleCircleIsOnTheSurfaceOfA) {
  const CircleCollider a(S(2), V(0, 0), false);
  const CircleCollider b(S(2), V(0, 0), false);
  const auto contact = CircleCircleContact(a, V(0, 0), b, V(3, 0));
  EXPECT_TRUE(contact.has_value());
  EXPECT_EQ(contact->point.x(), S(2));  // center_a + n * r_a
  EXPECT_EQ(contact->point.y(), S(0));
}

TEST(ContactPoint, CircleBoxIsTheClosestPointOfTheBox) {
  const CircleCollider circle(S(2), V(0, 0), false);
  const BoxCollider box(V(-5, 0), V(5, 4), V(0, 0), false);  // top face at y = 0
  const auto contact = CircleBoxContact(circle, V(1, -1), box, V(0, 0));
  EXPECT_TRUE(contact.has_value());
  EXPECT_EQ(contact->point.x(), S(1));
  EXPECT_EQ(contact->point.y(), S(0));
}

}  // namespace
}  // namespace gpr925