//
// Created by alexk on 03.11.2026.
//

#include <gtest/gtest.h>

#include <ostream>
#include <vector>

#include "gpr925/physics_world.h"

namespace gpr925 {

// Readable failure messages.
void PrintTo(const TriggerEvent& event, std::ostream* os) {
  *os << (event.type == TriggerEventType::kEnter ? "Enter" : "Exit") << "("
      << static_cast<std::size_t>(event.a) << ", "
      << static_cast<std::size_t>(event.b) << ")";
}

namespace {

constexpr TriggerEventType kEnter = TriggerEventType::kEnter;
constexpr TriggerEventType kExit = TriggerEventType::kExit;

constexpr BodyId Id(std::size_t value) { return static_cast<BodyId>(value); }
Vec2 V(int x, int y) { return Vec2{Ratio(x, 1), Ratio(y, 1)}; }

// No gravity: the bodies only move at the velocity they were given.
PhysicsWorld MakeWorld() {
  return PhysicsWorld(Ratio(0, 1), Ratio(1, 2), Rect{V(-100, -100), V(100, 100)},
                      8);
}

BodyDef Body2D(Vec2 position, Vec2 velocity, std::optional<Collider> collider,
               BodyType type = BodyType::kDynamic) {
  BodyDef def;
  def.position = position;
  def.velocity = velocity;
  def.mass = Ratio(1, 1);
  def.type = type;
  def.collider = collider;
  return def;
}

Collider Trigger(int radius = 1) {
  return MakeCircleCollider(Ratio(radius, 1), true);
}
Collider Solid(int radius = 1) { return MakeCircleCollider(Ratio(radius, 1)); }

std::vector<TriggerEvent> Events(const PhysicsWorld& world) {
  const auto events = world.TriggerEvents();
  return {events.begin(), events.end()};
}

using EventList = std::vector<TriggerEvent>;
const Scalar kStep = Ratio(1, 1);

// ============================================================
// Colliders in bodies
// ============================================================

TEST(BodyColliderTest, BodyWithoutColliderShouldReportNone) {
  PhysicsWorld world = MakeWorld();
  const BodyId id = world.AddBody(Body2D(V(0, 0), V(0, 0), std::nullopt));
  EXPECT_FALSE(world.GetBody(id).HasCollider());
}

TEST(BodyColliderTest, BodyShouldKeepTheColliderOfItsDefinition) {
  PhysicsWorld world = MakeWorld();
  const BodyId id = world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger(3)));
  ASSERT_TRUE(world.GetBody(id).HasCollider());
  const Collider& collider = *world.GetBody(id).GetCollider();
  EXPECT_TRUE(collider.is_trigger);
  EXPECT_EQ(std::get<CircleShape>(collider.shape).radius, Ratio(3, 1));
}

TEST(BodyColliderTest, ColliderCanBeSetAndRemovedAfterCreation) {
  PhysicsWorld world = MakeWorld();
  const BodyId id = world.AddBody(Body2D(V(0, 0), V(0, 0), std::nullopt));
  world.GetBody(id).SetCollider(Solid(2));
  EXPECT_TRUE(world.GetBody(id).HasCollider());
  world.GetBody(id).RemoveCollider();
  EXPECT_FALSE(world.GetBody(id).HasCollider());
}

// ============================================================
// Enter and Exit
// ============================================================

TEST(TriggerEventsTest, NoEventsInAnEmptyWorld) {
  PhysicsWorld world = MakeWorld();
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(TriggerEventsTest, ShouldEmitEnterThenNothingWhileOverlappingThenExit) {
  PhysicsWorld world = MakeWorld();
  // A: static trigger at the origin. B: solid, moves one unit left per step.
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger(), BodyType::kStatic));
  world.AddBody(Body2D(V(5, 0), V(-1, 0), Solid()));

  // x: 4, 3 -> apart (distance > 2)
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());

  // x = 2: distance 2 = 1 + 1, touching -> Enter
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));

  // x = 1, 0, -1, -2: still overlapping -> nothing
  for (int i = 0; i < 4; ++i) {
    world.Step(kStep);
    EXPECT_TRUE(world.TriggerEvents().empty()) << "step " << i;
  }

  // x = -3: apart again -> Exit
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kExit, Id(0), Id(1)}}));

  // x = -4: still apart -> nothing
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(TriggerEventsTest, ShouldEmitEnterOnTheFirstStepIfAlreadyOverlapping) {
  PhysicsWorld world = MakeWorld();
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger(), BodyType::kStatic));
  world.AddBody(Body2D(V(1, 0), V(0, 0), Solid()));
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(TriggerEventsTest, EventsShouldBeReplacedByTheNextStep) {
  PhysicsWorld world = MakeWorld();
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger(), BodyType::kStatic));
  world.AddBody(Body2D(V(1, 0), V(0, 0), Solid()));
  world.Step(kStep);
  ASSERT_EQ(world.TriggerEvents().size(), 1u);
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(TriggerEventsTest, BodyPassingThroughInOneStepShouldProduceNoEvent) {
  // Enters and leaves within a single step: contacts are only sampled once
  // per step, so nothing is seen (the "tunneling" of the exercise).
  PhysicsWorld world = MakeWorld();
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger(), BodyType::kStatic));
  world.AddBody(Body2D(V(5, 0), V(-10, 0), Solid()));
  world.Step(kStep);  // x = -5
  EXPECT_TRUE(world.TriggerEvents().empty());
}

// ============================================================
// Who produces events
// ============================================================

TEST(TriggerEventsTest, BodiesWithoutColliderShouldBeIgnored) {
  PhysicsWorld world = MakeWorld();
  world.AddBody(Body2D(V(0, 0), V(0, 0), std::nullopt));
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger()));
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(TriggerEventsTest, TwoSolidCollidersShouldNotProduceTriggerEvents) {
  PhysicsWorld world = MakeWorld();
  world.AddBody(Body2D(V(0, 0), V(0, 0), Solid()));
  world.AddBody(Body2D(V(1, 0), V(0, 0), Solid()));
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(TriggerEventsTest, OneTriggerIsEnoughWhicheverBodyItIs) {
  for (int trigger_first = 0; trigger_first < 2; ++trigger_first) {
    PhysicsWorld world = MakeWorld();
    world.AddBody(Body2D(V(0, 0), V(0, 0), trigger_first ? Trigger() : Solid()));
    world.AddBody(Body2D(V(1, 0), V(0, 0), trigger_first ? Solid() : Trigger()));
    world.Step(kStep);
    EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}))
        << "trigger_first = " << trigger_first;
  }
}

TEST(TriggerEventsTest, TwoTriggersShouldProduceOneEventForTheirPair) {
  PhysicsWorld world = MakeWorld();
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger()));
  world.AddBody(Body2D(V(1, 0), V(0, 0), Trigger()));
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
}

TEST(TriggerEventsTest, EventIdsShouldBeInIncreasingOrder) {
  PhysicsWorld world = MakeWorld();
  world.AddBody(Body2D(V(1, 0), V(0, 0), Solid()));                       // id 0
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger(), BodyType::kStatic));  // id 1
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
}

// ============================================================
// Several pairs
// ============================================================

TEST(TriggerEventsTest, ManyPairsShouldBeReportedInSortedOrder) {
  PhysicsWorld world = MakeWorld();
  for (int i = 0; i < 3; ++i) {
    world.AddBody(Body2D(V(i, 0), V(0, 0), Trigger()));  // all overlap
  }
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)},
                                      {kEnter, Id(0), Id(2)},
                                      {kEnter, Id(1), Id(2)}}));
}

TEST(TriggerEventsTest, EnterAndExitInTheSameStepShouldBothBeReported) {
  PhysicsWorld world = MakeWorld();
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger(), BodyType::kStatic));  // A
  const BodyId b = world.AddBody(Body2D(V(1, 0), V(0, 0), Solid()));      // B in
  const BodyId c = world.AddBody(Body2D(V(-5, 0), V(0, 0), Solid()));     // C out
  world.Step(kStep);  // warm-up: Enter(A, B)
  ASSERT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));

  world.GetBody(b).SetVelocity(V(5, 0));  // leaves: x = 6
  world.GetBody(c).SetVelocity(V(5, 0));  // arrives: x = 0
  world.Step(kStep);
  // Enters first, then exits.
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(2)},
                                      {kExit, Id(0), Id(1)}}));
}

// ============================================================
// Changing colliders and clearing
// ============================================================

TEST(TriggerEventsTest, RemovingTheColliderOfAnOverlappingBodyShouldEmitExit) {
  PhysicsWorld world = MakeWorld();
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger(), BodyType::kStatic));
  const BodyId b = world.AddBody(Body2D(V(1, 0), V(0, 0), Solid()));
  world.Step(kStep);
  ASSERT_EQ(world.TriggerEvents().size(), 1u);

  world.GetBody(b).RemoveCollider();
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kExit, Id(0), Id(1)}}));
}

TEST(TriggerEventsTest, ClearShouldForgetThePreviousContacts) {
  PhysicsWorld world = MakeWorld();
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger(), BodyType::kStatic));
  world.AddBody(Body2D(V(1, 0), V(0, 0), Solid()));
  world.Step(kStep);
  ASSERT_EQ(world.TriggerEvents().size(), 1u);

  world.Clear();
  EXPECT_TRUE(world.TriggerEvents().empty());
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger(), BodyType::kStatic));
  world.AddBody(Body2D(V(1, 0), V(0, 0), Solid()));
  world.Step(kStep);
  // A fresh world: the same contact is a new Enter, not a leftover.
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
}

TEST(TriggerEventsTest, ColliderOffsetShouldMoveTheContactZone) {
  PhysicsWorld world = MakeWorld();
  // Trigger body at the origin, but its circle sits at (10, 0).
  world.AddBody(Body2D(
      V(0, 0), V(0, 0), MakeCircleCollider(Ratio(1, 1), true, V(10, 0)),
      BodyType::kStatic));
  world.AddBody(Body2D(V(0, 0), V(0, 0), Solid()));  // at the origin: apart
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
  world.AddBody(Body2D(V(10, 0), V(0, 0), Solid()));  // on the circle
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(2)}}));
}

// ============================================================
// Both ways of stepping
// ============================================================

TEST(TriggerEventsTest, StepFreeRoamShouldProduceEventsToo) {
  PhysicsWorld world = MakeWorld();
  world.AddBody(Body2D(V(0, 0), V(0, 0), Trigger(), BodyType::kStatic));
  world.AddBody(Body2D(V(4, 0), V(-1, 0), Solid()));
  world.StepFreeRoam(kStep);  // x = 3
  EXPECT_TRUE(world.TriggerEvents().empty());
  world.StepFreeRoam(kStep);  // x = 2: touching
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
}

TEST(TriggerEventsTest, GravityShouldBringBodiesIntoContact) {
  // A heavy static trigger attracts a light body that starts apart.
  PhysicsWorld world(Ratio(1, 1), Ratio(1, 2), Rect{V(-100, -100), V(100, 100)}, 2);
  BodyDef heavy = Body2D(V(0, 0), V(0, 0), Trigger(), BodyType::kStatic);
  heavy.mass = Ratio(100, 1);
  world.AddBody(heavy);
  world.AddBody(Body2D(V(6, 0), V(0, 0), Solid()));

  bool entered = false;
  for (int i = 0; i < 600 && !entered; ++i) {
    world.Step(Ratio(1, 60));
    for (const TriggerEvent& event : world.TriggerEvents()) {
      if (event.type == kEnter) entered = true;
    }
  }
  EXPECT_TRUE(entered);
}

}  // namespace
}  // namespace gpr925