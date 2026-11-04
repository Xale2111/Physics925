//
// Scenario tests for the Enter/Exit events of gpr925::PhysicsWorld.
//

#include <gtest/gtest.h>

#include <ostream>
#include <vector>

#include "gpr925/circle_collider.h"
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

// What collider a test body gets (one circle collider per body).
struct ColliderSpec {
  bool present;
  int radius;
  bool is_trigger;
  Vec2 offset;
};
ColliderSpec Trigger(int radius = 1) { return {true, radius, true, Vec2{}}; }
ColliderSpec Solid(int radius = 1) { return {true, radius, false, Vec2{}}; }
ColliderSpec NoCollider() { return {false, 0, false, Vec2{}}; }

// Creates the body and, if asked, its own circle collider (owned by the world).
BodyId AddBody(PhysicsWorld& world, Vec2 position, Vec2 velocity,
               ColliderSpec collider, BodyType type = BodyType::kDynamic,
               Scalar mass = Ratio(1, 1)) {
  BodyDef def;
  def.position = position;
  def.velocity = velocity;
  def.mass = mass;
  def.type = type;
  const BodyId id = world.AddBody(def);
  if (collider.present) {
    world.CreateCollider<CircleCollider>(Ratio(collider.radius, 1),
                                         collider.offset, collider.is_trigger)
        .AddBody(id);
  }
  return id;
}

BodyId AddBody(PhysicsWorld& world, Vec2 position, Vec2 velocity) {
  return AddBody(world, position, velocity, NoCollider());
}

std::vector<TriggerEvent> Events(const PhysicsWorld& world) {
  const auto events = world.TriggerEvents();
  return {events.begin(), events.end()};
}

using EventList = std::vector<TriggerEvent>;
const Scalar kStep = Ratio(1, 1);

// ============================================================
// Colliders owned by the world
// ============================================================

TEST(WorldColliderTest, CreateColliderShouldReturnAWorkingCollider) {
  PhysicsWorld world = MakeWorld();
  CircleCollider& collider = world.CreateCollider<CircleCollider>(Ratio(3, 1), true);
  EXPECT_EQ(collider.GetRadius(), Ratio(3, 1));
  EXPECT_TRUE(collider.IsTrigger());
  EXPECT_EQ(world.CountCollider(), 1u);
}

TEST(WorldColliderTest, ReferencesShouldStayValidWhenMoreCollidersAreCreated) {
  PhysicsWorld world = MakeWorld();
  CircleCollider& first = world.CreateCollider<CircleCollider>(Ratio(5, 1), false);
  first.AddBody(Id(0));
  for (int i = 0; i < 200; ++i) {  // enough to move the vector several times
    world.CreateCollider<CircleCollider>(Ratio(1, 1), false);
  }
  EXPECT_EQ(first.GetRadius(), Ratio(5, 1));
  EXPECT_EQ(first.GetBodies().size(), 1u);
  EXPECT_EQ(world.CountCollider(), 201u);
}

TEST(WorldColliderTest, ClearShouldRemoveTheColliders) {
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(0, 0), V(0, 0), Trigger());
  ASSERT_EQ(world.CountCollider(), 1u);
  world.Clear();
  EXPECT_EQ(world.CountCollider(), 0u);
  EXPECT_EQ(world.CountBody(), 0u);
}

TEST(WorldColliderTest, WorldShouldBeMovable) {
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(0, 0), V(0, 0), Trigger());
  AddBody(world, V(1, 0), V(0, 0), Solid());
  PhysicsWorld other = std::move(world);  // the sample does `world_ = PhysicsWorld(...)`
  other.Step(Ratio(1, 1));
  EXPECT_EQ(other.CountCollider(), 2u);
  EXPECT_EQ(other.TriggerEvents().size(), 1u);
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
  AddBody(world, V(0, 0), V(0, 0), Trigger(), BodyType::kStatic);
  AddBody(world, V(5, 0), V(-1, 0), Solid());

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
  AddBody(world, V(0, 0), V(0, 0), Trigger(), BodyType::kStatic);
  AddBody(world, V(1, 0), V(0, 0), Solid());
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(TriggerEventsTest, EventsShouldBeReplacedByTheNextStep) {
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(0, 0), V(0, 0), Trigger(), BodyType::kStatic);
  AddBody(world, V(1, 0), V(0, 0), Solid());
  world.Step(kStep);
  ASSERT_EQ(world.TriggerEvents().size(), 1u);
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(TriggerEventsTest, BodyPassingThroughInOneStepShouldProduceNoEvent) {
  // Enters and leaves within a single step: contacts are only sampled once
  // per step, so nothing is seen (the "tunneling" of the exercise).
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(0, 0), V(0, 0), Trigger(), BodyType::kStatic);
  AddBody(world, V(5, 0), V(-10, 0), Solid());
  world.Step(kStep);  // x = -5
  EXPECT_TRUE(world.TriggerEvents().empty());
}

// ============================================================
// Who produces events
// ============================================================

TEST(TriggerEventsTest, BodiesWithoutColliderShouldBeIgnored) {
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(0, 0), V(0, 0), NoCollider());
  AddBody(world, V(0, 0), V(0, 0), Trigger());
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(TriggerEventsTest, TwoSolidCollidersShouldNotProduceTriggerEvents) {
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(0, 0), V(0, 0), Solid());
  AddBody(world, V(1, 0), V(0, 0), Solid());
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(TriggerEventsTest, OneTriggerIsEnoughWhicheverBodyItIs) {
  for (int trigger_first = 0; trigger_first < 2; ++trigger_first) {
    PhysicsWorld world = MakeWorld();
    AddBody(world, V(0, 0), V(0, 0), trigger_first ? Trigger() : Solid());
    AddBody(world, V(1, 0), V(0, 0), trigger_first ? Solid() : Trigger());
    world.Step(kStep);
    EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}))
        << "trigger_first = " << trigger_first;
  }
}

TEST(TriggerEventsTest, TwoTriggersShouldProduceOneEventForTheirPair) {
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(0, 0), V(0, 0), Trigger());
  AddBody(world, V(1, 0), V(0, 0), Trigger());
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
}

TEST(TriggerEventsTest, EventIdsShouldBeInIncreasingOrder) {
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(1, 0), V(0, 0), Solid());                       // id 0
  AddBody(world, V(0, 0), V(0, 0), Trigger(), BodyType::kStatic);  // id 1
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
}

// ============================================================
// Several pairs
// ============================================================

TEST(TriggerEventsTest, ManyPairsShouldBeReportedInSortedOrder) {
  PhysicsWorld world = MakeWorld();
  for (int i = 0; i < 3; ++i) {
    AddBody(world, V(i, 0), V(0, 0), Trigger());  // all overlap
  }
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)},
                                      {kEnter, Id(0), Id(2)},
                                      {kEnter, Id(1), Id(2)}}));
}

TEST(TriggerEventsTest, EnterAndExitInTheSameStepShouldBothBeReported) {
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(0, 0), V(0, 0), Trigger(), BodyType::kStatic);  // A
  const BodyId b = AddBody(world, V(1, 0), V(0, 0), Solid());      // B in
  const BodyId c = AddBody(world, V(-5, 0), V(0, 0), Solid());     // C out
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

TEST(TriggerEventsTest, UnlinkingABodyFromItsColliderShouldEmitExit) {
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(0, 0), V(0, 0), Trigger(), BodyType::kStatic);
  BodyDef def;
  def.position = V(1, 0);
  def.mass = Ratio(1, 1);
  const BodyId b = world.AddBody(def);
  CircleCollider& solid = world.CreateCollider<CircleCollider>(Ratio(1, 1), false);
  solid.AddBody(b);
  world.Step(kStep);
  ASSERT_EQ(world.TriggerEvents().size(), 1u);

  solid.RemoveBody(b);  // b has no collider anymore
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kExit, Id(0), Id(1)}}));
}

TEST(TriggerEventsTest, ClearShouldForgetThePreviousContacts) {
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(0, 0), V(0, 0), Trigger(), BodyType::kStatic);
  AddBody(world, V(1, 0), V(0, 0), Solid());
  world.Step(kStep);
  ASSERT_EQ(world.TriggerEvents().size(), 1u);

  world.Clear();
  EXPECT_TRUE(world.TriggerEvents().empty());
  AddBody(world, V(0, 0), V(0, 0), Trigger(), BodyType::kStatic);
  AddBody(world, V(1, 0), V(0, 0), Solid());
  world.Step(kStep);
  // A fresh world: the same contact is a new Enter, not a leftover.
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
}

TEST(TriggerEventsTest, ColliderOffsetShouldMoveTheContactZone) {
  PhysicsWorld world = MakeWorld();
  // Trigger body at the origin, but its circle sits at (10, 0).
  AddBody(world, V(0, 0), V(0, 0), {true, 1, true, V(10, 0)}, BodyType::kStatic);
  AddBody(world, V(0, 0), V(0, 0), Solid());  // at the origin: apart
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
  AddBody(world, V(10, 0), V(0, 0), Solid());  // on the circle
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(2)}}));
}

// ============================================================
// One collider for several bodies
// ============================================================

TEST(SharedColliderTest, EveryPairOfBodiesOfOneColliderShouldBeTested) {
  PhysicsWorld world = MakeWorld();
  CircleCollider& collider = world.CreateCollider<CircleCollider>(Ratio(1, 1), true);
  for (int i = 0; i < 3; ++i) {
    collider.AddBody(AddBody(world, V(i, 0), V(0, 0)));  // all overlap
  }
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)},
                                      {kEnter, Id(0), Id(2)},
                                      {kEnter, Id(1), Id(2)}}));
}

TEST(SharedColliderTest, OnlyTheClosePairsShouldProduceEvents) {
  PhysicsWorld world = MakeWorld();
  CircleCollider& collider = world.CreateCollider<CircleCollider>(Ratio(1, 1), true);
  collider.AddBody(AddBody(world, V(0, 0), V(0, 0)));
  collider.AddBody(AddBody(world, V(1, 0), V(0, 0)));
  collider.AddBody(AddBody(world, V(50, 0), V(0, 0)));  // far away
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
}

TEST(SharedColliderTest, SolidSharedColliderShouldProduceNoEvents) {
  PhysicsWorld world = MakeWorld();
  CircleCollider& collider = world.CreateCollider<CircleCollider>(Ratio(1, 1), false);
  collider.AddBody(AddBody(world, V(0, 0), V(0, 0)));
  collider.AddBody(AddBody(world, V(1, 0), V(0, 0)));
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(SharedColliderTest, ABodyAloneInItsColliderShouldNotCollideWithItself) {
  PhysicsWorld world = MakeWorld();
  CircleCollider& collider = world.CreateCollider<CircleCollider>(Ratio(1, 1), true);
  collider.AddBody(AddBody(world, V(0, 0), V(0, 0)));
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(SharedColliderTest, ABodyLinkedTwiceToTheSameColliderShouldNotCollideWithItself) {
  PhysicsWorld world = MakeWorld();
  CircleCollider& collider = world.CreateCollider<CircleCollider>(Ratio(1, 1), true);
  const BodyId id = AddBody(world, V(0, 0), V(0, 0));
  collider.AddBody(id);
  collider.AddBody(id);
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(SharedColliderTest, ColliderWithoutBodyShouldBeIgnored) {
  PhysicsWorld world = MakeWorld();
  world.CreateCollider<CircleCollider>(Ratio(1, 1), true);
  AddBody(world, V(0, 0), V(0, 0), Trigger());
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(SharedColliderTest, PairFoundThroughTwoCollidersShouldBeReportedOnce) {
  PhysicsWorld world = MakeWorld();
  const BodyId a = AddBody(world, V(0, 0), V(0, 0));
  const BodyId b = AddBody(world, V(1, 0), V(0, 0));
  // Both bodies carry two triggers: the same pair is found four times.
  for (int i = 0; i < 2; ++i) {
    CircleCollider& collider = world.CreateCollider<CircleCollider>(Ratio(1, 1), true);
    collider.AddBody(a);
    collider.AddBody(b);
  }
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
  world.Step(kStep);
  EXPECT_TRUE(world.TriggerEvents().empty());
}

TEST(SharedColliderTest, PairsShouldBeSortedWhateverTheOrderOfTheColliders) {
  PhysicsWorld world = MakeWorld();
  const BodyId a = AddBody(world, V(0, 0), V(0, 0));
  const BodyId b = AddBody(world, V(1, 0), V(0, 0));
  const BodyId c = AddBody(world, V(2, 0), V(0, 0));
  // Colliders created in the reverse order of the ids.
  world.CreateCollider<CircleCollider>(Ratio(1, 1), true).AddBody(c);
  world.CreateCollider<CircleCollider>(Ratio(1, 1), true).AddBody(b);
  world.CreateCollider<CircleCollider>(Ratio(1, 1), true).AddBody(a);
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)},
                                      {kEnter, Id(0), Id(2)},
                                      {kEnter, Id(1), Id(2)}}));
  // And the Exit events must find the same ordering again.
  world.GetBody(b).SetPosition(V(50, 0));
  world.Step(kStep);
  EXPECT_EQ(Events(world), (EventList{{kExit, Id(0), Id(1)},
                                      {kExit, Id(1), Id(2)}}));
}

TEST(SharedColliderTest, ColliderShouldFollowEachOfItsBodies) {
  PhysicsWorld world = MakeWorld();
  CircleCollider& collider = world.CreateCollider<CircleCollider>(Ratio(1, 1), true);
  collider.AddBody(AddBody(world, V(0, 0), V(0, 0), NoCollider(), BodyType::kStatic));
  collider.AddBody(AddBody(world, V(10, 0), V(-2, 0)));
  world.Step(kStep);  // x = 8
  world.Step(kStep);  // x = 6
  world.Step(kStep);  // x = 4
  EXPECT_TRUE(world.TriggerEvents().empty());
  world.Step(kStep);  // x = 2: touching
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
}

// ============================================================
// Stepping
// ============================================================

TEST(TriggerEventsTest, StepStayingInBoundsShouldProduceEventsToo) {
  PhysicsWorld world = MakeWorld();
  AddBody(world, V(0, 0), V(0, 0), Trigger(), BodyType::kStatic);
  AddBody(world, V(4, 0), V(-1, 0), Solid());
  world.Step(kStep, true);  // x = 3
  EXPECT_TRUE(world.TriggerEvents().empty());
  world.Step(kStep, true);  // x = 2: touching
  EXPECT_EQ(Events(world), (EventList{{kEnter, Id(0), Id(1)}}));
}

TEST(StepTest, StayingInBoundsShouldNotMoveBodiesTwice) {
  PhysicsWorld world = MakeWorld();
  const BodyId id = AddBody(world, V(0, 0), V(1, 0));
  world.Step(kStep, true);
  EXPECT_EQ(world.GetBody(id).GetPosition(), V(1, 0));  // one step = one move
}

TEST(StepTest, StayingInBoundsShouldBounceOnTheWorldEdge) {
  PhysicsWorld world = MakeWorld();  // bounds: -100..100
  const BodyId id = AddBody(world, V(99, 0), V(5, 0));
  world.Step(kStep, true);  // x = 104 > 100: the velocity is reversed
  EXPECT_EQ(world.GetBody(id).GetVelocity(), V(-5, 0));
  world.Step(kStep, true);  // comes back
  EXPECT_EQ(world.GetBody(id).GetPosition(), V(99, 0));
}

TEST(StepTest, WithoutGravityTwoBodiesAtTheSamePlaceShouldKeepTheirVelocity) {
  // G = 0 and min_distance = 0 (what the trigger sample uses): a distance of
  // zero must not be divided by (SIGFPE with a fixed point scalar, NaN with a
  // float).
  PhysicsWorld world(Scalar{}, Scalar{}, Rect{V(-100, -100), V(100, 100)}, 2);
  const BodyId a = AddBody(world, V(3, 3), V(1, 0));
  const BodyId b = AddBody(world, V(3, 3), V(0, 2));
  world.Step(kStep);
  EXPECT_EQ(world.GetBody(a).GetVelocity(), V(1, 0));
  EXPECT_EQ(world.GetBody(b).GetVelocity(), V(0, 2));
  EXPECT_EQ(world.GetBody(a).GetPosition(), V(4, 3));
  EXPECT_EQ(world.GetBody(b).GetPosition(), V(3, 5));
}

TEST(StepTest, WithoutBoundsAFastBodyShouldLeaveTheWorld) {
  PhysicsWorld world = MakeWorld();
  const BodyId id = AddBody(world, V(99, 0), V(5, 0));
  world.Step(kStep);
  world.Step(kStep);
  EXPECT_EQ(world.GetBody(id).GetVelocity(), V(5, 0));
  EXPECT_EQ(world.GetBody(id).GetPosition(), V(109, 0));
}

TEST(TriggerEventsTest, GravityShouldBringBodiesIntoContact) {
  // A heavy static trigger attracts a light body that starts apart.
  PhysicsWorld world(Ratio(1, 1), Ratio(1, 2), Rect{V(-100, -100), V(100, 100)}, 2);
  AddBody(world, V(0, 0), V(0, 0), Trigger(), BodyType::kStatic, Ratio(100, 1));
  AddBody(world, V(6, 0), V(0, 0), Solid());

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