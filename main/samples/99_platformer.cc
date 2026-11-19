#include "main/samples/99_platformer.h"

#include "gpr925/box_collider.h"
#include "gpr925/physics_material.h"
#include <iostream>

namespace gpr925 {

namespace {

// Distance between the frame lines and the edge of the world
constexpr Vec2 kFrameMargin{Ratio(1, 5), Ratio(1, 5)};

constexpr Vec2 kPlayerHalfSize{Ratio(1, 10), Ratio(18, 100)};
constexpr Vec2 kPlayerGroundCheckHalfSize{Ratio(1, 10), Ratio(20, 100)};
constexpr Scalar kGroundHalfHeight = Ratio(1, 10);

constexpr Scalar kMass = Ratio(1, 1);
// gravity is 9.81 m/s^2
constexpr Scalar kGravity = Ratio(15, 1);

// Material bounciness. Bounce is average of the two
constexpr Scalar kPlayerBounciness = Ratio(0, 1);  //.9
constexpr Scalar kPlayerFriction = Ratio(0, 1);
constexpr Scalar kGroundBounciness = Ratio(0, 1);

constexpr Scalar kPlayerSpeed = Ratio(4, 1);
constexpr Scalar kPlayerJumpForce = Ratio(25, 10);

constexpr size_t kTotalGameObjects = 16;

// Push a box in the boxes drawlist
void PushBox(DrawList& draw_list, const PhysicsWorld& world, BodyId id, ColorId color) {
  const Collider* collider = world.GetColliderWithBodyId(id);
  if (collider == nullptr || collider->GetType() != ColliderType::kBox) {
    return;
  }
  const BoxCollider& box = static_cast<const BoxCollider&>(*collider);
  const Vec2 half_size{(box.GetMax().x() - box.GetMin().x()) / Scalar{2},
                       (box.GetMax().y() - box.GetMin().y()) / Scalar{2}};
  const Vec2 center = world.GetBody(id).GetPosition() + box.GetOffset() +
                      box.GetMin() + half_size;
  draw_list.boxes.push_back({center, half_size, color});
}



}  // namespace

void Platformer::Begin(const SampleContext& context) {
  game_objects_.reserve(kTotalGameObjects);
  bounds_ = context.world_bounds;
  world_ = PhysicsWorld(Scalar{}, Scalar{}, bounds_, 3);

  const Vec2 center = bounds_.Center();
  const Scalar quarter_width = (bounds_.max_bound.x() - bounds_.min_bound.x()) / Scalar{4};
  const Scalar drop_y = bounds_.min_bound.y() + (bounds_.max_bound.y() - bounds_.min_bound.y()) / Scalar{4};

  // Some elements are the same for the ball and the box
  BodyDef def;
  def.mass = kMass;
  def.type = BodyType::kDynamic;

  BoxCollider& player_collider = world_.CreateCollider<BoxCollider>(
      -kPlayerHalfSize, kPlayerHalfSize, false, PhysicsMaterial{kPlayerBounciness, kPlayerFriction});
  def.position = {center.x() + quarter_width, drop_y};
  player_id = world_.AddBody(def);
  player_collider.AddBody(player_id);

  const Scalar ground_half_width =
      (bounds_.max_bound.x() - bounds_.min_bound.x()) / Scalar{2} -
      kFrameMargin.x();
  const Vec2 ground_half{ground_half_width, kGroundHalfHeight};
  BoxCollider& ground_collider = world_.CreateCollider<BoxCollider>(
      -ground_half, ground_half, false, PhysicsMaterial{kGroundBounciness});
  def.type = BodyType::kStatic;
  def.position = {center.x(),
                  bounds_.max_bound.y() - kFrameMargin.y() - kGroundHalfHeight};
  ground_id_ = world_.AddBody(def);
  ground_collider.AddBody(ground_id_);

  game_objects_.push_back({player_id, Tag::kPlayer, sample_color::kWhite});
  game_objects_.push_back({ground_id_, Tag::kGround, sample_color::kGreen});
}

void Platformer::FixedUpdate(const Input& input) {
  if (input.IsHeld(Button::kB)) {
    return;
  }

  Body& player = world_.GetBody(player_id);
  const Scalar mass = player.GetMass();

  Scalar target_vx{};

  if (input.IsHeld(Button::kLeft))  target_vx = -kPlayerSpeed;
  if (input.IsHeld(Button::kRight)) target_vx = kPlayerSpeed;

  const Scalar dvx = target_vx - player.GetVelocity().x();
  player.ApplyImpulse(Vec2{dvx * mass, Scalar{}});


  if (input.IsHeld(Button::kUp)) {
    player.ApplyImpulse(Vec2{Scalar{}, -kPlayerJumpForce * player.GetMass()});
  }

  player.ApplyForce(Vec2{Scalar{}, kGravity * mass});

  world_.Step(kFixedDeltaTime, true);
}

void Platformer::LateUpdate(const gpr925::Input& input) {
  if (input.IsHeld(Button::kB)) {
    return;
  }
  std::cout << "LateUpdate";
}

void Platformer::Draw(DrawList& draw_list) const {

  for (const auto& game_object : game_objects_) {
    PushBox(draw_list, world_, game_object.body_id, game_object.color);
  }

  const Vec2 top_left = bounds_.min_bound + kFrameMargin;
  const Vec2 bottom_right = bounds_.max_bound - kFrameMargin;
  const Vec2 top_right{bottom_right.x(), top_left.y()};
  const Vec2 bottom_left{top_left.x(), bottom_right.y()};
  draw_list.lines.push_back({top_left, top_right, sample_color::kGrey});
  draw_list.lines.push_back({top_right, bottom_right, sample_color::kGrey});
  draw_list.lines.push_back({bottom_right, bottom_left, sample_color::kGrey});
  draw_list.lines.push_back({bottom_left, top_left, sample_color::kGrey});
}

void Platformer::End() { world_.Clear(); }
}  // namespace gpr925