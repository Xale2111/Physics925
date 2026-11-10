#include "main/samples/11_ground_bounce.h"

#include "gpr925/box_collider.h"
#include "gpr925/circle_collider.h"
#include "gpr925/physics_material.h"

namespace gpr925 {

namespace {

// Distance between the frame lines and the edge of the world
constexpr Vec2 kFrameMargin{Ratio(1, 5), Ratio(1, 5)};

constexpr Scalar kBallRadius = Ratio(1, 2);
constexpr Vec2 kBoxHalfSize{Ratio(3, 10), Ratio(3, 10)};
constexpr Scalar kGroundHalfHeight = Ratio(1, 10);

constexpr Scalar kMass = Ratio(1, 1);
// gravity is 9.81 m/s^2
constexpr Scalar kGravity = Ratio(981, 100);

//Material bounciness. Bounce is average of the two
constexpr Scalar kBallBounciness = Ratio(1, 2);
constexpr Scalar kBoxBounciness = Ratio(1, 1);
constexpr Scalar kGroundBounciness = Ratio(1, 1);

// Push a box in the boxes drawlist
void PushBox(DrawList& draw_list, const PhysicsWorld& world, BodyId id,
             ColorId color) {
  const Collider* collider = world.GetColliderWithBodyId(id);
  if (collider == nullptr || collider->GetType() != ColliderType::kBox) {
    return;
  }
  const BoxCollider& box = static_cast<const BoxCollider&>(*collider);
  const Vec2 half_size{(box.GetMax().x() - box.GetMin().x()) / Scalar{2},(box.GetMax().y() - box.GetMin().y()) / Scalar{2}};
  const Vec2 center = world.GetBody(id).GetPosition() + box.GetOffset() + box.GetMin() + half_size;
  draw_list.boxes.push_back({center, half_size, color});
}

}  // namespace

void GroundBounce::Begin(const SampleContext& context) {
  bounds_ = context.world_bounds;
  world_ = PhysicsWorld(Scalar{}, Scalar{}, bounds_, 3);

  const Vec2 center = bounds_.Center();
  const Scalar quarter_width = (bounds_.max_bound.x() - bounds_.min_bound.x()) / Scalar{4};
  const Scalar drop_y = bounds_.min_bound.y() + (bounds_.max_bound.y() - bounds_.min_bound.y()) / Scalar{4};

  // Some elements are the same for the ball and the box
  BodyDef def;
  def.mass = kMass;
  def.type = BodyType::kDynamic;

  CircleCollider& ball_collider = world_.CreateCollider<CircleCollider>(kBallRadius, false, PhysicsMaterial{kBallBounciness});
  def.position = {center.x() - quarter_width, drop_y};
  ball_id_ = world_.AddBody(def);
  ball_collider.AddBody(ball_id_);

  BoxCollider& box_collider = world_.CreateCollider<BoxCollider>(-kBoxHalfSize, kBoxHalfSize, false, PhysicsMaterial{kBoxBounciness});
  def.position = {center.x() + quarter_width, drop_y};
  box_id_ = world_.AddBody(def);
  box_collider.AddBody(box_id_);

  const Scalar ground_half_width = (bounds_.max_bound.x() - bounds_.min_bound.x()) / Scalar{2} - kFrameMargin.x();
  const Vec2 ground_half{ground_half_width, kGroundHalfHeight};
  BoxCollider& ground_collider = world_.CreateCollider<BoxCollider>(-ground_half, ground_half, false, PhysicsMaterial{kGroundBounciness});
  def.type = BodyType::kStatic;
  def.position = {center.x(),bounds_.max_bound.y() - kFrameMargin.y() - kGroundHalfHeight};
  ground_id_ = world_.AddBody(def);
  ground_collider.AddBody(ground_id_);
}

void GroundBounce::FixedUpdate(const Input& input) {
  if (input.IsHeld(Button::kB)) {
    return;
  }

  for (const BodyId id : {ball_id_, box_id_}) {
    Body& body = world_.GetBody(id);
    body.ApplyForce(Vec2{Scalar{}, kGravity * body.GetMass()});
  }
  world_.Step(kFixedDeltaTime, true);
}

void GroundBounce::Draw(DrawList& draw_list) const {
  const Body& ball = world_.GetBody(ball_id_);
  draw_list.circles.push_back({ball.GetPosition(), kBallRadius, sample_color::kWhite});
  PushBox(draw_list, world_, box_id_, sample_color::kWhite);
  PushBox(draw_list, world_, ground_id_, sample_color::kGreen);

  const Vec2 top_left = bounds_.min_bound + kFrameMargin;
  const Vec2 bottom_right = bounds_.max_bound - kFrameMargin;
  const Vec2 top_right{bottom_right.x(), top_left.y()};
  const Vec2 bottom_left{top_left.x(), bottom_right.y()};
  draw_list.lines.push_back({top_left, top_right, sample_color::kGrey});
  draw_list.lines.push_back({top_right, bottom_right, sample_color::kGrey});
  draw_list.lines.push_back({bottom_right, bottom_left, sample_color::kGrey});
  draw_list.lines.push_back({bottom_left, top_left, sample_color::kGrey});
}

void GroundBounce::End() { world_.Clear(); }

}  // namespace gpr925