#include "main/samples/12_rolling_circle.h"

#include "gpr925/box_collider.h"
#include "gpr925/circle_collider.h"
#include "gpr925/physics_material.h"
#include "gpr925/inertia.h"
#include "gpr925//trigonometry.h"

namespace gpr925 {

namespace {

// Distance between the frame lines and the edge of the world
constexpr Vec2 kFrameMargin{Ratio(1, 5), Ratio(1, 5)};

constexpr Scalar kBallRadius = Ratio(1, 2);
constexpr Scalar kGroundHalfHeight = Ratio(1, 10);

constexpr Scalar kGroundAngle = kPi / Scalar{8};

constexpr Scalar kMass = Ratio(1, 1);
// gravity is 9.81 m/s^2
constexpr Scalar kGravity = Ratio(981, 100);

//Material bounciness. Bounce is average of the two
constexpr Scalar kBallBounciness = Ratio(0, 1);
constexpr Scalar kGroundBounciness = Ratio(0, 1);
constexpr Scalar kBallFriction = Ratio(1, 2);
constexpr Scalar kGroundFriction = Ratio(1, 2);

// Push a box in the boxes drawlist
void PushOrientedBox(DrawList& draw_list, const PhysicsWorld& world, BodyId id, ColorId color) {
  const Collider* collider = world.GetColliderWithBodyId(id);
  if (collider == nullptr ||
      collider->GetType() != ColliderType::kOrientedBox) {
    return;
  }
  const OrientedBoxCollider& box =
      static_cast<const OrientedBoxCollider&>(*collider);
  const Body& body = world.GetBody(id);
  const std::array<Vec2, 4> corners =
      box.GetCorners(body.GetPosition(), body.GetAngle());
  for (std::size_t i = 0; i < corners.size(); ++i) {
    draw_list.lines.push_back(
        {corners[i], corners[(i + 1) % corners.size()], color});
  }
}

}  // namespace

void RollingCircle::Begin(const SampleContext& context) {
  bounds_ = context.world_bounds;
  world_ = PhysicsWorld(Scalar{}, Scalar{}, bounds_, 3);

  const Vec2 center = bounds_.Center();
  const Scalar drop_y = bounds_.min_bound.y() + (bounds_.max_bound.y() - bounds_.min_bound.y()) / Scalar{4};
  const Scalar world_width = bounds_.max_bound.x() - bounds_.min_bound.x();

  // Some elements are the same for the ball and the box
  BodyDef def;
  def.mass = kMass;
  def.type = BodyType::kDynamic;
  def.inertia = DiscInertia(kMass, kBallRadius);

  CircleCollider& ball_collider = world_.CreateCollider<CircleCollider>(kBallRadius, false, PhysicsMaterial{kBallBounciness, kBallFriction});
  def.position = {center.x(), drop_y};
  ball_id_ = world_.AddBody(def);
  ball_collider.AddBody(ball_id_);

  const Scalar ground_half_width = world_width / Scalar{2};
  const Vec2 ground_half{ground_half_width, kGroundHalfHeight};
  OrientedBoxCollider& ground_collider = world_.CreateCollider<OrientedBoxCollider>(
      -ground_half, ground_half, false, PhysicsMaterial{kGroundBounciness, kGroundFriction});
  BodyDef ground_def;
  ground_def.type = BodyType::kStatic;
  ground_def.mass = kMass;
  ground_def.angle = kGroundAngle;
  ground_def.position = center;
  ground_id_ = world_.AddBody(ground_def);
  ground_collider.AddBody(ground_id_);
}

void RollingCircle::FixedUpdate(const Input& input) {
  if (input.IsHeld(Button::kB)) {
    return;
  }

  Body& body = world_.GetBody(ball_id_);
  body.ApplyForce(Vec2{Scalar{}, kGravity * body.GetMass()});

  world_.Step(kFixedDeltaTime, false);
}

void RollingCircle::Draw(DrawList& draw_list) const {
  const Body& ball = world_.GetBody(ball_id_);
  draw_list.circles.push_back({ball.GetPosition(), kBallRadius, sample_color::kWhite});
  const Vec2 spoke = Rotate(Vec2{kBallRadius, Scalar{}}, ball.GetAngle());
  draw_list.lines.push_back({ball.GetPosition(), ball.GetPosition() + spoke, sample_color::kRed});
  PushOrientedBox(draw_list, world_, ground_id_, sample_color::kGreen);

  const Vec2 top_left = bounds_.min_bound + kFrameMargin;
  const Vec2 bottom_right = bounds_.max_bound - kFrameMargin;
  const Vec2 top_right{bottom_right.x(), top_left.y()};
  const Vec2 bottom_left{top_left.x(), bottom_right.y()};
  draw_list.lines.push_back({top_left, top_right, sample_color::kGrey});
  draw_list.lines.push_back({top_right, bottom_right, sample_color::kGrey});
  draw_list.lines.push_back({bottom_right, bottom_left, sample_color::kGrey});
  draw_list.lines.push_back({bottom_left, top_left, sample_color::kGrey});
}

void RollingCircle::End() { world_.Clear(); }

}  // namespace gpr925