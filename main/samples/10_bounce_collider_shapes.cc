#include "main/samples/10_bounce_collider_shapes.h"

#include "gpr925/box_collider.h"
#include "gpr925/circle_collider.h"

#include "main/samples/sample_rng.h"

namespace gpr925 {

namespace {
constexpr std::uint32_t kSeed = 0x01C0FFEEu;
// Meters per second
constexpr Scalar kMaxBodySpeed = Ratio(3, 1);

// Distance between the frame lines and the edge of the world
constexpr Vec2 kFrameMargin{Ratio(1, 5), Ratio(1, 5)};

constexpr Scalar kMinBodyRadius = Ratio(1, 10);
constexpr Scalar kMaxBodyRadius = Ratio(3, 10);
constexpr Scalar kMinBoxHalfSize{Ratio(1, 20)};
constexpr Scalar kMaxBoxHalfSize{Ratio(5, 20)};

constexpr Scalar kMinBounciness = Ratio(0, 1);
constexpr Scalar kMaxBounciness = Ratio(1, 1);

// The speed is based on the mass, so the higher the mass, the faster the ball
// will be
constexpr Scalar kMinPlanetMass = Ratio(1, 1);
constexpr Scalar kMaxPlanetMass = Ratio(3, 1);

}  // namespace

void BounceColliderShapes::Begin(const SampleContext& context) {
  bounds_ = context.world_bounds;

  world_ = PhysicsWorld(Scalar{}, Scalar{}, bounds_, sample_config::kBouncingElements);
  bodies_ids_.reserve(sample_config::kBouncingElements);
  bodies_colors_.reserve(sample_config::kBouncingElements);

  SampleRng rng{kSeed};

  for (std::size_t i = 0; i < sample_config::kBouncingElements/2; ++i) {
    CircleCollider& collider = world_.CreateCollider<CircleCollider>(rng.Range(kMinBodyRadius, kMaxBodyRadius), false, PhysicsMaterial{Scalar{rng.Range(kMinBounciness, kMaxBounciness)}});
    BodyDef def;
    def.position = {rng.Range(bounds_.min_bound.x(), bounds_.max_bound.x()),
                    rng.Range(bounds_.min_bound.y(), bounds_.max_bound.y())};
    def.mass = rng.Range(kMinPlanetMass, kMaxPlanetMass);

    Vec2 direction{rng.Range(-kMaxBodySpeed, kMaxBodySpeed),
                   rng.Range(-kMaxBodySpeed, kMaxBodySpeed)};
    def.velocity = direction.Normalize() * def.mass;

    def.type = BodyType::kDynamic;

    bodies_ids_.push_back(world_.AddBody(def));
    collider.AddBody(bodies_ids_.back());

    bodies_colors_.push_back(sample_color::kRed);
  }

  for (std::size_t i = 0; i < sample_config::kBouncingElements/2; ++i) {
    const Scalar kBoxHalfSize = rng.Range(kMinBoxHalfSize, kMaxBoxHalfSize);
    BoxCollider& collider = world_.CreateCollider<BoxCollider>(Vec2{-kBoxHalfSize,-kBoxHalfSize}, Vec2{kBoxHalfSize,kBoxHalfSize}, false,PhysicsMaterial{Scalar{rng.Range(kMinBounciness, kMaxBounciness)}});
    BodyDef def;
    def.position = {rng.Range(bounds_.min_bound.x(), bounds_.max_bound.x()),
                    rng.Range(bounds_.min_bound.y(), bounds_.max_bound.y())};
    def.mass = rng.Range(kMinPlanetMass, kMaxPlanetMass);

    Vec2 direction{rng.Range(-kMaxBodySpeed, kMaxBodySpeed),
                   rng.Range(-kMaxBodySpeed, kMaxBodySpeed)};
    def.velocity = direction.Normalize() * def.mass;

    def.type = BodyType::kDynamic;

    bodies_ids_.push_back(world_.AddBody(def));
    collider.AddBody(bodies_ids_.back());

    bodies_colors_.push_back(sample_color::kRed);
  }
}

void BounceColliderShapes::FixedUpdate(const Input& input) {
  if (input.IsHeld(Button::kB)) {
    return;
  }
  world_.Step(kFixedDeltaTime, true);
}

void BounceColliderShapes::Draw(DrawList& draw_list) const {
  for (std::size_t i = 0; i < bodies_ids_.size(); ++i) {
    const Body& body = world_.GetBody(bodies_ids_[i]);
    const Collider* collider = world_.GetColliderWithBodyId(bodies_ids_[i]);
    if (world_.GetColliderTypeWithBodyId(bodies_ids_[i]).has_value())
    {
      if (world_.GetColliderTypeWithBodyId(bodies_ids_[i]) == ColliderType::kCircle) {
        const Scalar radius = static_cast<const CircleCollider&>(*collider).GetRadius();
        draw_list.circles.push_back({body.GetPosition(), radius, bodies_colors_[i]});
      }
      else {
        const BoxCollider& box = static_cast<const BoxCollider&>(*collider);
        const Vec2 half_size{(box.GetMax().x() - box.GetMin().x()) / Scalar{2},(box.GetMax().y() - box.GetMin().y()) / Scalar{2}};
        const Vec2 center = body.GetPosition() + box.GetOffset() + box.GetMin() + half_size;

        draw_list.boxes.push_back({center, half_size, bodies_colors_[i]});
      }
    }
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

void BounceColliderShapes::End() {
  world_.Clear();
  bodies_ids_.clear();
  bodies_colors_.clear();
}

}  // namespace gpr925