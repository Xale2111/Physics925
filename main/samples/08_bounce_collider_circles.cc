#include "main/samples/08_bounce_collider_circles.h"

#include "gpr925/circle_collider.h"
#include "gpr925/physics_material.h"

#include "main/samples/sample_rng.h"

namespace gpr925 {

namespace {
constexpr std::uint32_t kSeed = 0x01C0FFEEu;
// Meters per second
constexpr Scalar kMaxBodySpeed = Ratio(3, 1);

// Distance between the frame lines and the edge of the world
constexpr Vec2 kFrameMargin{Ratio(1, 5), Ratio(1, 5)};

constexpr Scalar kSizeRatio = Ratio(1, 10);

constexpr Scalar kMinBounciness = Ratio(0, 1);
constexpr Scalar kMaxBounciness = Ratio(1, 1);

// The speed is based on the mass, so the higher the mass, the faster the ball
// will be
constexpr Scalar kMinPlanetMass = Ratio(1, 1);
constexpr Scalar kMaxPlanetMass = Ratio(3, 1);

}  // namespace

void BounceColliderCircles::Begin(const SampleContext& context) {
  bounds_ = context.world_bounds;

  world_ = PhysicsWorld(Scalar{}, Scalar{}, bounds_,
                        sample_config::kBouncingElements);
  bodies_ids_.reserve(sample_config::kBouncingElements);
  bodies_colors_.reserve(sample_config::kBouncingElements);

  SampleRng rng{kSeed};

  // it; we only keep a reference to link the bodies while we create them.
  for (std::size_t i = 0; i < sample_config::kBouncingElements; ++i) {
    Scalar mass = rng.Range(kMinPlanetMass, kMaxPlanetMass);
    CircleCollider& collider = world_.CreateCollider<CircleCollider>(
        mass * kSizeRatio, false,
        PhysicsMaterial{Scalar{rng.Range(kMinBounciness, kMaxBounciness)}});
    BodyDef def;
    def.position = {rng.Range(bounds_.min_bound.x(), bounds_.max_bound.x()),
                    rng.Range(bounds_.min_bound.y(), bounds_.max_bound.y())};
    def.mass = mass;

    Vec2 direction{rng.Range(-kMaxBodySpeed, kMaxBodySpeed),
                   rng.Range(-kMaxBodySpeed, kMaxBodySpeed)};
    def.velocity = direction.Normalize() * def.mass;

    def.type = BodyType::kDynamic;

    bodies_ids_.push_back(world_.AddBody(def));
    collider.AddBody(bodies_ids_.back());

    bodies_colors_.push_back(sample_color::kRed);
  }
}

void BounceColliderCircles::FixedUpdate(const Input& input) {
  if (input.IsHeld(Button::kB)) {
    return;
  }
  world_.Step(kFixedDeltaTime, true);
}

void BounceColliderCircles::Draw(DrawList& draw_list) const {
  for (std::size_t i = 0; i < bodies_ids_.size(); ++i) {
    const Body& body = world_.GetBody(bodies_ids_[i]);
    const Collider* collider = world_.GetColliderWithBodyId(bodies_ids_[i]);
    const Scalar radius =
        static_cast<const CircleCollider&>(*collider).GetRadius();
    draw_list.circles.push_back(
        {body.GetPosition(), radius, bodies_colors_[i]});
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

void BounceColliderCircles::End() {
  world_.Clear();
  bodies_ids_.clear();
  bodies_colors_.clear();
}

}  // namespace gpr925