#include "main/samples/05_trigger_collider.h"

#include "main/samples/sample_rng.h"

namespace gpr925 {

namespace {

constexpr std::uint32_t kSeed = 0x01C0FFEEu;
// Meters per second
constexpr Scalar kMaxBodySpeed = Ratio(3, 1);
// Meters
constexpr Scalar kBodyRadius = Ratio(1, 10);
constexpr Vec2 kBoxHalfSize{Ratio(1, 2), Ratio(1, 4)};
constexpr Vec2 kBoxOffset{Ratio(3, 2), Ratio(1, 1)};
// Distance between the frame lines and the edge of the world
constexpr Vec2 kFrameMargin{Ratio(1, 5), Ratio(1, 5)};

//The speed is based on the mass, so the higher the mass, the faster the ball will be
constexpr Scalar kMinPlanetMass = Ratio(1, 1);
constexpr Scalar kMaxPlanetMass = Ratio(3, 1);

}  // namespace

void TriggerCollider::Begin(const SampleContext& context) {
  bounds_ = context.world_bounds;

  world_ = PhysicsWorld(Scalar{}, Scalar{}, bounds_,sample_config::kHelloBallCount);
  bodies_ids_.reserve(sample_config::kHelloBallCount);
  bodies_colors_.reserve(sample_config::kHelloBallCount);
  overlap_counts_.reserve(sample_config::kHelloBallCount);

  SampleRng rng{kSeed};

  for (std::size_t i = 0; i < sample_config::kHelloBallCount; ++i) {
    BodyDef def;
    def.position = {rng.Range(bounds_.min_bound.x(), bounds_.max_bound.x()),
                        rng.Range(bounds_.min_bound.y(), bounds_.max_bound.y())};
    def.mass = rng.Range(kMinPlanetMass, kMaxPlanetMass);

    Vec2 direction{rng.Range(-kMaxBodySpeed, kMaxBodySpeed),
                   rng.Range(-kMaxBodySpeed, kMaxBodySpeed)};
    def.velocity = direction.Normalize() * def.mass;

    def.collider = MakeCircleCollider(kBodyRadius, true);

    bodies_ids_.push_back(world_.AddBody(def));
    overlap_counts_.push_back(0);
    bodies_colors_.push_back(sample_color::kGreen);
  }
}

void TriggerCollider::FixedUpdate(const Input& input) {
  if (input.IsHeld(Button::kB)) {
    return;
  }
  world_.StepFreeRoam(kFixedDeltaTime);

  for (const TriggerEvent& event : world_.TriggerEvents()) {
    const int change = event.type == TriggerEventType::kEnter ? 1 : -1;
    overlap_counts_[static_cast<std::size_t>(event.a)] += change;
    overlap_counts_[static_cast<std::size_t>(event.b)] += change;
  }

}

void TriggerCollider::Draw(DrawList& draw_list) const {
  for (std::size_t i = 0; i < bodies_ids_.size(); ++i) {
    const Body& body = world_.GetBody(bodies_ids_[i]);
    const Scalar radius = std::get<CircleShape>(body.GetCollider()->shape).radius; //currently assuming the shpe is a circle
    const ColorId color = overlap_counts_[i] > 0 ? sample_color::kRed : bodies_colors_[i]; //is overlapping ? yes-> red, no -> green
    draw_list.circles.push_back({body.GetPosition(), radius, color});
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

void TriggerCollider::End() {
  world_.Clear();
  bodies_ids_.clear();
  bodies_colors_.clear();
  overlap_counts_.clear();
}

}  // namespace gpr925
