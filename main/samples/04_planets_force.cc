//
// Created by Alex King on 21.10.2026.
//


#include "main/samples/04_planets_force.h"

#include "main/samples/sample_rng.h"

namespace gpr925 {

namespace {

constexpr std::uint32_t kSeed = 0x01C0FFFEu;
// Meters per second
constexpr Scalar kMaxBallSpeed = Ratio(3, 1);
// Meters
constexpr Scalar kBallRadius = Ratio(1, 10);
constexpr maths::Vector2<Scalar> kBoxHalfSize{Ratio(1, 2), Ratio(1, 4)};
constexpr maths::Vector2<Scalar> kBoxOffset{Ratio(3, 2), Ratio(1, 1)};
// Distance between the frame lines and the edge of the world
constexpr maths::Vector2<Scalar> kFrameMargin{Ratio(1, 5), Ratio(1, 5)};

constexpr Scalar kRadiusEpsilon = Ratio(4, 5);   // 0.8 m
constexpr Scalar kGravity = Ratio(1, 1000);
constexpr Scalar kSunMass = Ratio(10000, 1); //center of the universe
constexpr Scalar kMinPlanetMass = Ratio(1, 10);
constexpr Scalar kMaxPlanetMass = Ratio(3, 10);

}  // namespace

void PlanetForceSample::Begin(const gpr925::SampleContext& context) {
  bounds_ = context.world_bounds;
  const Vec2 center = bounds_.Center();

  world_ = {kGravity, kRadiusEpsilon};

  center_id_ = world_.AddBody({center, Vec2{}, kSunMass, BodyType::kDynamic});

  SampleRng rng{kSeed};
  world_.ReserveBodyVectorSize(sample_config::kHelloBallCount);
  planet_ids_.reserve(sample_config::kHelloBallCount);
  planet_colors_.reserve(sample_config::kHelloBallCount);

  for (std::size_t i = 0; i < sample_config::kHelloBallCount; ++i) {
    const Vec2 position{rng.Range(bounds_.min_bound.x(), bounds_.max_bound.x()),
                        rng.Range(bounds_.min_bound.y(), bounds_.max_bound.y())};
    const Scalar mass = rng.Range(kMinPlanetMass, kMaxPlanetMass);
    const ColorId color = rng.RandomColor();

    const Vec2 to_center = center - position;

    const Scalar radius = to_center.Magnitude();
    const Scalar speed = maths::NumberTraits<Scalar>::Sqrt(kGravity * kSunMass / radius);
    // Perpendicular to the radius: counterclockwise
    const Vec2 tangent = Vec2{-to_center.y(), to_center.x()}.Normalize();


    planet_ids_.push_back(
        world_.AddBody({position, tangent * speed, mass, BodyType::kDynamic}));
    planet_colors_.push_back(color);
  }

}

void PlanetForceSample::FixedUpdate(const Input& input) {
  if (input.IsHeld(Button::kB)) {
    return;  // Held -> pause.
  }

  world_.Step(kFixedDeltaTime);
}
void PlanetForceSample::Draw(DrawList& draw_list) const {
  for (std::size_t i = 0; i < planet_ids_.size(); ++i) {
    draw_list.circles.push_back({world_.GetBody(planet_ids_[i]).GetPosition(),
                                 kBallRadius, planet_colors_[i]});
  }

  draw_list.circles.push_back({world_.GetBody(center_id_).GetPosition(),
                               kBallRadius, sample_color::kWhite});


  const maths::Vector2<Scalar> top_left = bounds_.min_bound + kFrameMargin;
  const maths::Vector2<Scalar> bottom_right = bounds_.max_bound - kFrameMargin;
  const maths::Vector2<Scalar> top_right{bottom_right.x(), top_left.y()};
  const maths::Vector2<Scalar> bottom_left{top_left.x(), bottom_right.y()};
  draw_list.lines.push_back({top_left, top_right, sample_color::kGrey});
  draw_list.lines.push_back({top_right, bottom_right, sample_color::kGrey});
  draw_list.lines.push_back({bottom_right, bottom_left, sample_color::kGrey});
  draw_list.lines.push_back({bottom_left, top_left, sample_color::kGrey});
}

void PlanetForceSample::End() {
  world_.Clear();
  planet_ids_.clear();
  planet_colors_.clear();
}

}  // namespace gpr925