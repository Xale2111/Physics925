//
// Created by alexk on 14.10.2026.
//

#include "main/samples/02_planets.h"
#include "main/samples/sample_rng.h"

namespace gpr925 {

namespace {

constexpr std::uint32_t kSeed = 0x01C0FFEEu;
// Meters per second
constexpr Scalar kMaxBallSpeed = Ratio(3, 1);
// Meters
constexpr Scalar kBallRadius = Ratio(1, 10);
constexpr maths::Vector2<Scalar> kBoxHalfSize{Ratio(1, 2), Ratio(1, 4)};
constexpr maths::Vector2<Scalar> kBoxOffset{Ratio(3, 2), Ratio(1, 1)};
// Distance between the frame lines and the edge of the world
constexpr maths::Vector2<Scalar> kFrameMargin{Ratio(1, 5), Ratio(1, 5)};

} //namespace

void PlanetSample::Begin(const gpr925::SampleContext& context) {
  bounds_ = context.world_bounds;
  center_ = bounds_.Center();

  SampleRng rng{kSeed};
  balls_.reserve(sample_config::kHelloBallCount);
  for (std::size_t i = 0; i < sample_config::kHelloBallCount; ++i) {
    Ball ball;
    ball.position = {rng.Range(bounds_.min_bound.x(), bounds_.max_bound.x()),
                     rng.Range(bounds_.min_bound.y(), bounds_.max_bound.y())};
    ball.velocity = {rng.Range(-kMaxBallSpeed, kMaxBallSpeed),
                     rng.Range(-kMaxBallSpeed, kMaxBallSpeed)};
    ball.color = rng.RandomColor();
    balls_.push_back(ball);
  }
}

void PlanetSample::FixedUpdate(const Input& input) {
  if (!input.IsHeld(Button::kB)) {
   //Not held -> continue. Held -> pause.
  }

  //Center of world bounds is the center of the galaxie
  //Each planets needs to to a mcu based on the center
  // a = (v*v) / r
  // vector a is from current position to center
  // vector v is tangent to the circle (radius)

}

void PlanetSample::Draw(DrawList& draw_list) const {
  for (const Ball& ball : balls_) {
    draw_list.circles.push_back({ball.position, kBallRadius, ball.color});
  }

  const maths::Vector2<Scalar> top_left = bounds_.min_bound + kFrameMargin;
  const maths::Vector2<Scalar> bottom_right = bounds_.max_bound - kFrameMargin;
  const maths::Vector2<Scalar> top_right{bottom_right.x(), top_left.y()};
  const maths::Vector2<Scalar> bottom_left{top_left.x(), bottom_right.y()};
  draw_list.lines.push_back({top_left, top_right, sample_color::kGrey});
  draw_list.lines.push_back({top_right, bottom_right, sample_color::kGrey});
  draw_list.lines.push_back({bottom_right, bottom_left, sample_color::kGrey});
  draw_list.lines.push_back({bottom_left, top_left, sample_color::kGrey});
}

void PlanetSample::End() { balls_.clear(); }

}  // namespace gpr925