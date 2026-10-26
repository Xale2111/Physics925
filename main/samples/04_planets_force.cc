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

inline constexpr Scalar kRadiusEpsilon = Ratio(4, 5);   // 0.8 m

}  // namespace

void PlanetForceSample::Begin(const gpr925::SampleContext& context) {
  bounds_ = context.world_bounds;
  center_ = bounds_.Center();

  center_ball_.position = center_;
  center_ball_.velocity = Vec2{};
  center_ball_.color = sample_color::kWhite;

  SampleRng rng{kSeed};
  balls_.reserve(sample_config::kHelloBallCount);
  for (std::size_t i = 0; i < sample_config::kHelloBallCount; ++i) {
    Ball ball;
    ball.position = {rng.Range(bounds_.min_bound.x(), bounds_.max_bound.x()),
                     rng.Range(bounds_.min_bound.y(), bounds_.max_bound.y())};

    // Setting the velocity of the ball directly based on the tangent to the
    // center
    const Vec2 to_center = center_ - ball.position;
    const Vec2 tangent = Vec2{-to_center.y(), to_center.x()}.Normalize();

    // direction is based on the tangent (-x then counterclockwise, -y
    // clockwise) here it's counterclockwise since the direction is defined by
    // the tangent, we only need to get a rdm speed
    const auto speed = rng.Range(kMaxBallSpeed / 4, kMaxBallSpeed);
    ball.velocity = tangent * speed;

    ball.color = rng.RandomColor();
    balls_.push_back(ball);
  }
}
/*
void PlanetSample::FixedUpdate(const Input& input) {
  if (input.IsHeld(Button::kB)) {
    return;  // Held -> pause.
  }


}*/

void PlanetForceSample::FixedUpdate(const Input& input) {
  if (input.IsHeld(Button::kB)) {
    return;  // Held -> pause.
  }

  if constexpr (kScalarIsFixed)
  {
    // To avoid narrowing with fixed point, less usage of divisions
    for (Ball& ball : balls_) {
      const Vec2 to_center = center_ - ball.position;
      const Scalar sqr_radius = to_center.SqrMagnitude();
      if (sqr_radius < kRadiusEpsilon * kRadiusEpsilon) {
        ball.position += ball.velocity * kFixedDeltaTime;
        continue;
      }

      // calculate the acceleration
      //  a = v*v/r (this is float so we to convert to vector)
      const Scalar k = ball.velocity.SqrMagnitude() / sqr_radius;
      ball.velocity += (to_center * k) * kFixedDeltaTime;
      ball.position += ball.velocity * kFixedDeltaTime;
    }
  }
  else{
    for (Ball& ball : balls_) {
      const Vec2 to_center = center_ - ball.position;
      const auto radius = to_center.Magnitude();

      //calculate the acceleration
      // a = v*v/r (this is float so we to convert to vector)
      const auto speed = ball.velocity.Magnitude();
      const auto a_length = (speed * speed) / radius;
      const Vec2 a = to_center.Normalize() * a_length;

      //setting the new velocity
      ball.velocity += a * kFixedDeltaTime;

      // changing position based on the velocity
      ball.position += ball.velocity * kFixedDeltaTime;
    }
  }
}
void PlanetForceSample::Draw(DrawList& draw_list) const {
  for (const Ball& ball : balls_) {
    draw_list.circles.push_back({ball.position, kBallRadius, ball.color});
  }

  draw_list.circles.push_back(
      {center_ball_.position, kBallRadius, center_ball_.color});

  const maths::Vector2<Scalar> top_left = bounds_.min_bound + kFrameMargin;
  const maths::Vector2<Scalar> bottom_right = bounds_.max_bound - kFrameMargin;
  const maths::Vector2<Scalar> top_right{bottom_right.x(), top_left.y()};
  const maths::Vector2<Scalar> bottom_left{top_left.x(), bottom_right.y()};
  draw_list.lines.push_back({top_left, top_right, sample_color::kGrey});
  draw_list.lines.push_back({top_right, bottom_right, sample_color::kGrey});
  draw_list.lines.push_back({bottom_right, bottom_left, sample_color::kGrey});
  draw_list.lines.push_back({bottom_left, top_left, sample_color::kGrey});
}

void PlanetForceSample::End() { balls_.clear(); }

}  // namespace gpr925