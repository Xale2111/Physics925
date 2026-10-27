//
// Created by alexk on 25.10.2026.
//

#include "main/samples/03_planets_aosoa.h"

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

constexpr Scalar kRadiusEpsilon = Ratio(4, 5);  // 0.8 m

// using size_t for the type since we will be looping with it
constexpr size_t kFour = 4;

// issue with kHelloBallCount % 4 != 0? where does the remainder go?
// since kHelloBallCount % 4 can't be >3, add 3?
// Should this include the + 3?
//adding +1 here to manager the last (possibly not complete) group of 4
constexpr size_t kBallsDividedByFour = sample_config::kHelloBallCount / kFour +1;

}  // namespace

void PlanetAoSoaSample::Begin(const gpr925::SampleContext& context) {
  bounds_ = context.world_bounds;
  center_ = bounds_.Center();

  center_ball_.position = center_;
  center_ball_.velocity = Vec2{};
  center_ball_.color = sample_color::kWhite;

  balls_positions_.assign(kBallsDividedByFour,
                          maths::FourVector2<Scalar>{});
  balls_velocities_.assign(kBallsDividedByFour,
                           maths::FourVector2<Scalar>{});
  balls_colors_.assign(kBallsDividedByFour, sample_color::kWhite);

  SampleRng rng{kSeed};
  // First loop to go through all the fourVector
  // Second loop to go through all the balls inside a fourVector
  for (std::size_t i = 0; i < kBallsDividedByFour; ++i) {
    for (std::size_t j = 0; j < kFour; ++j) {
      const Vec2 position = {
          rng.Range(bounds_.min_bound.x(), bounds_.max_bound.x()),
          rng.Range(bounds_.min_bound.y(), bounds_.max_bound.y())};

      const Vec2 to_center = center_ - position;
      const Vec2 tangent = Vec2{-to_center.y(), to_center.x()}.Normalize();
      const Scalar speed = rng.Range(kMaxBallSpeed / 4, kMaxBallSpeed);
      const Vec2 velocity = tangent * speed;

      const ColorId color = rng.RandomColor();

      balls_positions_[i].Xs()[j] = position.x();
      balls_positions_[i].Ys()[j] = position.y();

      balls_velocities_[i].Xs()[j] = velocity.x();
      balls_velocities_[i].Ys()[j] = velocity.y();

      balls_colors_[i] = color;
    }
  }
}

void PlanetAoSoaSample::FixedUpdate(const Input& input) {
  if (input.IsHeld(Button::kB)) {
    return;  // Held -> pause.
  }

  if constexpr (kScalarIsFixed) {
    // To avoid narrowing with fixed point, less usage of divisions
    for (std::size_t i = 0; i < kBallsDividedByFour; ++i) {
      for (std::size_t j = 0; j < kFour; ++j) {
        Vec2 ballPosition = {balls_positions_[i].Xs()[j],
                             balls_positions_[i].Ys()[j]};
        const Vec2 to_center = center_ - ballPosition;
        const Scalar sqr_radius = to_center.SqrMagnitude();
        if (sqr_radius < kRadiusEpsilon * kRadiusEpsilon) {
          balls_positions_[i].Xs()[j] +=
              balls_velocities_[i].Xs()[j] * kFixedDeltaTime;
          balls_positions_[i].Ys()[j] +=
              balls_velocities_[i].Ys()[j] * kFixedDeltaTime;
          continue;
        }

        // calculate the acceleration
        //  a = v*v/r (this is float so we to convert to vector)
        Vec2 ballVelocity = {balls_velocities_[i].Xs()[j],
                             balls_velocities_[i].Ys()[j]};
        const Scalar k = ballVelocity.SqrMagnitude() / sqr_radius;

        balls_velocities_[i].Xs()[j] += (to_center * k).x() * kFixedDeltaTime;
        balls_velocities_[i].Ys()[j] += (to_center * k).y() * kFixedDeltaTime;

        balls_positions_[i].Xs()[j] +=
            balls_velocities_[i].Xs()[j] * kFixedDeltaTime;
        balls_positions_[i].Ys()[j] +=
            balls_velocities_[i].Ys()[j] * kFixedDeltaTime;
      }
    }
  } else {
    for (std::size_t i = 0; i < kBallsDividedByFour; ++i) {
      for (std::size_t j = 0; j < kFour; ++j) {
        Vec2 ballPosition = {balls_positions_[i].Xs()[j],
                             balls_positions_[i].Ys()[j]};
        const Vec2 to_center = center_ - ballPosition;
        const auto radius = to_center.Magnitude();

        // calculate the acceleration
        //  a = v*v/r (this is float so we to convert to vector)
        Vec2 ballVelocity = {balls_velocities_[i].Xs()[j],
                             balls_velocities_[i].Ys()[j]};

        const auto a_length = ballVelocity.SqrMagnitude() / radius;
        const Vec2 a = to_center.Normalize() * a_length;

        // setting the new velocity
        balls_velocities_[i].Xs()[j] += a.x() * kFixedDeltaTime;
        balls_velocities_[i].Ys()[j] += a.y() * kFixedDeltaTime;

        // changing position based on the velocity
        balls_positions_[i].Xs()[j] +=
            balls_velocities_[i].Xs()[j] * kFixedDeltaTime;
        balls_positions_[i].Ys()[j] +=
            balls_velocities_[i].Ys()[j] * kFixedDeltaTime;
      }
    }
  }
}

void PlanetAoSoaSample::Draw(DrawList& draw_list) const {
  for (std::size_t i = 0; i < kBallsDividedByFour; ++i) {
    for (std::size_t j = 0; j < kFour; ++j) {
      Vec2 ballPosition = {balls_positions_[i].Xs()[j], balls_positions_[i].Ys()[j]};
      draw_list.circles.push_back(
          {ballPosition,
           kBallRadius,
           balls_colors_[i]});
    }
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

void PlanetAoSoaSample::End() {
  balls_positions_.clear();
  balls_velocities_.clear();
  balls_colors_.clear();
}

}  // namespace gpr925
