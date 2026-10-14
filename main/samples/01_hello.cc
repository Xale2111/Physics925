#include "main/samples/01_hello.h"

#include "main/samples/sample_rng.h"

namespace gpr925 {

namespace {

constexpr std::uint32_t kSeed = 0x01C0FFEEu;
// Meters per second
constexpr Scalar kMaxBallSpeed = Ratio(3, 1);
constexpr Scalar kPlayerSpeed = Ratio(4, 1);
// Meters
constexpr Scalar kBallRadius = Ratio(1, 10);
constexpr Scalar kPlayerRadius = Ratio(3, 10);
constexpr Vec2 kBoxHalfSize{Ratio(1, 2), Ratio(1, 4)};
constexpr Vec2 kBoxOffset{Ratio(3, 2), Ratio(1, 1)};
// Distance between the frame lines and the edge of the world
constexpr Vec2 kFrameMargin{Ratio(1, 5), Ratio(1, 5)};

// EngineLib's Vector2 has no vector * scalar product, only the
// component-wise one, so a scale is a vector with twice the same value
constexpr Vec2 Scale(Vec2 v, Scalar s) { return v * Vec2{s, s}; }

// Reverses the velocity components that take a point further out of bounds. A
// component is flipped only when the point is outside on that axis and still
// moving away, so it doesn't jitter on the border.
void BounceInside(Vec2 position, Vec2& velocity, const Rect& bounds) {
  if ((velocity.x > Scalar{} && position.x > bounds.max_bound.x) ||
      (velocity.x < Scalar{} && position.x < bounds.min_bound.x)) {
    velocity.x = -velocity.x;
  }
  if ((velocity.y > Scalar{} && position.y > bounds.max_bound.y) ||
      (velocity.y < Scalar{} && position.y < bounds.min_bound.y)) {
    velocity.y = -velocity.y;
  }
}

constexpr Scalar Clamp(Scalar value, Scalar low, Scalar high) {
  return value < low ? low : (value > high ? high : value);
}

}  // namespace

void HelloSample::Begin(const SampleContext& context) {
  bounds_ = context.world_bounds;
  player_position_ = bounds_.Center();
  player_color_ = sample_color::kWhite;

  SampleRng rng{kSeed};
  balls_.reserve(sample_config::kHelloBallCount);
  for (std::size_t i = 0; i < sample_config::kHelloBallCount; ++i) {
    Ball ball;
    ball.position = {rng.Range(bounds_.min_bound.x, bounds_.max_bound.x),
                     rng.Range(bounds_.min_bound.y, bounds_.max_bound.y)};
    ball.velocity = {rng.Range(-kMaxBallSpeed, kMaxBallSpeed),
                     rng.Range(-kMaxBallSpeed, kMaxBallSpeed)};
    ball.color = rng.RandomColor();
    balls_.push_back(ball);
  }
}

void HelloSample::FixedUpdate(const Input& input) {
  if (!input.IsHeld(Button::kB)) {
    for (Ball& ball : balls_) {
      ball.position += Scale(ball.velocity, kFixedDeltaTime);
      BounceInside(ball.position, ball.velocity, bounds_);
    }
  }

  Vec2 direction{};
  if (input.IsHeld(Button::kLeft)) {
    direction.x -= ToScalar(1);
  }
  if (input.IsHeld(Button::kRight)) {
    direction.x += ToScalar(1);
  }
  if (input.IsHeld(Button::kUp)) {
    direction.y -= ToScalar(1);
  }
  if (input.IsHeld(Button::kDown)) {
    direction.y += ToScalar(1);
  }
  player_position_ += Scale(direction, kPlayerSpeed * kFixedDeltaTime);
  // Keep the whole circle on screen
  player_position_.x =
      Clamp(player_position_.x, bounds_.min_bound.x + kPlayerRadius,
            bounds_.max_bound.x - kPlayerRadius);
  player_position_.y =
      Clamp(player_position_.y, bounds_.min_bound.y + kPlayerRadius,
            bounds_.max_bound.y - kPlayerRadius);

  if (input.WasPressed(Button::kA)) {
    player_color_ = player_color_ == sample_color::kLastBright
                        ? sample_color::kWhite
                        : static_cast<ColorId>(player_color_ + 1);
  }
}

void HelloSample::Draw(DrawList& draw_list) const {
  for (const Ball& ball : balls_) {
    draw_list.circles.push_back({ball.position, kBallRadius, ball.color});
  }
  draw_list.circles.push_back({player_position_, kPlayerRadius, player_color_});

  draw_list.boxes.push_back(
      {bounds_.min_bound + kBoxOffset, kBoxHalfSize, sample_color::kTeal});

  const Vec2 top_left = bounds_.min_bound + kFrameMargin;
  const Vec2 bottom_right = bounds_.max_bound - kFrameMargin;
  const Vec2 top_right{bottom_right.x, top_left.y};
  const Vec2 bottom_left{top_left.x, bottom_right.y};
  draw_list.lines.push_back({top_left, top_right, sample_color::kGrey});
  draw_list.lines.push_back({top_right, bottom_right, sample_color::kGrey});
  draw_list.lines.push_back({bottom_right, bottom_left, sample_color::kGrey});
  draw_list.lines.push_back({bottom_left, top_left, sample_color::kGrey});
}

void HelloSample::End() { balls_.clear(); }

}  // namespace gpr925
