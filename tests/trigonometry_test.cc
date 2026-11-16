//
// Created by alexk on 16.11.2026.
//

#include <gtest/gtest.h>

#include <cmath>

#include "gpr925/trigonometry.h"

namespace gpr925 {
namespace {

// Fixed 20.12 has a step of 1/4096: ~2.4e-4. A few steps of error are normal.
constexpr float kTolerance = 2e-3F;

float Error(float got, float expected) { return std::fabs(got - expected); }

TEST(Trigonometry, KnownValues) {
  EXPECT_TRUE(Error(ToFloat(Sin(Scalar{})), 0.0F) < kTolerance);
  EXPECT_TRUE(Error(ToFloat(Cos(Scalar{})), 1.0F) < kTolerance);
  EXPECT_TRUE(Error(ToFloat(Sin(kHalfPi)), 1.0F) < kTolerance);
  EXPECT_TRUE(Error(ToFloat(Cos(kHalfPi)), 0.0F) < kTolerance);
  EXPECT_TRUE(Error(ToFloat(Sin(kPi)), 0.0F) < kTolerance);
  EXPECT_TRUE(Error(ToFloat(Cos(kPi)), -1.0F) < kTolerance);
  EXPECT_TRUE(Error(ToFloat(Sin(-kHalfPi)), -1.0F) < kTolerance);
}

// From -20 to +20 radians (several turns, both signs) against the standard library.
TEST(Trigonometry, MatchesStdOnManyAngles) {
  float worst = 0.0F;
  for (int k = -2000; k <= 2000; ++k) {
    const Scalar angle = Ratio(k, 100);
    const float x = ToFloat(angle);  // the angle really stored (Fixed rounds it)
    worst = std::fmax(worst, Error(ToFloat(Sin(angle)), std::sin(x)));
    worst = std::fmax(worst, Error(ToFloat(Cos(angle)), std::cos(x)));
  }
  EXPECT_TRUE(worst < kTolerance);
}

TEST(Trigonometry, SinSquaredPlusCosSquaredIsOne) {
  for (int k = -300; k <= 300; k += 7) {
    const Scalar angle = Ratio(k, 50);
    const float s = ToFloat(Sin(angle));
    const float c = ToFloat(Cos(angle));
    EXPECT_TRUE(Error(s * s + c * c, 1.0F) < kTolerance);
  }
}

TEST(Trigonometry, WrapAngleStaysInsideAndKeepsTheSine) {
  for (int k = -3000; k <= 3000; k += 37) {
    const Scalar angle = Ratio(k, 100);
    const Scalar wrapped = WrapAngle(angle);
    EXPECT_TRUE(wrapped >= -kPi - Ratio(1, 100) && wrapped <= kPi + Ratio(1, 100));
    EXPECT_TRUE(Error(std::sin(ToFloat(wrapped)), std::sin(ToFloat(angle))) < kTolerance);
  }
}

TEST(Trigonometry, RotateQuarterTurn) {
  const Vec2 v = Rotate(Vec2{Scalar{1}, Scalar{}}, kHalfPi);
  EXPECT_TRUE(Error(ToFloat(v.x()), 0.0F) < kTolerance);
  EXPECT_TRUE(Error(ToFloat(v.y()), 1.0F) < kTolerance);  // +x turns toward +y
}

}  // namespace
}  // namespace gpr925