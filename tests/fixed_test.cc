#include "gpr925/fixed.h"

#include <gtest/gtest.h>

namespace gpr925 {
namespace {

constexpr float kStep = 1.0f / 4096.0f;

TEST(Fixed, ConvertsFromAndToInt) {
  EXPECT_EQ(Fixed{3}.raw(), 3 * 4096);
  EXPECT_EQ(static_cast<int>(Fixed{-7}), -7);
  // Truncation toward zero, like libnds' f32toint
  EXPECT_EQ(ToInt(Fixed{1.75f}), 1);
  EXPECT_EQ(ToInt(Fixed{-1.75f}), -1);
}

TEST(Fixed, ConvertsFromFloatConstant) {
  EXPECT_EQ(Fixed{0.5f}.raw(), 2048);
  EXPECT_EQ(Fixed{-0.25f}.raw(), -1024);
  EXPECT_FLOAT_EQ(ToFloat(Fixed{1.5f}), 1.5f);
}

TEST(Fixed, AddsAndSubtracts) {
  EXPECT_EQ(Fixed{1.5f} + Fixed{2.25f}, Fixed{3.75f});
  EXPECT_EQ(Fixed{1.5f} - Fixed{2.25f}, Fixed{-0.75f});
  EXPECT_EQ(-Fixed{2}, Fixed{-2});
  Fixed value{1};
  value += Fixed{0.5f};
  value -= Fixed{2};
  EXPECT_EQ(value, Fixed{-0.5f});
}

TEST(Fixed, Multiplies) {
  EXPECT_EQ(Fixed{3} * Fixed{0.5f}, Fixed{1.5f});
  EXPECT_EQ(Fixed{-3} * Fixed{0.5f}, Fixed{-1.5f});
  EXPECT_EQ(Fixed{0.5f} * 4, Fixed{2});
  EXPECT_EQ(4 * Fixed{0.5f}, Fixed{2});
  Fixed value{3};
  value *= Fixed{3};
  EXPECT_EQ(value, Fixed{9});
}

TEST(Fixed, Divides) {
  EXPECT_EQ(Fixed{3} / Fixed{2}, Fixed{1.5f});
  EXPECT_EQ(Fixed{-3} / Fixed{2}, Fixed{-1.5f});
  EXPECT_EQ(Fixed{3} / 2, Fixed{1.5f});
  EXPECT_NEAR(ToFloat(Fixed{1} / Fixed{3}), 1.0f / 3.0f, kStep);
  Fixed value{9};
  value /= Fixed{3};
  EXPECT_EQ(value, Fixed{3});
}

TEST(Fixed, Compares) {
  EXPECT_LT(Fixed{1}, Fixed{1.5f});
  EXPECT_GT(Fixed{-1}, Fixed{-1.5f});
  EXPECT_LE(Fixed{2}, Fixed{2});
  EXPECT_NE(Fixed{2}, Fixed{3});
}

TEST(Fixed, SquareRoot) {
  EXPECT_EQ(Sqrt(Fixed{4}), Fixed{2});
  EXPECT_EQ(Sqrt(Fixed{0.25f}), Fixed{0.5f});
  EXPECT_NEAR(ToFloat(Sqrt(Fixed{2})), 1.41421356f, kStep);
  EXPECT_EQ(Sqrt(Fixed{}), Fixed{});
  EXPECT_EQ(Sqrt(Fixed{-4}), Fixed{});
}

TEST(Fixed, AbsoluteValue) {
  EXPECT_EQ(Abs(Fixed{-1.5f}), Fixed{1.5f});
  EXPECT_EQ(Abs(Fixed{1.5f}), Fixed{1.5f});
}

// Everything but the DS hardware calls is usable in constant expressions
static_assert(Fixed{3} * Fixed{0.5f} == Fixed{1.5f});
static_assert(Fixed{1} / Fixed{4} == Fixed{0.25f});
static_assert(Sqrt(Fixed{9}) == Fixed{3});

}  // namespace
}  // namespace gpr925
