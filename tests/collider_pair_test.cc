//
// Created by alexk on 03.11.2026.
//

#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
#include <vector>

#include "gpr925/collider_pair.h"

namespace gpr925 {
namespace {

constexpr BodyId Id(std::size_t value) { return static_cast<BodyId>(value); }

TEST(ColliderPairTest, ShouldKeepTheSmallerIdFirst) {
  const ColliderPair pair(Id(7), Id(3));
  EXPECT_EQ(pair.first(), Id(3));
  EXPECT_EQ(pair.second(), Id(7));
}

TEST(ColliderPairTest, OrderOfTheArgumentsShouldNotMatter) {
  EXPECT_EQ(ColliderPair(Id(1), Id(2)), ColliderPair(Id(2), Id(1)));
}

TEST(ColliderPairTest, DifferentPairsShouldBeDifferent) {
  EXPECT_NE(ColliderPair(Id(1), Id(2)), ColliderPair(Id(1), Id(3)));
  EXPECT_NE(ColliderPair(Id(1), Id(2)), ColliderPair(Id(0), Id(2)));
}

TEST(ColliderPairTest, ShouldBeOrderedByFirstThenSecond) {
  EXPECT_LT(ColliderPair(Id(0), Id(5)), ColliderPair(Id(1), Id(2)));
  EXPECT_LT(ColliderPair(Id(1), Id(2)), ColliderPair(Id(1), Id(3)));
  EXPECT_GT(ColliderPair(Id(2), Id(3)), ColliderPair(Id(1), Id(9)));
}

TEST(ColliderPairTest, ShouldBeSortable) {
  std::vector<ColliderPair> pairs = {
      ColliderPair(Id(2), Id(3)), ColliderPair(Id(1), Id(0)),
      ColliderPair(Id(1), Id(2)), ColliderPair(Id(3), Id(2))};
  std::ranges::sort(pairs);
  EXPECT_EQ(pairs[0], ColliderPair(Id(0), Id(1)));
  EXPECT_EQ(pairs[1], ColliderPair(Id(1), Id(2)));
  EXPECT_EQ(pairs[2], ColliderPair(Id(2), Id(3)));
  EXPECT_EQ(pairs[3], ColliderPair(Id(2), Id(3)));
}

TEST(ColliderPairTest, SetDifferenceShouldFindTheChanges) {
  // The enter/exit bookkeeping relies on this on two sorted lists.
  const std::vector<ColliderPair> previous = {ColliderPair(Id(0), Id(1)),
                                              ColliderPair(Id(1), Id(2))};
  const std::vector<ColliderPair> current = {ColliderPair(Id(1), Id(2)),
                                             ColliderPair(Id(2), Id(0))};
  std::vector<ColliderPair> entered;
  std::vector<ColliderPair> exited;
  std::ranges::set_difference(current, previous, std::back_inserter(entered));
  std::ranges::set_difference(previous, current, std::back_inserter(exited));
  EXPECT_EQ(entered, (std::vector<ColliderPair>{ColliderPair(Id(0), Id(2))}));
  EXPECT_EQ(exited, (std::vector<ColliderPair>{ColliderPair(Id(1), Id(0))}));
}

TEST(ColliderPairTest, ShouldBeUsableInConstantExpressions) {
  constexpr ColliderPair pair(Id(9), Id(4));
  static_assert(pair.first() == Id(4));
  static_assert(pair.second() == Id(9));
  SUCCEED();
}

}  // namespace
}  // namespace gpr925