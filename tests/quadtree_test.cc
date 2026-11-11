//
// Created by alexk on 11.11.2026.
//
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "gpr925/quadtree.h"

namespace gpr925 {
namespace {

// Whole numbers only: they are exact in float and in the 20.12 fixed point, and
// they make the equalities (touching, middle lines) happen.
Scalar S(int value) { return static_cast<Scalar>(value); }

Rect MakeRect(int min_x, int min_y, int max_x, int max_y) {
  Rect rect;
  rect.min_bound = Vec2{S(min_x), S(min_y)};
  rect.max_bound = Vec2{S(max_x), S(max_y)};
  return rect;
}

using PairSet = std::set<std::pair<std::size_t, std::size_t>>;

PairSet Normalized(const std::vector<BroadphasePair>& pairs) {
  PairSet result;
  for (const BroadphasePair& pair : pairs) {
    result.insert({std::min(pair.first, pair.second), std::max(pair.first, pair.second)});
  }
  return result;
}

PairSet BruteForce(const std::vector<QuadTreeObject>& objects) {
  PairSet result;
  for (std::size_t i = 0; i < objects.size(); ++i) {
    for (std::size_t j = i + 1; j < objects.size(); ++j) {
      if (OverlapRect(objects[i].aabb, objects[j].aabb)) {
        result.insert({objects[i].id, objects[j].id});
      }
    }
  }
  return result;
}

std::vector<BroadphasePair> TreePairs(const Rect& bounds, const std::vector<QuadTreeObject>& objects) {
  QuadTree tree(bounds, 0);
  for (const QuadTreeObject& object : objects) {
    tree.Insert(object);
  }
  std::vector<BroadphasePair> pairs;
  tree.GeneratePairs(pairs);
  return pairs;
}

std::size_t CountNodes(const QuadTree& tree) {
  std::size_t count = 0;
  auto visit = [&count](const Rect&) { ++count; };
  tree.ForEachNode(visit);
  return count;
}

TEST(QuadTree, EmptyTreeHasNoPair) {
  EXPECT_EQ(TreePairs(MakeRect(0, 0, 100, 100), {}).size(), 0U);
}

TEST(QuadTree, TwoOverlappingObjectsGiveOnePair) {
  const std::vector<QuadTreeObject> objects = {{0, MakeRect(10, 10, 20, 20)},
                                               {1, MakeRect(15, 15, 25, 25)}};
  EXPECT_EQ(TreePairs(MakeRect(0, 0, 100, 100), objects).size(), 1U);
}

TEST(QuadTree, TwoFarObjectsGiveNoPair) {
  const std::vector<QuadTreeObject> objects = {{0, MakeRect(10, 10, 20, 20)},
                                               {1, MakeRect(60, 60, 70, 70)}};
  EXPECT_EQ(TreePairs(MakeRect(0, 0, 100, 100), objects).size(), 0U);
}

TEST(QuadTree, TouchingCounts) {
  const std::vector<QuadTreeObject> objects = {{0, MakeRect(10, 10, 20, 20)},
                                               {1, MakeRect(20, 10, 30, 20)}};
  EXPECT_EQ(TreePairs(MakeRect(0, 0, 100, 100), objects).size(), 1U);
}

// Two objects touching exactly on a middle line, once the tree is split: an
// object touching the line must go into both sides, or the pair would be lost.
TEST(QuadTree, TouchingOnAMiddleLineIsFound) {
  std::vector<QuadTreeObject> objects;
  // Enough small objects far away to force a split of the root.
  for (std::size_t i = 0; i < 20; ++i) {
    const int x = 1 + static_cast<int>(i);
    objects.push_back({i, MakeRect(x, 1, x + 1, 2)});
  }
  objects.push_back({100, MakeRect(40, 60, 50, 70)});  // max.x == middle (50)
  objects.push_back({101, MakeRect(50, 60, 60, 70)});  // min.x == middle (50)
  const PairSet expected = BruteForce(objects);
  EXPECT_TRUE(expected.contains({100, 101}));
  EXPECT_TRUE(Normalized(TreePairs(MakeRect(0, 0, 100, 100), objects)) == expected);
}

TEST(QuadTree, ABigObjectOverlapsSmallOnesInEveryQuadrant) {
  std::vector<QuadTreeObject> objects;
  std::size_t id = 0;
  for (int x = 5; x < 100; x += 20) {
    for (int y = 5; y < 100; y += 20) {
      objects.push_back({id++, MakeRect(x, y, x + 3, y + 3)});
    }
  }
  objects.push_back({id++, MakeRect(0, 0, 100, 100)});  // covers everything
  const PairSet expected = BruteForce(objects);
  EXPECT_TRUE(Normalized(TreePairs(MakeRect(0, 0, 100, 100), objects)) == expected);
  EXPECT_EQ(expected.size(), 25U);
}

// An object over a line is stored in every quadrant it overlaps (and nothing is
// compared with the parent).
TEST(QuadTree, AnObjectOverTheMiddleIsStoredInEachQuadrant) {
  QuadTree tree(MakeRect(0, 0, 100, 100), 0);
  for (std::size_t i = 0; i < QuadTree::kMaxObjects; ++i) {
    tree.Insert({i, MakeRect(1, 1, 2, 2)});  // all in the top-left corner
  }
  EXPECT_EQ(tree.CountStoredObjects(), QuadTree::kMaxObjects);
  tree.Insert({100, MakeRect(45, 45, 55, 55)});  // over the center: overlaps the 4 quadrants
  // 8 small ones (once each) + the big one in 4 leaves.
  EXPECT_EQ(tree.CountStoredObjects(), QuadTree::kMaxObjects + 4U);
}

TEST(QuadTree, AnObjectInOneQuadrantIsStoredOnce) {
  QuadTree tree(MakeRect(0, 0, 100, 100), 0);
  for (std::size_t i = 0; i < QuadTree::kMaxObjects + 1U; ++i) {
    tree.Insert({i, MakeRect(1, 1, 2, 2)});
  }
  EXPECT_EQ(tree.CountStoredObjects(), QuadTree::kMaxObjects + 1U);
}

// Two big objects both over the center are in the same four leaves: the pair is
// found four times by the recursion and must come out once.
TEST(QuadTree, ABigPairFoundInSeveralLeavesIsReportedOnce) {
  std::vector<QuadTreeObject> objects;
  for (std::size_t i = 0; i < 12; ++i) {
    const int x = 1 + static_cast<int>(i) * 8;
    objects.push_back({i, MakeRect(x, 1, x + 2, 3)});
  }
  objects.push_back({100, MakeRect(30, 30, 70, 70)});
  objects.push_back({101, MakeRect(40, 40, 80, 80)});
  const std::vector<BroadphasePair> pairs = TreePairs(MakeRect(0, 0, 100, 100), objects);
  EXPECT_EQ(std::count(pairs.begin(), pairs.end(), BroadphasePair{100, 101}), 1);
  EXPECT_TRUE(Normalized(pairs) == BruteForce(objects));
}

TEST(QuadTree, PairsComeSortedWithTheSmallerIdFirst) {
  std::mt19937 rng(3);
  std::uniform_int_distribution<int> position(0, 80);
  std::vector<QuadTreeObject> objects;
  for (std::size_t i = 0; i < 100; ++i) {
    const int x = position(rng);
    const int y = position(rng);
    objects.push_back({i, MakeRect(x, y, x + 20, y + 20)});
  }
  const std::vector<BroadphasePair> pairs = TreePairs(MakeRect(0, 0, 100, 100), objects);
  EXPECT_TRUE(std::is_sorted(pairs.begin(), pairs.end()));
  for (const BroadphasePair& pair : pairs) {
    EXPECT_TRUE(pair.first < pair.second);
  }
}

// The ids can come in any order (the world inserts by entry index, but nothing
// forces it): a pair is always (smaller id, bigger id).
TEST(QuadTree, PairsAreNormalizedWhateverTheInsertionOrder) {
  std::mt19937 rng(11);
  std::uniform_int_distribution<int> position(0, 80);
  std::vector<QuadTreeObject> objects;
  for (std::size_t i = 100; i > 0; --i) {  // ids inserted from big to small
    const int x = position(rng);
    const int y = position(rng);
    objects.push_back({i, MakeRect(x, y, x + 20, y + 20)});
  }
  const std::vector<BroadphasePair> pairs = TreePairs(MakeRect(0, 0, 100, 100), objects);
  EXPECT_GT(pairs.size(), 0U);
  for (const BroadphasePair& pair : pairs) {
    EXPECT_TRUE(pair.first < pair.second);
  }
  EXPECT_EQ(pairs.size(), BruteForce(objects).size());
}

TEST(QuadTree, ObjectsOutsideTheRootAreNotLost) {
  std::vector<QuadTreeObject> objects;
  for (std::size_t i = 0; i < 12; ++i) {
    const int x = 1 + static_cast<int>(i) * 8;
    objects.push_back({i, MakeRect(x, 1, x + 2, 3)});  // force a split
  }
  objects.push_back({100, MakeRect(-50, -50, -40, -40)});  // far outside, top-left
  objects.push_back({101, MakeRect(-45, -45, -35, -35)});
  objects.push_back({102, MakeRect(150, 150, 160, 160)});  // far outside, bottom-right
  objects.push_back({103, MakeRect(155, 155, 165, 165)});
  const PairSet expected = BruteForce(objects);
  EXPECT_TRUE(expected.contains({100, 101}));
  EXPECT_TRUE(expected.contains({102, 103}));
  EXPECT_TRUE(Normalized(TreePairs(MakeRect(0, 0, 100, 100), objects)) == expected);
}

TEST(QuadTree, NoPairIsReportedTwice) {
  std::mt19937 rng(7);
  std::uniform_int_distribution<int> position(0, 90);
  std::uniform_int_distribution<int> size(1, 30);
  std::vector<QuadTreeObject> objects;
  for (std::size_t i = 0; i < 150; ++i) {
    const int x = position(rng);
    const int y = position(rng);
    objects.push_back({i, MakeRect(x, y, x + size(rng), y + size(rng))});
  }
  const std::vector<BroadphasePair> pairs = TreePairs(MakeRect(0, 0, 100, 100), objects);
  EXPECT_EQ(pairs.size(), Normalized(pairs).size());
}

// The property that matters: the broadphase never loses a pair that overlaps.
// Integer coordinates make the equalities (touching, middle lines) happen.
TEST(QuadTree, SameResultAsBruteForceOnRandomObjects) {
  for (unsigned seed = 1; seed <= 40; ++seed) {
    std::mt19937 rng(seed);
    // Wider than the world on purpose: objects may be outside the root bounds.
    std::uniform_int_distribution<int> position(-30, 130);
    std::uniform_int_distribution<int> size(0, 25);
    std::uniform_int_distribution<int> count(0, 250);
    std::vector<QuadTreeObject> objects;
    const int n = count(rng);
    for (int i = 0; i < n; ++i) {
      const int x = position(rng);
      const int y = position(rng);
      objects.push_back({static_cast<std::size_t>(i), MakeRect(x, y, x + size(rng), y + size(rng))});
    }
    EXPECT_TRUE(Normalized(TreePairs(MakeRect(0, 0, 100, 100), objects)) == BruteForce(objects));
  }
}

// Same result with a world whose bounds are not centered on zero, and a mix of
// tiny and huge objects.
TEST(QuadTree, SameResultAsBruteForceWithMixedSizes) {
  std::mt19937 rng(99);
  std::uniform_int_distribution<int> position(0, 200);
  std::uniform_int_distribution<int> small_size(0, 3);
  std::uniform_int_distribution<int> big_size(40, 120);
  std::vector<QuadTreeObject> objects;
  for (std::size_t i = 0; i < 300; ++i) {
    const int x = position(rng);
    const int y = position(rng);
    const bool big = i % 25 == 0;
    const int width = big ? big_size(rng) : small_size(rng);
    const int height = big ? big_size(rng) : small_size(rng);
    objects.push_back({i, MakeRect(x, y, x + width, y + height)});
  }
  EXPECT_TRUE(Normalized(TreePairs(MakeRect(50, 50, 250, 250), objects)) == BruteForce(objects));
}

TEST(QuadTree, ObjectsPiledOnOneSpotStopAtTheDepthLimit) {
  std::vector<QuadTreeObject> objects;
  for (std::size_t i = 0; i < 200; ++i) {
    objects.push_back({i, MakeRect(10, 10, 11, 11)});  // all identical
  }
  QuadTree tree(MakeRect(0, 0, 100, 100), 0);
  for (const QuadTreeObject& object : objects) {
    tree.Insert(object);
  }
  std::vector<BroadphasePair> pairs;
  tree.GeneratePairs(pairs);
  EXPECT_EQ(pairs.size(), 200U * 199U / 2U);  // still O(n^2) inside the pile

  // A full tree of depth d has (4^(d+1) - 1) / 3 nodes: we can never have more.
  std::size_t full_tree = 0;
  std::size_t level = 1;
  for (int depth = 0; depth <= QuadTree::kMaxDepth; ++depth) {
    full_tree += level;
    level *= 4;
  }
  EXPECT_LE(CountNodes(tree), full_tree);
}

TEST(QuadTree, SpreadObjectsSubdivideTheTree) {
  QuadTree tree(MakeRect(0, 0, 100, 100), 0);
  for (std::size_t i = 0; i < 100; ++i) {
    const int x = static_cast<int>((i % 10) * 10);
    const int y = static_cast<int>((i / 10) * 10);
    tree.Insert({i, MakeRect(x + 1, y + 1, x + 3, y + 3)});
  }
  EXPECT_GT(CountNodes(tree), 1U);
}

TEST(QuadTree, FewObjectsDoNotSplit) {
  QuadTree tree(MakeRect(0, 0, 100, 100), 0);
  for (std::size_t i = 0; i < QuadTree::kMaxObjects; ++i) {
    tree.Insert({i, MakeRect(1, 1, 2, 2)});
  }
  EXPECT_EQ(CountNodes(tree), 1U);  // exactly kMaxObjects: not split yet
  tree.Insert({99, MakeRect(1, 1, 2, 2)});
  EXPECT_GT(CountNodes(tree), 1U);  // one more: it splits
}

// The tree must refine the part of the world where the objects are, not
// another one (a quadrant index that does not match its rectangle would still
// give the right pairs, but a useless tree and wrong lines on screen).
TEST(QuadTree, SubdividesWhereTheObjectsAre) {
  QuadTree tree(MakeRect(0, 0, 100, 100), 0);
  for (std::size_t i = 0; i < 20; ++i) {
    const int x = 80 + static_cast<int>(i % 5) * 3;   // top-right corner of the world
    const int y = 2 + static_cast<int>(i / 5) * 2;
    tree.Insert({i, MakeRect(x, y, x + 1, y + 1)});
  }
  int deep_nodes_around_objects = 0;
  auto visit = [&deep_nodes_around_objects](const Rect& node) {
    const bool deep = node.max_bound.x() - node.min_bound.x() <= S(25);  // depth 2 or more
    const bool around = node.min_bound.x() <= S(85) && node.max_bound.x() >= S(85) &&
                        node.min_bound.y() <= S(5) && node.max_bound.y() >= S(5);
    if (deep && around) {
      ++deep_nodes_around_objects;
    }
  };
  tree.ForEachNode(visit);
  EXPECT_GT(deep_nodes_around_objects, 0);
}

TEST(QuadTree, ForEachNodeVisitsTheRootFirst) {
  const Rect bounds = MakeRect(0, 0, 100, 100);
  QuadTree tree(bounds, 0);
  std::vector<Rect> visited;
  auto visit = [&visited](const Rect& rect) { visited.push_back(rect); };
  tree.ForEachNode(visit);
  EXPECT_EQ(visited.size(), 1U);
  EXPECT_TRUE(visited[0].max_bound.x() == S(100));
}

}  // namespace
}  // namespace gpr925