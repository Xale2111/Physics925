//
// Created by alexk on 11.11.2026.
//

#include "gpr925/quadtree.h"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace gpr925 {

bool OverlapRect(const Rect& a, const Rect& b) {
  return a.min_bound.x() <= b.max_bound.x() && a.max_bound.x() >= b.min_bound.x() &&
         a.min_bound.y() <= b.max_bound.y() && a.max_bound.y() >= b.min_bound.y();
}

namespace {

void AddPair(std::vector<BroadphasePair>& pairs, std::size_t a, std::size_t b) {
  //smaller id first. A,B is the same as B,A
  pairs.push_back(a < b ? BroadphasePair{a, b} : BroadphasePair{b, a});
}

}  // namespace

void QuadTree::Insert(const QuadTreeObject& object) {
  if (children_[0]) {
    const Vec2 middle = bounds_.Center();
    const bool left = object.aabb.min_bound.x() <= middle.x();
    const bool right = object.aabb.max_bound.x() >= middle.x();
    const bool top = object.aabb.min_bound.y() <= middle.y();
    const bool bottom = object.aabb.max_bound.y() >= middle.y();
    if (left && top) children_[0]->Insert(object);
    if (right && top) children_[1]->Insert(object);
    if (left && bottom) children_[2]->Insert(object);
    if (right && bottom) children_[3]->Insert(object);
    return;
  }
  objects_.push_back(object);
  if (objects_.size() > kMaxObjects && depth_ < kMaxDepth) {
    Split();
  }
}

//create four children and push down all objects. Doesn't keep track of the objects
void QuadTree::Split() {
  for (std::size_t i = 0; i < children_.size(); ++i) {
    children_[i] = std::make_unique<QuadTree>(Quadrant(i), depth_ + 1);
  }
  auto objects = std::move(objects_);
  objects_.clear();
  for (const QuadTreeObject& object : objects) {
    Insert(object);
  }
}

Rect QuadTree::Quadrant(std::size_t index) const {
  const Vec2 middle = bounds_.Center();
  const bool high_x = (index & 1U) != 0;  //bit 0 -> right half
  const bool high_y = (index & 2U) != 0;  //bit 1 -> bottom half
  Rect quadrant;
  quadrant.min_bound = Vec2{high_x ? middle.x() : bounds_.min_bound.x(),
                            high_y ? middle.y() : bounds_.min_bound.y()};
  quadrant.max_bound = Vec2{high_x ? bounds_.max_bound.x() : middle.x(),
                            high_y ? bounds_.max_bound.y() : middle.y()};
  return quadrant;
}

void QuadTree::GeneratePairs(std::vector<BroadphasePair>& pairs) const {
  const std::size_t first_new = pairs.size();
  AddPairs(pairs);
  //An object stored in several leaves can give the same pair in each of them
  //sort what was just added, then drop the repeats. A resolution would run twice on the same contact otherwise.
  const auto first = pairs.begin() + static_cast<std::ptrdiff_t>(first_new);
  std::ranges::sort(first, pairs.end());
  const auto duplicates = std::ranges::unique(first, pairs.end());
  pairs.erase(duplicates.begin(), duplicates.end());
}

void QuadTree::AddPairs(std::vector<BroadphasePair>& pairs) const {
  // The children first
  if (children_[0]) {
    for (const auto& child : children_) {
      child->AddPairs(pairs);
    }
  }
  for (std::size_t i = 0; i < objects_.size(); ++i) {
    for (std::size_t j = i + 1; j < objects_.size(); ++j) {
      if (OverlapRect(objects_[i].aabb, objects_[j].aabb)) {
        AddPair(pairs, objects_[i].id, objects_[j].id);
      }
    }
  }
}

std::size_t QuadTree::CountStoredObjects() const {
  std::size_t count = objects_.size();
  for (const auto& child : children_) {
    if (child) {
      count += child->CountStoredObjects();
    }
  }
  return count;
}

}  // namespace gpr925