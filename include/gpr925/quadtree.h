//
// Created by alexk on 11.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_QUADTREE_H_
#define GPR925PHYSICS_INCLUDE_GPR925_QUADTREE_H_
//For now, it's naive quad tree. Next step is to do the narrow phase

#include <array>
#include <compare>
#include <cstddef>
#include <memory>
#include <vector>

#include "vec2.h"

namespace gpr925 {

// One thing stored in the tree: its AABB and an id given by the caller
// (for the world: the index of the (body, collider) entry).
struct QuadTreeObject {
  std::size_t id;
  Rect aabb;
};

//pair candidate, the ids of the objects overlapping
//first < second, so (A, B) and (B, A) are the same pair.
struct BroadphasePair {
  std::size_t first;
  std::size_t second;

  auto operator<=>(const BroadphasePair&) const = default;
};

//AABB against AABB, touching counts
[[nodiscard]] bool OverlapRect(const Rect& a, const Rect& b);

class QuadTree {
 public:
  //max number of objects per node
  static constexpr std::size_t kMaxObjects = 4;
  //max depth of a node, avoid infinite recursion
  static constexpr int kMaxDepth = 5;

  QuadTree() = default;
  QuadTree(const Rect& bounds, int depth) : bounds_(bounds), depth_(depth) {}

  //insert is recursive
  void Insert(const QuadTreeObject& object);

  void GeneratePairs(std::vector<BroadphasePair>& pairs) const;

  //number of nodes stored in the tree
  [[nodiscard]] std::size_t CountStoredObjects() const;

  template <class Visitor>
  void ForEachNode(Visitor& visit) const {
    visit(bounds_);
    for (const auto& child : children_) {
      if (child) {
        child->ForEachNode(visit);
      }
    }
  }

 private:
  void Split();
  [[nodiscard]] Rect Quadrant(std::size_t index) const;
  //AddPairs will first check the children, then the objects in the current node
  void AddPairs(std::vector<BroadphasePair>& pairs) const;

  Rect bounds_{};
  int depth_ = 0;
  std::vector<QuadTreeObject> objects_;
  std::array<std::unique_ptr<QuadTree>, 4> children_{};
};

}  // namespace gpr925

#endif  // GPR925PHYSICS_INCLUDE_GPR925_QUADTREE_H_
