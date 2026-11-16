//
// Created by alexk on 16.11.2026.
//
#include "gpr925/oriented_box_collider.h"

#include "gpr925/trigonometry.h"

namespace gpr925 {
  Vec2 OrientedBoxCollider::GetLocalCenter() const {
    return GetOffset() + Vec2{(min_.x() + max_.x()) / Scalar{2}, (min_.y() +max_.y())/Scalar{2}};
  }

  Vec2 OrientedBoxCollider::GetHalfSize() const {
    return Vec2{(max_.x() - min_.x()) / Scalar{2}, (max_.y() - min_.y()) / Scalar{2}};
  }

  Vec2 OrientedBoxCollider::GetWorldCenter(const gpr925::Vec2& position, gpr925::Scalar angle) const {
    return position + Rotate(GetLocalCenter(), angle);
  }

  std::array<Vec2, 4> OrientedBoxCollider::GetCorners(const gpr925::Vec2& position, gpr925::Scalar angle) const {
   const Vec2 center = GetWorldCenter(position, angle);
   const Vec2 half = GetHalfSize();

   const Scalar cos = Cos(angle);
   const Scalar sin = Sin(angle);

   const auto corner = [&](Scalar x, Scalar y)
   {
     return center + Vec2{x*cos - y * sin, x*sin + y *cos};
   };
   return {corner(-half.x(), -half.y()), corner(half.x(), -half.y()),
   corner(half.x(), half.y()), corner(-half.x(), half.y())};
  }
}