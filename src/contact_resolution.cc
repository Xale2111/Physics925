//
// Created by alexk on 16.11.2026.
//
#include "gpr925/contact_resolution.h"

namespace gpr925 {

namespace {

  //turns around the center perpendicular to r
  Vec2 VelocityAt(const Body& body, const Vec2& r)
  {
    const Scalar omega = body.GetAngularVelocity();
    return body.GetVelocity() + Vec2{-omega * r.y(), omega * r.x()};
  }
}

void ResolveContact(Body& a, Body& b, const Contact& contact, Scalar restitution, Scalar friction)
{
    const Scalar inv_mass_a = a.GetInverseMass();
    const Scalar inv_mass_b = b.GetInverseMass();
    const Scalar inv_mass_sum = inv_mass_a + inv_mass_b;
    if (inv_mass_sum == Scalar{}) return; //nothing can move since both masses are 0

    //inverse inertia
    const Scalar inv_inertia_a = a.GetInverseMass();
    const Scalar inv_inertia_b = b.GetInverseMass();
    const Vec2& normal = contact.normal;

    //rotation point for a and b
    const Vec2 r_a = contact.point - a.GetPosition();
    const Vec2 r_b = contact.point - b.GetPosition();

    //Relative velocity at contact point along the normal
    //if negative, the two points are coming closer
    //if positive, the two points are separating
    const Vec2 relative_velocity = VelocityAt(b, r_b) - VelocityAt(a, r_a);
    const Scalar closing_speed = Vec2::Dot(relative_velocity, normal);

    Scalar normal_impulse{};
    if(closing_speed < Scalar{})
    {
      const Scalar r_a_cross_normal = Vec2::Cross(r_a, normal);
      const Scalar r_b_cross_normal = Vec2::Cross(r_b, normal);
      //(r * normal)^2
      const Scalar effective_inv_mass = inv_mass_sum +
          r_a_cross_normal * r_a_cross_normal * inv_inertia_a +
          r_b_cross_normal * r_b_cross_normal * inv_inertia_b;

      //Restitution of Newton
      const Scalar j = -(Scalar{1} + restitution) * closing_speed/effective_inv_mass;
      normal_impulse = j;

      //Third law of Newton
      const Vec2 impulse = j * normal;
      a.SetVelocity(a.GetVelocity() - impulse * inv_mass_a);
      b.SetVelocity(b.GetVelocity() + impulse * inv_mass_b);
      a.SetAngularVelocity(a.GetAngularVelocity() - inv_inertia_a * r_a_cross_normal * j);
      b.SetAngularVelocity(b.GetAngularVelocity() + inv_inertia_b * r_b_cross_normal * j);
    }

    if(normal_impulse > Scalar{})
    {
      const Vec2 velocity_after = VelocityAt(b, r_b) - VelocityAt(a, r_a);

      const Vec2 sliding = velocity_after - normal * Vec2::Dot(velocity_after, normal);
      const Scalar sliding_sqr = sliding.SqrMagnitude();

      if(sliding_sqr > Scalar{})
      {
        const Scalar sliding_speed = maths::NumberTraits<Scalar>::Sqrt(sliding_sqr);
        const Vec2 tangent = sliding * (Scalar{1} / sliding_speed);
        const Scalar r_a_cross_tangent = Vec2::Cross(r_a, tangent);
        const Scalar r_b_cross_tangent = Vec2::Cross(r_b, tangent);

        const Scalar tangent_inv_mass = inv_mass_sum +
            r_a_cross_tangent * r_a_cross_tangent * inv_inertia_a +
            r_b_cross_tangent * r_b_cross_tangent * inv_inertia_b;

       Scalar j_t = -sliding_speed / tangent_inv_mass;

       const Scalar max_friction = friction *normal_impulse;
       j_t = std::clamp(j_t, -max_friction, max_friction);

       const Vec2 friction_impulse = tangent * j_t;

       a.SetVelocity(a.GetVelocity() - friction_impulse * inv_mass_a);
       b.SetVelocity(b.GetVelocity() + friction_impulse * inv_mass_b);
       a.SetAngularVelocity(a.GetAngularVelocity() - inv_inertia_a * r_a_cross_tangent * j_t);
       b.SetAngularVelocity(b.GetAngularVelocity() + inv_inertia_b * r_b_cross_tangent * j_t);
      }
    }

    //position correction
    const Scalar share_a = inv_mass_a / inv_mass_sum;
    const Scalar share_b = inv_mass_b / inv_mass_sum;

    a.SetPosition(a.GetPosition() - normal * (contact.penetration * share_a));
    b.SetPosition(b.GetPosition() + normal * (contact.penetration * share_b));
}
}