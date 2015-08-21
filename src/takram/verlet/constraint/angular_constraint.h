//
//  takram/verlet/constraint/angular_constraint.h
//
//  LGPL License
//
//  Copyright (C) 2015 Shota Matsuda
//
//  Based on toxiclibs <toxiclibs.org>
//  Copyright (C) 2006-2011 Karsten Schmidt
//  Distributed under LGPL License
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
//

#pragma once
#ifndef TAKRAM_VERLET_CONSTRAINT_ANGULAR_CONSTRAINT_H_
#define TAKRAM_VERLET_CONSTRAINT_ANGULAR_CONSTRAINT_H_

#include <cassert>
#include <cmath>

#include "takram/math/vector.h"
#include "takram/verlet/constraint/constraint.h"
#include "takram/verlet/particle.h"

namespace takram {
namespace verlet {
namespace constraint {

template <class T>
class AngularConstraint : public Constraint2<T> {
 public:
  explicit AngularConstraint(T angle);
  AngularConstraint(T angle, const Vec2<T>& origin);

  // Copy semantics
  AngularConstraint(const AngularConstraint&) = default;
  AngularConstraint& operator=(const AngularConstraint&) = default;

  // Constraint
  void apply(Particle2<T> *particle) override;

  // Parameters
  T angle() const { return angle_; }
  void set_angle(T value) { angle_ = value; }
  const Vec2<T>& origin() const { return origin_; }
  void set_origin(const Vec2<T>& value) { origin_ = value; }

 private:
  T angle_;
  Vec2<T> origin_;
};

#pragma mark -

template <class T>
inline AngularConstraint<T>::AngularConstraint(T angle) : angle_(angle) {}

template <class T>
inline AngularConstraint<T>::AngularConstraint(T angle, const Vec2<T>& origin)
    : angle_(angle),
      origin_(origin) {}

#pragma mark Constraint

template <class T>
inline void AngularConstraint<T>::apply(Particle2<T> *particle) {
  assert(particle);
  auto& position = particle->position();
  const auto delta = position - origin_;
  const auto angle = std::floor(delta.heading() / angle_) * angle_;
  particle->moveTo(origin_ + Vec2<T>::heading(angle) * delta.magnitude());
}

}  // namespace constraint

using constraint::AngularConstraint;

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_CONSTRAINT_ANGULAR_CONSTRAINT_H_
