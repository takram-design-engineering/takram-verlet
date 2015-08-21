//
//  takram/verlet/constraint/circular_constraint.h
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
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
//

#pragma once
#ifndef TAKRAM_VERLET_CONSTRAINT_CIRCULAR_CONSTRAINT_H_
#define TAKRAM_VERLET_CONSTRAINT_CIRCULAR_CONSTRAINT_H_

#include <cassert>

#include "takram/math/circle.h"
#include "takram/verlet/constraint/constraint.h"
#include "takram/verlet/particle.h"

namespace takram {
namespace verlet {
namespace constraint {

template <class T>
class CircularConstraint : public Constraint2<T> {
 public:
  template <class... Args>
  explicit CircularConstraint(Args&&... args);

  // Copy semantics
  CircularConstraint(const CircularConstraint&) = default;
  CircularConstraint& operator=(const CircularConstraint&) = default;

  // Constraint
  void apply(Particle2<T> *particle) override;

  // Parameters
  const Circle2<T>& circle() const { return circle_; }
  void set_circle(const Circle2<T>& value) { circle_ = value; }

 private:
  Circle2<T> circle_;
};

#pragma mark -

template <class T>
template <class... Args>
inline CircularConstraint<T>::CircularConstraint(Args&&... args)
    : circle_(args...) {}

#pragma mark Constraint

template <class T>
inline void CircularConstraint<T>::apply(Particle2<T> *particle) {
  assert(particle);
  if (circle_.contains(particle->position())) {
    const auto normal = (particle->position() - circle_.center).normalize();
    particle->moveTo(circle_.center + normal * circle_.radius);
  }
}

}  // namespace constraint

using constraint::CircularConstraint;

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_CONSTRAINT_CIRCULAR_CONSTRAINT_H_
