//
//  takram/verlet/constraint/axis_constraint.h
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
#ifndef TAKRAM_VERLET_CONSTRAINT_AXIS_CONSTRAINT_H_
#define TAKRAM_VERLET_CONSTRAINT_AXIS_CONSTRAINT_H_

#include <cassert>

#include "takram/math/axis.h"
#include "takram/math/vector.h"
#include "takram/verlet/constraint/constraint.h"
#include "takram/verlet/particle.h"

namespace takram {
namespace verlet {
namespace constraint {

template <class T>
class AxisConstraint : public Constraint2<T> {
 public:
  AxisConstraint(Axis axis, T constraint);

  // Copy semantics
  AxisConstraint(const AxisConstraint&) = default;
  AxisConstraint& operator=(const AxisConstraint&) = default;

  // Constraint
  void apply(Particle2<T> *particle) override;

  // Parameters
  Axis axis() const { return axis_; }
  void set_axis(Axis value) { axis_ = value; }
  T constraint() const { return constraint_; }
  void set_constraint(T value) { constraint_ = value; }

 private:
  Axis axis_;
  T constraint_;
};

#pragma mark -

template <class T>
inline AxisConstraint<T>::AxisConstraint(Axis axis, T constraint)
    : axis_(axis),
      constraint_(constraint) {}

#pragma mark Constraint

template <class T>
inline void AxisConstraint<T>::apply(Particle2<T> *particle) {
  assert(particle);
  auto position = particle->position();
  position.at(axis_) = constraint_;
  particle->moveTo(position);
}

}  // namespace constraint

using constraint::AxisConstraint;

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_CONSTRAINT_AXIS_CONSTRAINT_H_
