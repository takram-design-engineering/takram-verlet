//
//  takram/verlet/behavior/constant_force_behavior.h
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
#ifndef TAKRAM_VERLET_BEHAVIOR_CONSTANT_FORCE_BEHAVIOR_H_
#define TAKRAM_VERLET_BEHAVIOR_CONSTANT_FORCE_BEHAVIOR_H_

#include <cassert>

#include "takram/math/vector.h"
#include "takram/verlet/behavior/behavior.h"
#include "takram/verlet/particle.h"
#include "takram/verlet/physics.h"

namespace takram {
namespace verlet {
namespace behavior {

template <class T, int D>
class ConstantForceBehavior : public Behavior<T, D> {
 public:
  explicit ConstantForceBehavior(const Vec<T, D>& force);

  // Copy semantics
  ConstantForceBehavior(const ConstantForceBehavior&) = default;
  ConstantForceBehavior& operator=(const ConstantForceBehavior&) = default;

  // Behavior
  void apply(Particle<T, D> *particle) override;
  void configure(const Physics<T, D>& physics) override;

  // Parameters
  const Vec<T, D>& force() const { return force_; }
  void set_force(const Vec<T, D>& value);

 protected:
  Vec<T, D> force_;
  T time_step_;
};

template <class T>
using ConstantForceBehavior2 = ConstantForceBehavior<T, 2>;
template <class T>
using ConstantForceBehavior3 = ConstantForceBehavior<T, 3>;

using ConstantForceBehavior2f = ConstantForceBehavior2<float>;
using ConstantForceBehavior2d = ConstantForceBehavior2<double>;
using ConstantForceBehavior3f = ConstantForceBehavior3<float>;
using ConstantForceBehavior3d = ConstantForceBehavior3<double>;

#pragma mark -

template <class T, int D>
inline ConstantForceBehavior<T, D>::ConstantForceBehavior(
    const Vec<T, D>& force)
    : force_(force),
      time_step_() {}

#pragma mark Behavior

template <class T, int D>
inline void ConstantForceBehavior<T, D>::apply(Particle<T, D> *particle) {
  assert(particle);
  particle->addForce(force_ * time_step_);
}

template <class T, int D>
inline void ConstantForceBehavior<T, D>::configure(
    const Physics<T, D>& physics) {
  time_step_ = physics.time_step();
}

#pragma mark Parameters

template <class T, int D>
inline void ConstantForceBehavior<T, D>::set_force(const Vec<T, D>& value) {
  force_ = value;
}

}  // namespace behavior

using behavior::ConstantForceBehavior;
using behavior::ConstantForceBehavior2;
using behavior::ConstantForceBehavior3;

using behavior::ConstantForceBehavior2f;
using behavior::ConstantForceBehavior2d;
using behavior::ConstantForceBehavior3f;
using behavior::ConstantForceBehavior3d;

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_BEHAVIOR_CONSTANT_FORCE_BEHAVIOR_H_
