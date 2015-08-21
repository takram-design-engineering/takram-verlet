//
//  takram/verlet/behavior/gravity_behavior.h
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
#ifndef TAKRAM_VERLET_BEHAVIOR_GRAVITY_BEHAVIOR_H_
#define TAKRAM_VERLET_BEHAVIOR_GRAVITY_BEHAVIOR_H_

#include <cassert>

#include "takram/math/vector.h"
#include "takram/verlet/behavior/behavior.h"
#include "takram/verlet/particle.h"
#include "takram/verlet/physics.h"

namespace takram {
namespace verlet {
namespace behavior {

template <class T, int D>
class GravityBehavior : public Behavior<T, D> {
 public:
  explicit GravityBehavior(const Vec<T, D>& gravity);

  // Copy semantics
  GravityBehavior(const GravityBehavior&) = default;
  GravityBehavior& operator=(const GravityBehavior&) = default;

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
using GravityBehavior2 = GravityBehavior<T, 2>;
template <class T>
using GravityBehavior3 = GravityBehavior<T, 3>;

using GravityBehavior2f = GravityBehavior2<float>;
using GravityBehavior2d = GravityBehavior2<double>;
using GravityBehavior3f = GravityBehavior3<float>;
using GravityBehavior3d = GravityBehavior3<double>;

#pragma mark -

template <class T, int D>
inline GravityBehavior<T, D>::GravityBehavior(const Vec<T, D>& gravity)
    : force_(gravity),
      time_step_() {}

#pragma mark Behavior

template <class T, int D>
inline void GravityBehavior<T, D>::apply(Particle<T, D> *particle) {
  assert(particle);
  particle->addForce(force_ * time_step_ * time_step_);
}

template <class T, int D>
inline void GravityBehavior<T, D>::configure(const Physics<T, D>& physics) {
  time_step_ = physics.time_step();
}

#pragma mark Parameters

template <class T, int D>
inline void GravityBehavior<T, D>::set_force(const Vec<T, D>& value) {
  force_ = value;
}

}  // namespace behavior

using behavior::GravityBehavior;
using behavior::GravityBehavior2;
using behavior::GravityBehavior3;

using behavior::GravityBehavior2f;
using behavior::GravityBehavior2d;
using behavior::GravityBehavior3f;
using behavior::GravityBehavior3d;

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_BEHAVIOR_GRAVITY_BEHAVIOR_H_
