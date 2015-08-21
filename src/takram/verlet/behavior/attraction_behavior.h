//
//  takram/verlet/behavior/attraction_behavior.h
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
#ifndef TAKRAM_VERLET_BEHAVIOR_ATTRACTION_BEHAVIOR_H_
#define TAKRAM_VERLET_BEHAVIOR_ATTRACTION_BEHAVIOR_H_

#include <cassert>

#include "takram/math/vector.h"
#include "takram/verlet/behavior/behavior.h"
#include "takram/verlet/particle.h"
#include "takram/verlet/physics.h"

namespace takram {
namespace verlet {
namespace behavior {

template <class T, int D>
class AttractionBehavior : public Behavior<T, D> {
 public:
  AttractionBehavior(const Vec<T, D>& attractor, T radius, T strength);
  AttractionBehavior(const Vec<T, D>& attractor,
                     T radius,
                     T strength,
                     T jitter);

  // Copy semantics
  AttractionBehavior(const AttractionBehavior&) = default;
  AttractionBehavior& operator=(const AttractionBehavior&) = default;

  // Behavior
  void apply(Particle<T, D> *particle) override;
  void configure(const Physics<T, D>& physics) override;

  // Parameters
  const Vec<T, D>& attractor() const { return attractor_; }
  void set_attractor(const Vec<T, D>& value);
  T radius() const { return radius_; }
  void set_radius(T value);
  T strength() const { return strength_; }
  void set_strength(T value);
  T jitter() const { return jitter_; }
  void set_jitter(T value);

 protected:
  Vec<T, D> attractor_;
  T radius_;
  T radius_squared_;
  T strength_;
  T jitter_;
  T time_step_;
};

template <class T>
using AttractionBehavior2 = AttractionBehavior<T, 2>;
template <class T>
using AttractionBehavior3 = AttractionBehavior<T, 3>;

using AttractionBehavior2f = AttractionBehavior2<float>;
using AttractionBehavior2d = AttractionBehavior2<double>;
using AttractionBehavior3f = AttractionBehavior3<float>;
using AttractionBehavior3d = AttractionBehavior3<double>;

#pragma mark -

template <class T, int D>
inline AttractionBehavior<T, D>::AttractionBehavior(
    const Vec<T, D>& attractor,
    T radius,
    T strength)
    : AttractionBehavior(attractor, radius, strength, T()) {}

template <class T, int D>
inline AttractionBehavior<T, D>::AttractionBehavior(
    const Vec<T, D>& attractor,
    T radius,
    T strength,
    T jitter)
    : attractor_(attractor),
      radius_(radius),
      radius_squared_(radius * radius),
      strength_(strength),
      jitter_(jitter),
      time_step_() {}

#pragma mark Behavior

template <class T, int D>
inline void AttractionBehavior<T, D>::apply(Particle<T, D> *particle) {
  assert(particle);
  const auto delta = attractor_ - particle->position();
  const auto distance = delta.magnitudeSquared();
  if (distance < radius_squared_) {
    auto force = delta.normalized() * (1 - distance / radius_squared_);
    force.jitter({jitter_, jitter_});
    particle->addForce(force * strength_ * time_step_);
  }
}

template <class T, int D>
inline void AttractionBehavior<T, D>::configure(const Physics<T, D>& physics) {
  time_step_ = physics.time_step();
}

#pragma mark Parameters

template <class T, int D>
inline void AttractionBehavior<T, D>::set_attractor(const Vec<T, D>& value) {
  attractor_ = value;
}

template <class T, int D>
inline void AttractionBehavior<T, D>::set_radius(T value) {
  radius_ = value;
  radius_squared_ = value * value;
}

template <class T, int D>
inline void AttractionBehavior<T, D>::set_strength(T value) {
  strength_ = value;
}

template <class T, int D>
inline void AttractionBehavior<T, D>::set_jitter(T value) {
  jitter_ = value;
}

}  // namespace behavior

using behavior::AttractionBehavior;
using behavior::AttractionBehavior2;
using behavior::AttractionBehavior3;

using behavior::AttractionBehavior2f;
using behavior::AttractionBehavior2d;
using behavior::AttractionBehavior3f;
using behavior::AttractionBehavior3d;

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_BEHAVIOR_ATTRACTION_BEHAVIOR_H_
