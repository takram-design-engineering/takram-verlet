//
//  takram/verlet/spring/spring.h
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
#ifndef TAKRAM_VERLET_SPRING_SPRING_H_
#define TAKRAM_VERLET_SPRING_SPRING_H_

#include <cassert>
#include <limits>
#include <type_traits>
#include <utility>

#include "takram/math/vector.h"
#include "takram/verlet/particle.h"

namespace takram {
namespace verlet {
namespace spring {

template <class T, int D>
class Spring {
  static_assert(!std::is_integral<T>::value, "Integral type is not supported");

 public:
  using Type = T;
  static constexpr const int dimensions = D;

 public:
  Spring(Particle<T, D> *a, Particle<T, D> *b, T rest_length, T strength);
  virtual ~Spring() = default;

  // Copy semantics
  Spring(const Spring&) = default;
  Spring& operator=(const Spring&) = default;

  // Updates
  virtual void update();
  void applyConstraints();

  // Attributes
  Particle<T, D>& a() const { return *a_; }
  Particle<T, D>& b() const { return *b_; }

  // Parameters
  T rest_length() const { return rest_length_; }
  T rest_length_squared() const { return rest_length_; }
  void set_rest_length(T value);
  T strength() const { return strength_; }
  void set_strength(T value) { strength_ = value; }
  T force_limit() const { return force_limit_; }
  void set_force_limit(T value) { force_limit_ = value; }

 protected:
  Vec<T, D> calculateForce() const;

 private:
  Particle<T, D> *a_;
  Particle<T, D> *b_;
  T rest_length_;
  T rest_length_squared_;
  T strength_;
  T force_limit_;
};

template <class T>
using Spring2 = Spring<T, 2>;
template <class T>
using Spring3 = Spring<T, 3>;

using Spring2f = Spring2<float>;
using Spring2d = Spring2<double>;
using Spring3f = Spring3<float>;
using Spring3d = Spring3<double>;

#pragma mark -

template <class T, int D>
inline Spring<T, D>::Spring(Particle<T, D> *a,
                            Particle<T, D> *b,
                            T rest_length,
                            T strength)
    : a_(a),
      b_(b),
      rest_length_(rest_length),
      rest_length_squared_(rest_length * rest_length),
      strength_(strength),
      force_limit_() {
  assert(a && b);
}

#pragma mark Updates

template <class T, int D>
inline void Spring<T, D>::update() {
  const auto force = calculateForce();
  if (!a_->fixed()) {
    a_->moveBy(force * a_->inverse_mass());
  }
  if (!b_->fixed()) {
    b_->moveBy(-force * b_->inverse_mass());
  }
}

template <class T, int D>
inline void Spring<T, D>::applyConstraints() {
  if (!a_->fixed()) {
    a_->applyConstraints();
  }
  if (!b_->fixed()) {
    b_->applyConstraints();
  }
}

template <class T, int D>
inline Vec<T, D> Spring<T, D>::calculateForce() const {
  const auto delta = b_->position() - a_->position();
  const auto distance = delta.magnitude() + std::numeric_limits<T>::epsilon();
  Vec<T, D> force(delta * (distance - rest_length_) /
      (distance * (a_->inverse_mass() + b_->inverse_mass())) * strength_);
  if (force_limit_) {
    force.limit(force_limit_);
  }
  return std::move(force);
}

#pragma mark Parameters

template <class T, int D>
inline void Spring<T, D>::set_rest_length(T value) {
  rest_length_ = value;
  rest_length_squared_ = value * value;
}

}  // namespace spring

using spring::Spring;
using spring::Spring2;
using spring::Spring3;

using spring::Spring2f;
using spring::Spring2d;
using spring::Spring3f;
using spring::Spring3d;

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_SPRING_SPRING_H_
