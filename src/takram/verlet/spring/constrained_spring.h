//
//  takram/verlet/spring/constrained_spring.h
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
#ifndef TAKRAM_VERLET_SPRING_CONSTRAINED_SPRING_H_
#define TAKRAM_VERLET_SPRING_CONSTRAINED_SPRING_H_

#include <limits>

#include "takram/verlet/particle.h"
#include "takram/verlet/spring/spring.h"

namespace takram {
namespace verlet {
namespace spring {

template <class T, int D>
class ConstrainedSpring : public Spring<T, D> {
 public:
  ConstrainedSpring(Particle<T, D> *a,
                    Particle<T, D> *b,
                    T rest_length,
                    T strength,
                    T limit = std::numeric_limits<T>::max());

  // Copy semantics
  ConstrainedSpring(const ConstrainedSpring&) = default;
  ConstrainedSpring& operator=(const ConstrainedSpring&) = default;

  // Updates
  void update() override;

  // Parameters
  T limit() const { return limit_; }
  void set_limit(T value) { limit_ = value; }

 private:
  T limit_;
};

template <class T>
using ConstrainedSpring2 = ConstrainedSpring<T, 2>;
template <class T>
using ConstrainedSpring3 = ConstrainedSpring<T, 3>;

using ConstrainedSpring2f = ConstrainedSpring2<float>;
using ConstrainedSpring2d = ConstrainedSpring2<double>;
using ConstrainedSpring3f = ConstrainedSpring3<float>;
using ConstrainedSpring3d = ConstrainedSpring3<double>;

#pragma mark -

template <class T, int D>
inline ConstrainedSpring<T, D>::ConstrainedSpring(Particle<T, D> *a,
                                                  Particle<T, D> *b,
                                                  T rest_length,
                                                  T strength,
                                                  T limit)
    : Spring<T, D>(a, b, rest_length, strength),
      limit_(limit) {}

#pragma mark Updates

template <class T, int D>
inline void ConstrainedSpring<T, D>::update() {
  auto& a = this->a();
  auto& b = this->b();
  const auto force = this->calculateForce();
  if (!a.fixed()) {
    a.moveTo((a.position() + force * a.inverse_mass()).limit(limit_));
  }
  if (!b.fixed()) {
    b.moveTo((b.position() - force * b.inverse_mass()).limit(limit_));
  }
}

}  // namespace spring

using spring::ConstrainedSpring;
using spring::ConstrainedSpring2;
using spring::ConstrainedSpring3;

using spring::ConstrainedSpring2f;
using spring::ConstrainedSpring2d;
using spring::ConstrainedSpring3f;
using spring::ConstrainedSpring3d;

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_SPRING_CONSTRAINED_SPRING_H_
