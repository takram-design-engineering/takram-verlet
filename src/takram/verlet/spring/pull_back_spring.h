//
//  takram/verlet/spring/pull_back_spring.h
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
#ifndef TAKRAM_VERLET_SPRING_PULL_BACK_SPRING_H_
#define TAKRAM_VERLET_SPRING_PULL_BACK_SPRING_H_

#include "takram/verlet/particle.h"
#include "takram/verlet/spring/spring.h"

namespace takram {
namespace verlet {
namespace spring {

template <class T, int D>
class PullBackSpring : public Spring<T, D> {
 public:
  PullBackSpring(Particle<T, D> *a, Particle<T, D> *b, T strength);

  // Copy semantics
  PullBackSpring(const PullBackSpring&) = default;
  PullBackSpring& operator=(const PullBackSpring&) = default;

  // Updates
  void update() override;
};

template <class T>
using PullBackSpring2 = PullBackSpring<T, 2>;
template <class T>
using PullBackSpring3 = PullBackSpring<T, 3>;

using PullBackSpring2f = PullBackSpring2<float>;
using PullBackSpring2d = PullBackSpring2<double>;
using PullBackSpring3f = PullBackSpring3<float>;
using PullBackSpring3d = PullBackSpring3<double>;

#pragma mark -

template <class T, int D>
inline PullBackSpring<T, D>::PullBackSpring(Particle<T, D> *a,
                                            Particle<T, D> *b,
                                            T strength)
    : Spring<T, D>(a, b, 0.5, strength) {
  this->a().fix();
}

#pragma mark Updates

template <class T, int D>
inline void PullBackSpring<T, D>::update() {
  const auto& a = this->a().position();
  const auto& b = this->b().position();
  if (b.distanceSquared(a) > this->rest_length_squared()) {
    Spring<T, D>::update();
  }
}

}  // namespace spring

using spring::PullBackSpring;
using spring::PullBackSpring2;
using spring::PullBackSpring3;

using spring::PullBackSpring2f;
using spring::PullBackSpring2d;
using spring::PullBackSpring3f;
using spring::PullBackSpring3d;

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_SPRING_PULL_BACK_SPRING_H_
