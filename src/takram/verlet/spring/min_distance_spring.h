//
//  takram/verlet/spring/min_distance_spring.h
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
#ifndef TAKRAM_VERLET_SPRING_MIN_DISTANCE_SPRING_H_
#define TAKRAM_VERLET_SPRING_MIN_DISTANCE_SPRING_H_

#include "takram/verlet/particle.h"
#include "takram/verlet/spring/spring.h"

namespace takram {
namespace verlet {
namespace spring {

template <class T, int D>
class MinDistanceSpring : public Spring<T, D> {
 public:
  using Spring<T, D>::Spring;

  // Copy semantics
  MinDistanceSpring(const MinDistanceSpring&) = default;
  MinDistanceSpring& operator=(const MinDistanceSpring&) = default;

  // Updates
  void update() override;
};

template <class T>
using MinDistanceSpring2 = MinDistanceSpring<T, 2>;
template <class T>
using MinDistanceSpring3 = MinDistanceSpring<T, 3>;

using MinDistanceSpring2f = MinDistanceSpring2<float>;
using MinDistanceSpring2d = MinDistanceSpring2<double>;
using MinDistanceSpring3f = MinDistanceSpring3<float>;
using MinDistanceSpring3d = MinDistanceSpring3<double>;

#pragma mark -

#pragma mark Updates

template <class T, int D>
inline void MinDistanceSpring<T, D>::update() {
  const auto& a = this->a().position();
  const auto& b = this->b().position();
  if (b.distanceSquared(a) < this->rest_length_squared()) {
    Spring<T, D>::update();
  }
}

}  // namespace spring

using spring::MinDistanceSpring;
using spring::MinDistanceSpring2;
using spring::MinDistanceSpring3;

using spring::MinDistanceSpring2f;
using spring::MinDistanceSpring2d;
using spring::MinDistanceSpring3f;
using spring::MinDistanceSpring3d;

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_SPRING_MIN_DISTANCE_SPRING_H_
