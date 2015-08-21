//
//  takram/verlet/behavior/behavior.h
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
#ifndef TAKRAM_VERLET_BEHAVIOR_BEHAVIOR_H_
#define TAKRAM_VERLET_BEHAVIOR_BEHAVIOR_H_

#include "takram/verlet/particle.h"

namespace takram {
namespace verlet {

template <class T, int D>
class Physics;

namespace behavior {

template <class T, int D>
class Behavior {
 public:
  using Type = T;
  static constexpr const int dimensions = D;

 public:
  virtual ~Behavior() = default;
  virtual void apply(Particle<T, D> *particle) = 0;
  virtual void configure(const Physics<T, D>& physics) {}
};

template <class T>
using Behavior2 = Behavior<T, 2>;
template <class T>
using Behavior3 = Behavior<T, 3>;

}  // namespace behavior

using behavior::Behavior;
using behavior::Behavior2;
using behavior::Behavior3;

}  // namespace verlet
}  // namespace takram

#include "takram/verlet/physics.h"

#endif  // TAKRAM_VERLET_BEHAVIOR_BEHAVIOR_H_
