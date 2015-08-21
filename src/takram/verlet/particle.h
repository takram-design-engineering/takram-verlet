//
//  takram/verlet/particle.h
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
#ifndef TAKRAM_VERLET_PARTICLE_H_
#define TAKRAM_VERLET_PARTICLE_H_

#include <limits>
#include <memory>
#include <type_traits>
#include <utility>

#include "takram/math/vector.h"
#include "takram/verlet/container.h"

namespace takram {
namespace verlet {

namespace behavior {

template <class T, int D>
class Behavior;

}  // namespace behavior

using behavior::Behavior;

namespace constraint {

template <class T, int D>
class Constraint;

}  // namespace constraint

using constraint::Constraint;

template <class T, int D>
class Particle {
  static_assert(!std::is_integral<T>::value, "Integral type is not supported");

 public:
  using Type = T;
  using Behavior = Behavior<T, D>;
  using Constraint = Constraint<T, D>;
  using Behaviors = Container<Behavior>;
  using Constraints = Container<Constraint>;
  using BehaviorIterator = typename Behaviors::Iterator;
  using ConstraintIterator = typename Constraints::Iterator;
  using BehaviorConstIterator = typename Behaviors::ConstIterator;
  using ConstraintConstIterator = typename Constraints::ConstIterator;
  static constexpr const int dimensions = D;

 public:
  Particle();
  explicit Particle(const Vec<T, D>& position, T mass = 1);
  virtual ~Particle() = default;

  // Disallow copy semantics
  Particle(const Particle&) = delete;
  Particle& operator=(const Particle&) = delete;

  // Move semantics
  Particle(Particle&&) = default;
  Particle& operator=(Particle&&) = default;

  // Particle
  virtual void update();
  void applyBehaviors();
  void applyConstraints();

  // Position
  void moveBy(const Vec<T, D>& delta);
  void moveTo(const Vec<T, D>& position);

  // Force
  void addForce(const Vec<T, D>& velocity);
  void scaleForce(T scalar);
  void resetForce();

  // Velocity
  void addVelocity(const Vec<T, D>& velocity);
  void scaleVelocity(T scalar);
  void resetVelocity();

  // Behaviors
  template <class U, class... Args>
  U& emplaceBehavior(Args&&... args);
  template <class U>
  U& addBehavior(U&& behavior);
  template <class U>
  U& addBehavior(std::unique_ptr<U>&& behavior);
  std::unique_ptr<Behavior> removeBehavior(const Behavior& behavior);
  BehaviorIterator eraseBehavior(BehaviorIterator pos);
  BehaviorIterator eraseBehavior(BehaviorConstIterator pos);
  void clearBehaviors();

  // Constraints
  template <class U, class... Args>
  U& emplaceConstraint(Args&&... args);
  template <class U>
  U& addConstraint(U&& constraint);
  template <class U>
  U& addConstraint(std::unique_ptr<U>&& constraint);
  std::unique_ptr<Constraint> removeConstraint(const Constraint& constraint);
  ConstraintIterator eraseConstraint(ConstraintIterator pos);
  ConstraintIterator eraseConstraint(ConstraintConstIterator pos);
  void clearConstraints();

  // Mobility
  Particle& free();
  Particle& fix();
  bool fixed() const { return fixed_; }

  // Attributes
  const Vec<T, D>& position() const { return current_; }
  void set_position(const Vec<T, D>& value) { current_ = value; }
  const Vec<T, D>& force() const { return force_; }
  void set_force(const Vec<T, D>& value) { force_ = value; }
  Vec<T, D> velocity() const;
  void set_velocity(const Vec<T, D>& value);
  T mass() const { return mass_; }
  void set_mass(T value);
  T inverse_mass() const { return inverse_mass_; }

  // Aggregations
  const Behaviors& behaviors() const { return behaviors_; }
  Behaviors& behaviors() { return behaviors_; }
  const Constraints& constraints() const { return constraints_; }
  Constraints& constraints() { return constraints_; }

 private:
  Vec<T, D> current_;
  Vec<T, D> previous_;
  Vec<T, D> force_;
  T mass_;
  T inverse_mass_;
  bool fixed_;
  Container<Behavior> behaviors_;
  Container<Constraint> constraints_;
};

template <class T>
using Particle2 = Particle<T, 2>;
template <class T>
using Particle3 = Particle<T, 3>;

using Particle2f = Particle2<float>;
using Particle2d = Particle2<double>;
using Particle3f = Particle3<float>;
using Particle3d = Particle3<double>;

}  // namespace verlet
}  // namespace takram

#include "takram/verlet/behavior.h"
#include "takram/verlet/constraint.h"

namespace takram {
namespace verlet {

#pragma mark -

template <class T, int D>
inline Particle<T, D>::Particle() : Particle(Vec<T, D>()) {}

template <class T, int D>
inline Particle<T, D>::Particle(const Vec<T, D>& position, T mass)
    : current_(position),
      previous_(position),
      mass_(),
      fixed_() {
  set_mass(mass);
}

#pragma mark Particle

template <class T, int D>
inline void Particle<T, D>::update() {
  if (fixed_) {
    return;
  }
  applyBehaviors();
  applyConstraints();

  // Verlet integration without velocities
  const auto position = current_;
  current_ += current_ - previous_ + force_ * mass_;
  previous_ = position;
  force_.reset();
}

template <class T, int D>
inline void Particle<T, D>::applyBehaviors() {
  for (auto& behavior : behaviors_) {
    behavior.apply(this);
  }
}

template <class T, int D>
inline void Particle<T, D>::applyConstraints() {
  for (auto& constraint : constraints_) {
    constraint.apply(this);
  }
}

#pragma mark Position

template <class T, int D>
inline void Particle<T, D>::moveBy(const Vec<T, D>& delta) {
  current_ += delta;
}

template <class T, int D>
inline void Particle<T, D>::moveTo(const Vec<T, D>& position) {
  current_ = position;
}

#pragma mark Force

template <class T, int D>
inline void Particle<T, D>::addForce(const Vec<T, D>& force) {
  force_ += force;
}

template <class T, int D>
inline void Particle<T, D>::scaleForce(T scalar) {
  force_ *= scalar;
}

template <class T, int D>
inline void Particle<T, D>::resetForce() {
  force_.reset();
}

#pragma mark Velocity

template <class T, int D>
inline void Particle<T, D>::addVelocity(const Vec<T, D>& velocity) {
  previous_ -= velocity;
}

template <class T, int D>
inline void Particle<T, D>::scaleVelocity(T scalar) {
  previous_.lerp(current_, 1 - scalar);
}

template <class T, int D>
inline void Particle<T, D>::resetVelocity() {
  previous_ = current_;
}

#pragma mark Behaviors

template <class T, int D>
template <class U, class... Args>
inline U& Particle<T, D>::emplaceBehavior(Args&&... args) {
  return behaviors_.emplace(std::forward<Args>(args)...);
}

template <class T, int D>
template <class U>
inline U& Particle<T, D>::addBehavior(U&& behavior) {
  return behaviors_.add(std::forward<U>(behavior));
}

template <class T, int D>
template <class U>
inline U& Particle<T, D>::addBehavior(std::unique_ptr<U>&& behavior) {
  return behaviors_.add(std::move(behavior));
}

template <class T, int D>
inline std::unique_ptr<Behavior<T, D>> Particle<T, D>::removeBehavior(
    const Behavior& behavior) {
  return behaviors_.remove(behavior);
}

template <class T, int D>
inline typename Particle<T, D>::BehaviorIterator
    Particle<T, D>::eraseBehavior(BehaviorIterator pos) {
  return behaviors_.erase(pos);
}

template <class T, int D>
inline typename Particle<T, D>::BehaviorIterator
    Particle<T, D>::eraseBehavior(BehaviorConstIterator pos) {
  return behaviors_.erase(pos);
}

template <class T, int D>
inline void Particle<T, D>::clearBehaviors() {
  behaviors_.clear();
}

#pragma mark Constraints

template <class T, int D>
template <class U, class... Args>
inline U& Particle<T, D>::emplaceConstraint(Args&&... args) {
  return constraints_.emplace(std::forward<Args>(args)...);
}

template <class T, int D>
template <class U>
inline U& Particle<T, D>::addConstraint(U&& constraint) {
  return constraints_.add(std::forward<U>(constraint));
}

template <class T, int D>
template <class U>
inline U& Particle<T, D>::addConstraint(std::unique_ptr<U>&& constraint) {
  return constraints_.add(std::move(constraint));
}

template <class T, int D>
inline std::unique_ptr<Constraint<T, D>> Particle<T, D>::removeConstraint(
    const Constraint& constraint) {
  return constraints_.remove(constraint);
}

template <class T, int D>
inline typename Particle<T, D>::ConstraintIterator
    Particle<T, D>::eraseConstraint(ConstraintIterator pos) {
  return constraints_.erase(pos);
}

template <class T, int D>
inline typename Particle<T, D>::ConstraintIterator
    Particle<T, D>::eraseConstraint(ConstraintConstIterator pos) {
  return constraints_.erase(pos);
}

template <class T, int D>
inline void Particle<T, D>::clearConstraints() {
  constraints_.clear();
}

#pragma mark Mobility

template <class T, int D>
inline Particle<T, D>& Particle<T, D>::free() {
  fixed_ = false;
  return *this;
}

template <class T, int D>
inline Particle<T, D>& Particle<T, D>::fix() {
  fixed_ = true;
  return *this;
}

#pragma mark Attributes

template <class T, int D>
inline Vec<T, D> Particle<T, D>::velocity() const {
  return current_ - previous_;
}

template <class T, int D>
inline void Particle<T, D>::set_velocity(const Vec<T, D>& value) {
  previous_ = current_ - value;
}

template <class T, int D>
inline void Particle<T, D>::set_mass(T value) {
  if (value) {
    mass_ = value;
    inverse_mass_ = 1 / value;
  } else {
    mass_ = std::numeric_limits<T>::epsilon();
    inverse_mass_ = T();
  }
}

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_PARTICLE_H_
