//
//  takram/verlet/physics.h
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
#ifndef TAKRAM_VERLET_PHYSICS_H_
#define TAKRAM_VERLET_PHYSICS_H_

#include <cassert>
#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

#include "takram/math/vector.h"
#include "takram/verlet/behavior/behavior.h"
#include "takram/verlet/behavior/gravity_behavior.h"
#include "takram/verlet/constraint/constraint.h"
#include "takram/verlet/container.h"
#include "takram/verlet/particle.h"
#include "takram/verlet/spring/spring.h"

namespace takram {
namespace verlet {

template <class T, int D>
class Physics {
  static_assert(!std::is_integral<T>::value, "Integral type is not supported");

 public:
  using Type = T;
  using Particle = Particle<T, D>;
  using Spring = Spring<T, D>;
  using Behavior = Behavior<T, D>;
  using Constraint = Constraint<T, D>;
  using Particles = Container<Particle>;
  using Springs = Container<Spring>;
  using Behaviors = Container<Behavior>;
  using Constraints = Container<Constraint>;
  using ParticleIterator = typename Particles::Iterator;
  using SpringIterator = typename Springs::Iterator;
  using BehaviorIterator = typename Behaviors::Iterator;
  using ConstraintIterator = typename Constraints::Iterator;
  using ParticleConstIterator = typename Particles::ConstIterator;
  using SpringConstIterator = typename Springs::ConstIterator;
  using BehaviorConstIterator = typename Behaviors::ConstIterator;
  using ConstraintConstIterator = typename Constraints::ConstIterator;
  static constexpr const int dimensions = D;

 public:
  Physics();
  Physics(unsigned int iterations, T drag, T time_step);
  Physics(const Vec<T, D>& gravity,
          unsigned int iterations,
          T drag,
          T time_step);

  // Disallow copy semantics
  Physics(const Physics&) = delete;
  Physics& operator=(const Physics&) = delete;

  // Move semantics
  Physics(Physics&&) = default;
  Physics& operator=(Physics&&) = default;

  // Physics
  void update();
  void reset();

  // Particles
  template <class U, class... Args>
  U& emplaceParticle(Args&&... args);
  template <class U>
  U& addParticle(U&& particle);
  template <class U>
  U& addParticle(std::unique_ptr<U>&& particle);
  std::unique_ptr<Particle> removeParticle(const Particle& particle);
  ParticleIterator eraseParticle(ParticleIterator pos);
  ParticleIterator eraseParticle(ParticleConstIterator pos);
  void clearParticles();

  // Springs
  template <class U, class... Args>
  U& emplaceSpring(Args&&... args);
  template <class U>
  U& addSpring(U&& spring);
  template <class U>
  U& addSpring(std::unique_ptr<U>&& spring);
  std::unique_ptr<Spring> removeSpring(const Spring& spring);
  std::unique_ptr<Spring> removeSpringElements(const Spring& spring);
  SpringIterator eraseSpring(SpringIterator pos);
  SpringIterator eraseSpring(SpringConstIterator pos);
  void clearSprings();

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

  // Parameters
  T drag() const { return drag_; }
  void set_drag(T value);
  unsigned int iterations() const { return iterations_; }
  void set_iterations(unsigned int value);
  T time_step() const { return time_step_; }
  void set_time_step(T value);

  // Aggregations
  const Particles& particles() const { return particles_; }
  Particles& particles() { return particles_; }
  const Springs& springs() const { return springs_; }
  Springs& springs() { return springs_; }
  const Behaviors& behaviors() const { return behaviors_; }
  Behaviors& behaviors() { return behaviors_; }
  const Constraints& constraints() const { return constraints_; }
  Constraints& constraints() { return constraints_; }

 protected:
  void updateParticles();
  void updateSprings();
  void applyBehaviors();
  void applyConstraints();

  // Configurators
  void configureBehavior(Behavior *behavior);
  void configureConstraint(Constraint *constraint);

 private:
  T drag_;
  unsigned int iterations_;
  T time_step_;
  Container<Particle> particles_;
  Container<Spring> springs_;
  Container<Behavior> behaviors_;
  Container<Constraint> constraints_;
};

template <class T>
using Physics2 = Physics<T, 2>;
template <class T>
using Physics3 = Physics<T, 3>;

using Physics2f = Physics2<float>;
using Physics2d = Physics2<double>;
using Physics3f = Physics3<float>;
using Physics3d = Physics3<double>;

#pragma mark -

template <class T, int D>
inline Physics<T, D>::Physics() : Physics(Vec<T, D>(), 50, 0, 1) {}

template <class T, int D>
inline Physics<T, D>::Physics(unsigned int iterations, T drag, T time_step)
    : iterations_(iterations),
      drag_(1 - drag),
      time_step_(time_step),
      behaviors_(std::bind(&Physics::configureBehavior, this,
                           std::placeholders::_1)),
      constraints_(std::bind(&Physics::configureConstraint, this,
                             std::placeholders::_1)) {}

template <class T, int D>
inline Physics<T, D>::Physics(const Vec<T, D>& gravity,
                              unsigned int iterations,
                              T drag,
                              T time_step)
    : Physics(iterations, drag, time_step) {
  if (!gravity.empty()) {
    behaviors_.template emplace<GravityBehavior<T, D>>(gravity);
  }
}

#pragma mark Physics

template <class T, int D>
inline void Physics<T, D>::update() {
  applyBehaviors();
  updateParticles();
  updateSprings();
  applyConstraints();
}

template <class T, int D>
inline void Physics<T, D>::reset() {
  constraints_.clear();
  behaviors_.clear();
  springs_.clear();
  particles_.clear();
}

template <class T, int D>
inline void Physics<T, D>::updateParticles() {
  for (auto& particle : particles_) {
    particle.scaleVelocity(drag_);
    particle.update();
  }
}

template <class T, int D>
inline void Physics<T, D>::updateSprings() {
  for (auto i = iterations_; i; --i) {
    for (auto& spring : springs_) {
      spring.update();
      if (i == 1) {
        spring.applyConstraints();
      }
    }
  }
}

template <class T, int D>
inline void Physics<T, D>::applyBehaviors() {
  for (auto& behavior : behaviors_) {
    for (auto& particle : particles_) {
      behavior.apply(&particle);
    }
  }
}

template <class T, int D>
inline void Physics<T, D>::applyConstraints() {
  for (auto& constraint : constraints_) {
    for (auto& particle : particles_) {
      constraint.apply(&particle);
    }
  }
}

#pragma mark Particles

template <class T, int D>
template <class U, class... Args>
inline U& Physics<T, D>::emplaceParticle(Args&&... args) {
  return particles_.emplace(std::forward<Args>(args)...);
}

template <class T, int D>
template <class U>
inline U& Physics<T, D>::addParticle(U&& particle) {
  return particles_.add(std::forward<U>(particle));
}

template <class T, int D>
template <class U>
inline U& Physics<T, D>::addParticle(std::unique_ptr<U>&& particle) {
  return particles_.add(std::move(particle));
}

template <class T, int D>
inline std::unique_ptr<Particle<T, D>> Physics<T, D>::removeParticle(
    const Particle& particle) {
  return particles_.remove(particle);
}

template <class T, int D>
inline typename Physics<T, D>::ParticleIterator
    Physics<T, D>::eraseParticle(ParticleIterator pos) {
  return particles_.erase(pos);
}

template <class T, int D>
inline typename Physics<T, D>::ParticleIterator
    Physics<T, D>::eraseParticle(ParticleConstIterator pos) {
  return particles_.erase(pos);
}

template <class T, int D>
inline void Physics<T, D>::clearParticles() {
  particles_.clear();
}

#pragma mark Springs

template <class T, int D>
template <class U, class... Args>
inline U& Physics<T, D>::emplaceSpring(Args&&... args) {
  return springs_.emplace(std::forward<Args>(args)...);
}

template <class T, int D>
template <class U>
inline U& Physics<T, D>::addSpring(U&& spring) {
  return springs_.add(std::forward<U>(spring));
}

template <class T, int D>
template <class U>
inline U& Physics<T, D>::addSpring(std::unique_ptr<U>&& spring) {
  return springs_.add(std::move(spring));
}

template <class T, int D>
inline std::unique_ptr<Spring<T, D>> Physics<T, D>::removeSpring(
    const Spring& spring) {
  return springs_.remove(spring);
}

template <class T, int D>
inline std::unique_ptr<Spring<T, D>> Physics<T, D>::removeSpringElements(
    const Spring& spring) {
  auto removed = springs_.remove(spring);
  if (removed) {
    removeParticle(removed->a());
    removeParticle(removed->b());
  }
  return std::move(removed);
}

template <class T, int D>
inline typename Physics<T, D>::SpringIterator
    Physics<T, D>::eraseSpring(SpringIterator pos) {
  return springs_.erase(pos);
}

template <class T, int D>
inline typename Physics<T, D>::SpringIterator
    Physics<T, D>::eraseSpring(SpringConstIterator pos) {
  return springs_.erase(pos);
}

template <class T, int D>
inline void Physics<T, D>::clearSprings() {
  springs_.clear();
}

#pragma mark Behaviors

template <class T, int D>
template <class U, class... Args>
inline U& Physics<T, D>::emplaceBehavior(Args&&... args) {
  return behaviors_.emplace(std::forward<Args>(args)...);
}

template <class T, int D>
template <class U>
inline U& Physics<T, D>::addBehavior(U&& behavior) {
  return behaviors_.add(std::forward<U>(behavior));
}

template <class T, int D>
template <class U>
inline U& Physics<T, D>::addBehavior(std::unique_ptr<U>&& behavior) {
  return behaviors_.add(std::move(behavior));
}

template <class T, int D>
inline std::unique_ptr<Behavior<T, D>> Physics<T, D>::removeBehavior(
    const Behavior& behavior) {
  return behaviors_.remove(behavior);
}

template <class T, int D>
inline typename Physics<T, D>::BehaviorIterator
    Physics<T, D>::eraseBehavior(BehaviorIterator pos) {
  return behaviors_.erase(pos);
}

template <class T, int D>
inline typename Physics<T, D>::BehaviorIterator
    Physics<T, D>::eraseBehavior(BehaviorConstIterator pos) {
  return behaviors_.erase(pos);
}

template <class T, int D>
inline void Physics<T, D>::clearBehaviors() {
  behaviors_.clear();
}

#pragma mark Constraints

template <class T, int D>
template <class U, class... Args>
inline U& Physics<T, D>::emplaceConstraint(Args&&... args) {
  return constraints_.emplace(std::forward<Args>(args)...);
}

template <class T, int D>
template <class U>
inline U& Physics<T, D>::addConstraint(U&& constraint) {
  return constraints_.add(std::forward<U>(constraint));
}

template <class T, int D>
template <class U>
inline U& Physics<T, D>::addConstraint(std::unique_ptr<U>&& constraint) {
  return constraints_.add(std::move(constraint));
}

template <class T, int D>
inline std::unique_ptr<Constraint<T, D>> Physics<T, D>::removeConstraint(
    const Constraint& constraint) {
  return constraints_.remove(constraint);
}

template <class T, int D>
inline typename Physics<T, D>::ConstraintIterator
    Physics<T, D>::eraseConstraint(ConstraintIterator pos) {
  return constraints_.erase(pos);
}

template <class T, int D>
inline typename Physics<T, D>::ConstraintIterator
    Physics<T, D>::eraseConstraint(ConstraintConstIterator pos) {
  return constraints_.erase(pos);
}

template <class T, int D>
inline void Physics<T, D>::clearConstraints() {
  constraints_.clear();
}

#pragma mark Configurators

template <class T, int D>
inline void Physics<T, D>::configureBehavior(Behavior *behavior) {
  assert(behavior);
  behavior->configure(*this);
}

template <class T, int D>
inline void Physics<T, D>::configureConstraint(Constraint *constraint) {
  assert(constraint);
  constraint->configure(*this);
}

#pragma mark Parameters

template <class T, int D>
inline void Physics<T, D>::set_drag(T value) {
  drag_ = 1 - value;
}

template <class T, int D>
inline void Physics<T, D>::set_iterations(unsigned int value) {
  iterations_ = value;
}

template <class T, int D>
inline void Physics<T, D>::set_time_step(T value) {
  time_step_ = value;
  for (auto& behavior : behaviors_) {
    behavior.configure(*this);
  }
  for (auto& constraint : constraints_) {
    constraint.configure(*this);
  }
}

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_PHYSICS_H_
