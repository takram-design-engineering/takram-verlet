//
//  takram/verlet/container.h
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
#ifndef TAKRAM_VERLET_CONTAINER_H_
#define TAKRAM_VERLET_CONTAINER_H_

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <functional>
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>

#include "takram/verlet/reference_iterator.h"

namespace takram {
namespace verlet {

template <class Base>
class Container {
 public:
  using Type = Base;
  using Iterator = ReferenceIterator<
      typename std::vector<std::unique_ptr<Base>>::iterator>;
  using ConstIterator = ReferenceIterator<
      typename std::vector<std::unique_ptr<Base>>::const_iterator>;
  using ReverseIterator = std::reverse_iterator<Iterator>;
  using ConstReverseIterator = std::reverse_iterator<ConstIterator>;
  using Configurator = std::function<void(Base *)>;

 public:
  Container() = default;
  explicit Container(const Configurator& configurator);

  // Disallow copy semantics
  Container(const Container&) = delete;
  Container& operator=(const Container&) = delete;

  // Move semantics
  Container(Container&&) = default;
  Container& operator=(Container&&) = default;

  // Comparison
  bool operator==(const Container& other) const;
  bool operator!=(const Container& other) const;

  // Modifiers
  template <class... Args>
  Base& emplace(Args&&... args);
  template <class Derived, class... Args>
  Derived& emplace(Args&&... args);
  template <class Derived>
  Derived& add(Derived&& item);
  template <class Derived>
  Derived& add(std::unique_ptr<Derived>&& item);
  std::unique_ptr<Base> remove(const Base& item);
  Iterator erase(Iterator pos);
  Iterator erase(ConstIterator pos);
  void swap(Container& other);
  void clear();

  // Element access
  Base& operator[](std::size_t index) { return *items_[index]; }
  const Base& operator[](std::size_t index) const { return *items_[index]; }
  Base& at(std::size_t index) { return *items_.at(index); }
  const Base& at(std::size_t index) const { return *items_(index); }
  Base& front() { return *items_.front(); }
  const Base& front() const { return *items_.front(); }
  Base& back() { return *items_.back(); }
  const Base& back() const { return *items_.back(); }

  // Capacity
  std::size_t size() const { return items_.size(); }
  std::size_t max_size() const { return items_.max_size(); }
  bool empty() const { return items_.empty(); }

  // Configurator
  const std::function<void(Base *)>& configurator() const;

  // Iterator
  Iterator begin() { return Iterator(std::begin(items_)); }
  ConstIterator begin() const { return ConstIterator(std::begin(items_)); }
  Iterator end() { return Iterator(std::end(items_)); }
  ConstIterator end() const { return ConstIterator(std::end(items_)); }
  ReverseIterator rbegin() { return ReverseIterator(begin()); }
  ConstReverseIterator rbegin() const { return ConstReverseIterator(begin()); }
  ReverseIterator rend() { return ReverseIterator(end()); }
  ConstReverseIterator rend() const { return ConstReverseIterator(end()); }

 private:
  std::vector<std::unique_ptr<Base>> items_;
  Configurator configurator_;
};

#pragma mark -

template <class Base>
inline Container<Base>::Container(const Configurator& configurator)
    : configurator_(configurator) {}

#pragma mark Comparison

template <class Base>
inline bool Container<Base>::operator==(const Container& other) const {
  return items_ == other.items_;
}

template <class Base>
inline bool Container<Base>::operator!=(const Container& other) const {
  return !operator==(other);
}

#pragma mark Modifiers

template <class Base>
template <class... Args>
inline Base& Container<Base>::emplace(Args&&... args) {
  static_assert(!std::is_abstract<Base>::value, "Base type is abstract");
  return add(std::make_unique<Base>(std::forward<Args>(args)...));
}

template <class Base>
template <class Derived, class... Args>
inline Derived& Container<Base>::emplace(Args&&... args) {
  static_assert(std::is_convertible<Derived&, Base&>::value,
                "Type must be convertible to the base type");
  return add(std::make_unique<Derived>(std::forward<Args>(args)...));
}

template <class Base>
template <class Derived>
inline Derived& Container<Base>::add(Derived&& item) {
  using Decay = std::decay_t<Derived>;
  static_assert(std::is_convertible<Decay&, Base&>::value,
                "Type must be convertible to the base type");
  return add(std::make_unique<Decay>(std::forward<Derived>(item)));
}

template <class Base>
template <class Derived>
inline Derived& Container<Base>::add(std::unique_ptr<Derived>&& item) {
  static_assert(std::is_convertible<Derived&, Base&>::value,
                "Type must be convertible to the base type");
  assert(item);
  auto& reference = *item;
  items_.emplace_back(std::move(item));
  if (configurator_) {
    configurator_(&reference);
  }
  return reference;
}

template <class Base>
inline std::unique_ptr<Base> Container<Base>::remove(const Base& item) {
  const auto predicate = [&item](const auto& pointer) {
    return &item == pointer.get();
  };
  const auto end = std::end(items_);
  const auto itr = std::find_if(std::begin(items_), end, predicate);
  if (itr != end) {
    std::unique_ptr<Base> erased(std::move(*itr));
    items_.erase(itr);
    return std::move(erased);
  }
  return std::unique_ptr<Base>();
}

template <class Base>
inline typename Container<Base>::Iterator
    Container<Base>::erase(Iterator pos) {
  return Iterator(items_.erase(pos.get()));
}

template <class Base>
inline typename Container<Base>::Iterator
    Container<Base>::erase(ConstIterator pos) {
  return Iterator(items_.erase(pos.get()));
}

template <class Base>
inline void Container<Base>::swap(Container& other) {
  items_.swap(other.items_);
  configurator_.swap(other.configurator_);
}

template <class Base>
inline void Container<Base>::clear() {
  items_.clear();
}

#pragma mark Configurator

template <class Base>
inline const std::function<void(Base *)>&
    Container<Base>::configurator() const {
  return configurator_;
}

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_CONTAINER_H_
