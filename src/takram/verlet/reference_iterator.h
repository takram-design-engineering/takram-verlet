//
//  takram/verlet/reference_iterator.h
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
#ifndef TAKRAM_VERLET_REFERENCE_ITERATOR_H_
#define TAKRAM_VERLET_REFERENCE_ITERATOR_H_

#include <iterator>

namespace takram {
namespace verlet {

template <class Iterator>
class ReferenceIterator final
    : public std::iterator<std::forward_iterator_tag,
                           typename Iterator::value_type::element_type,
                           typename Iterator::difference_type,
                           typename Iterator::value_type::pointer,
                           typename Iterator::value_type::element_type&> {
 public:
  using Type = typename Iterator::value_type::element_type;
  using Pointer = typename Iterator::value_type::pointer;
  using Reference = typename Iterator::value_type::element_type&;

 public:
  ReferenceIterator();
  explicit ReferenceIterator(Iterator iterator);

  // Copy semantics
  ReferenceIterator(const ReferenceIterator&) = default;
  ReferenceIterator& operator=(const ReferenceIterator&) = default;

  // Comparison
  bool operator==(const ReferenceIterator& other) const;
  bool operator!=(const ReferenceIterator& other) const;

  // Iterator
  Reference operator*() const;
  Pointer operator->() const { return &operator*(); }
  ReferenceIterator& operator++();
  const ReferenceIterator& operator++(int);
  const Iterator& get() const { return iterator_; }

 private:
  Iterator iterator_;
};

#pragma mark -

template <class Iterator>
inline ReferenceIterator<Iterator>::ReferenceIterator() {}

template <class Iterator>
inline ReferenceIterator<Iterator>::ReferenceIterator(Iterator iterator)
    : iterator_(iterator) {}

#pragma mark Comparison

template <class Iterator>
inline bool ReferenceIterator<Iterator>::operator==(
    const ReferenceIterator& other) const {
  return iterator_ == other.iterator_;
}

template <class Iterator>
inline bool ReferenceIterator<Iterator>::operator!=(
    const ReferenceIterator& other) const {
  return iterator_ != other.iterator_;
}

#pragma mark Iterator

template <class Iterator>
inline typename ReferenceIterator<Iterator>::Reference
    ReferenceIterator<Iterator>::operator*() const {
  assert(*iterator_);
  return **iterator_;
}

template <class Iterator>
inline typename ReferenceIterator<Iterator>::ReferenceIterator&
    ReferenceIterator<Iterator>::operator++() {
  ++iterator_;
  return *this;
}

template <class Iterator>
inline const typename ReferenceIterator<Iterator>::ReferenceIterator&
    ReferenceIterator<Iterator>::operator++(int) {
  ReferenceIterator result(*this);
  operator++();
  return result;
}

}  // namespace verlet
}  // namespace takram

#endif  // TAKRAM_VERLET_REFERENCE_ITERATOR_H_
