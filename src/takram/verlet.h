//
//  takram/verlet.h
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
#ifndef TAKRAM_VERLET_H_
#define TAKRAM_VERLET_H_

namespace takram {
namespace verlet {

extern const double version_number;
extern const unsigned char version_string[];

}  // namespace verlet
}  // namespace takram

#include "takram/verlet/behavior.h"
#include "takram/verlet/constraint.h"
#include "takram/verlet/container.h"
#include "takram/verlet/particle.h"
#include "takram/verlet/physics.h"
#include "takram/verlet/spring.h"

#endif  // TAKRAM_VERLET_H_
