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
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
//

#include "takram/verlet.h"

namespace takram {
namespace verlet {

const double version_number = 1.0;
const unsigned char version_string[] = "1.0";

// For static analysis
template class Physics<double, 2>;
template class Physics<double, 3>;
template class Particle<double, 2>;
template class Particle<double, 3>;
template class spring::Spring<double, 2>;
template class spring::Spring<double, 3>;
template class spring::PullBackSpring<double, 2>;
template class spring::PullBackSpring<double, 3>;
template class spring::ConstrainedSpring<double, 2>;
template class spring::ConstrainedSpring<double, 3>;
template class spring::MinDistanceSpring<double, 2>;
template class spring::MinDistanceSpring<double, 3>;
template class behavior::AttractionBehavior<double, 2>;
template class behavior::AttractionBehavior<double, 3>;
template class behavior::ConstantForceBehavior<double, 2>;
template class behavior::ConstantForceBehavior<double, 3>;
template class behavior::GravityBehavior<double, 2>;
template class behavior::GravityBehavior<double, 3>;
template class constraint::AngularConstraint<double>;
template class constraint::AxisConstraint<double>;
template class constraint::CircularConstraint<double>;
template class constraint::MaxConstraint<double>;
template class constraint::MinConstraint<double>;

}  // namespace verlet
}  // namespace takram
