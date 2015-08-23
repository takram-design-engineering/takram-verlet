Verlet
======

An unofficial C++ port of toxiclib's Verlet physics (WIP).

### Classes

- [`takram::verlet::Physics`](src/takram/verlet/physics.h)
- [`takram::verlet::Particle`](src/takram/verlet/particle.h)
- [`takram::verlet::Spring`](src/takram/verlet/spring.h)
- [`takram::verlet::Behavior`](src/takram/verlet/behavior/behavior.h)
- [`takram::verlet::Constraint`](src/takram/verlet/constraint/constraint.h)

## Setup Guide

Run "setup.sh" inside "script" directory to initialize submodules and build dependant libraries.

### Dependencies

- [Math](https://github.com/takram-design-engineering/takram-math)

### Submodules

- [Google Testing Framework](https://chromium.googlesource.com/external/googletest)

## License

LGPL License

Copyright (C) 2015 Shota Matsuda

Based on toxiclibs http://toxiclibs.org/<br>
Copyright (C) 2006-2011 Karsten Schmidt<br>
Distributed under LGPL License

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
