/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

/** \defgroup mrpt_math_grp [mrpt-math]

Math C++ library: vectors and matrices, probability distributions, statistics,
geometry, etc.

[TOC]

# Library `mrpt-math`
<small> [New in MRPT 2.0.0] </small>

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-math-dev

Main classes and concepts associated with this library:

Lightweight geometry entities: Write me!

Comparison: lightweight vs. {CPose*, CPoint*}: (Move to a new doc page?)
 - Both can be serialized, but CPose* are CSerializable-based.
 - CPose* require aligned memory (they hold Eigen containers).
 - CPose* include a cache for precomputed cos/sin values, so they are preferable
when doing many pose (+) point compositions.
 - Lightweight containers can be constexpr constructed, CPose* cannot.

*/
