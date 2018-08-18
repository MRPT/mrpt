/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

/** \defgroup mrpt_rtti_grp [mrpt-rtti]

Runtime Type Information (RTTI) library, providing compiler-independent class
registry, class factory, and inheritance information.

[TOC]

# Library `mrpt-rtti`
<small> [New in MRPT 2.0.0] </small>

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-rtti-dev

Any class with RTTI support has to be derived from mrpt::rtti::CObject, either
directly or via a hierarchy of inheriting classes.
Class factory by name enables deserialization of polymorphic classes in the
library \ref mrpt_serialization_grp.

All classes defined in each MRPT module are automatically registered when
loading the module (if dynamically linked).

## Example #1: defining new user classes

See: \ref rtti_example1/test.cpp
\snippet rtti_example1/test.cpp example-define-class
\snippet rtti_example1/test.cpp example-define-class-test
Output:
\include rtti_example1/console-ex1.out

## Example #2: using the class factory

See: \ref rtti_example1/test.cpp
\snippet rtti_example1/test.cpp example-define-class
\snippet rtti_example1/test.cpp example-factory
Output:
\include rtti_example1/console-ex2.out

*/
