/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \defgroup mrpt_typemeta_grp [mrpt-typemeta]

Metaprogramming header-only library to obtain `constexpr` textual string representations of enum types and type names, including complex STL compound types.

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

# Library `mrpt-typemeta`
<small> [New in MRPT 2.0.0] </small>

This library is part of MRPT but has no dependencies, so it can be installed
in Debian-based systems with:

        sudo apt install libmrpt-typemeta-dev

## compile-time type/struct/class names to strings
Use mrpt::typemeta::TTypeName to extract a `constexpr` string with a compiler-independent
representation of arbitrarily-complex types and STL containers.
Note that creating objects from a run-time string representation of its type
is handled in a different library (\ref mrpt_serialization_grp).

\snippet typemeta_TTypeName/test.cpp example typename
Output:
\include typemeta_TTypeName/console.out

## compile-time constexpr strings manipulation
\snippet typemeta_StaticString/test.cpp example sstring
Output:
\include typemeta_StaticString/console-sstring.out

## compile-time numbers to strings
\snippet typemeta_StaticString/test.cpp example num2str
Output:
\include typemeta_StaticString/console-num2str.out

## enum values to/from strings
\snippet typemeta_TEnumType/test.cpp example
Output:
\include typemeta_TEnumType/console.out

*/
