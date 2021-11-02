\defgroup mrpt_typemeta_grp [mrpt-typemeta]

Metaprogramming header-only library to obtain `constexpr` textual string
representations of enum types and type names, including smart pointers and
complex STL compound types.


[TOC]

# Library mrpt-typemeta


This library is part of MRPT but has no dependencies, so it can be installed
in Debian-based systems with:

		sudo apt install libmrpt-typemeta-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).


## Example #1: compile-time type/struct/class names to strings
Use mrpt::typemeta::TTypeName to extract a `constexpr` string with a
compiler-independent representation of arbitrarily-complex types and STL
containers. Note that creating objects from a run-time string representation of
its type is handled in a different library (\ref mrpt_serialization_grp).

See: \ref typemeta_TTypeName/test.cpp
\snippet typemeta_TTypeName/test.cpp example typename
Output:
\include typemeta_TTypeName/console.out

## Example #2: compile-time constexpr strings manipulation
See: \ref typemeta_StaticString/test.cpp
\snippet typemeta_StaticString/test.cpp example sstring
Output:
\include typemeta_StaticString/console-sstring.out

## Example #3: compile-time numbers to strings
See: \ref typemeta_StaticString/test.cpp
\snippet typemeta_StaticString/test.cpp example num2str
Output:
\include typemeta_StaticString/console-num2str.out

## Example #4: enum values to/from strings
See: \ref typemeta_TEnumType/test.cpp
\snippet typemeta_TEnumType/test.cpp example
Output:
\include typemeta_TEnumType/console.out

# Library contents
