/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \defgroup mrpt_serialization_grp [mrpt-serialization]

Serialization (marshalling) portable library for C++ objects persistence.

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

# Library `mrpt-serialization`
<small> [New in MRPT 2.0.0] </small>

This library is part of MRPT and can be installed in Debian-based systems with:

        sudo apt install libmrpt-serialization-dev

Main classes and concepts associated with this library:
- mrpt::serialization::CArchive provides:
	- An abstraction of I/O streams (e.g. std::stream's, MRPT's mrpt::io::CStream's, sockets)
	- A portable API (endianness aware) for serialization of data structures on those streams.
- mrpt::serialization::CSerializable provides:
	- A generic way to define persistent C++ classes.
	- Versioning: if class fields are added or removed, you'll still be able to read old data files.

Serialization happens via `archive << object` operators in all cases but, underneath, two mechanisms are provided:
- **Direct overloading of the `<<`/`>>` operators** for mrpt::serialization::CArchive objects.
	- Pros: concise declaration (just the two operator functions).
	- Cons: Cannot handle versioning. Cannot deserialize unknown types (i.e. no support for class factory).
- **Via mrpt::serialization::CSerializable** and associated macros:
	- Pros: Allows polymorphic classes to be (de)serialized. Allows versioning.
	- Cons: Requires adding macros to class definitions. Requires **registering** the class with \ref mrpt_rtti_grp.

Support for STL containers is provided via this "direct mechanism" for the container structure itself, but contained
elements can use any of the serialization mechanisms.

## Example #1: serialize STL container via MRPT `CStream`s

See: \ref serialization_stl/test.cpp
\snippet serialization_stl/test.cpp example
Output:
\include serialization_stl/console.out

## Example #2: serialize STL container via `std::ostream` and `std::istream`

See: \ref serialization_stl/test.cpp
\snippet serialization_stl/test.cpp example_stdio
Output:
\include serialization_stl/console.out

## Example #3: Serialization of existing MRPT classes

Write me!

## Example #4: Polymorphic serialization of user classes

Write me!

*/
