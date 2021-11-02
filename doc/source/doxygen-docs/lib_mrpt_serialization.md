\defgroup mrpt_serialization_grp [mrpt-serialization]

Serialization (marshalling) portable library for C++ objects persistence.

[TOC]

# Library mrpt-serialization


This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-serialization-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

## Binary serialization (most efficient)

Main classes and concepts associated with this library:
- mrpt::serialization::CArchive provides:
	- An abstraction of I/O streams (e.g. std::stream's, MRPT's
mrpt::io::CStream's, sockets)
	- A portable API (endianness aware) for serialization of data structures on
those streams.
- mrpt::serialization::CSerializable provides:
	- A generic way to define persistent C++ classes.
	- Versioning: if class fields are added or removed, you'll still be able to
read old data files.

Serialization happens via `archive << object` operators in all cases but,
underneath, two mechanisms are provided:
- **Direct overloading of the `<<`/`>>` operators** for
mrpt::serialization::CArchive objects.
	- Pros: concise declaration (just the two operator functions).
	- Cons: Cannot handle versioning. Cannot deserialize unknown types (i.e. no
support for class factory).
- **Via mrpt::serialization::CSerializable** and associated macros:
	- Pros: Allows polymorphic classes to be (de)serialized. Allows versioning.
	- Cons: Requires adding macros to class definitions. Requires
**registering** the class with \ref mrpt_rtti_grp.

Support for STL containers is provided via this "direct mechanism" for the
container structure itself, but contained elements can use any of the
serialization mechanisms.

Serializing `shared_ptr<T>` is supported for any arbitrary type `T`. It is legal
to serialize an empty (`nullptr`) smart pointer; an empty pointer will be read
back. Polymorphic classes can be also writen and read, although reading a smart
pointer to a polymorphic base class is only supported for classes derived from
MRPT's CSerializable, since that operation requires registering types in a class
factory (see \a mrpt_rtti_grp and mrpt::serialization::CSerializable).

### Example #1: serialize STL container via MRPT `CStream`s

See: \ref serialization_stl/test.cpp
\snippet serialization_stl/test.cpp example
Output:
\include serialization_stl/console.out

### Example #2: serialize STL container via `std::ostream` and `std::istream`

See: \ref serialization_stl/test.cpp
\snippet serialization_stl/test.cpp example_stdio
Output:
\include serialization_stl/console.out

### Example #3: Serialization of existing MRPT classes

Write me!

### Example #4: Polymorphic serialization of user classes

Write me!


## Schema (plain-text) serialization (human readable)

An alternative mechanism to serialize objects is based on
mrpt::serialization::CSchemeArchive and allow objects to be (de)serialized in
plain text formats like XML, JSON, YAML, etc.
For now (Aug 2018) only JSON is implemented.

### Example #1: Easy JSON serialization

This method only requires having MRPT built against jsoncpp, but does not
enforce the user to also depend on that library.

See: \ref serialization_json_example/test.cpp
\snippet serialization_json_example/test.cpp example
Output:
\include serialization_json_example/console.out

### Example #2: JSON serialization with full access to jsoncpp

If you want to have full control on the JSON formatting and other details,
you may directly depend on jsoncpp and use the following, template-based access
to MRPT serialization:

See: \ref serialization_json_example/test.cpp
\snippet serialization_json_example/test.cpp example_raw

# Library contents
