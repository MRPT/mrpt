/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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

*/
