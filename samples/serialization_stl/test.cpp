/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
/** \example serialization_stl/test.cpp */

//! [example]
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>

#include <iostream> // cout
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator

void WriteAndReadExample()
{
	// Declare data to be serialized:
	std::vector<uint32_t>  m1 { 1,2,3 };

	// === Write ===
	{
		// CStream output:
		mrpt::io::CFileOutputStream  ofs("file.bin");
		auto arch_out = mrpt::serialization::CArchiveStream(ofs);
		// Use << to serialize in binary form:
		arch_out << m1;
	}

	// === Read ===
	std::vector<uint32_t>  m2;
	{
		// CStream output:
		mrpt::io::CFileInputStream  ifs("file.bin");
		auto arch_in = mrpt::serialization::CArchiveStream(ifs);
		// Use >> to deserialize:
		arch_in >> m2;
	}

	std::cout << "Wrote: ";
	std::copy(m1.begin(), m1.end(), std::ostream_iterator<uint32_t>(std::cout, " "));
	std::cout << std::endl << "Read: ";
	std::copy(m2.begin(), m2.end(), std::ostream_iterator<uint32_t>(std::cout, " "));
	std::cout << std::endl;
}
//! [example]

//! [example_stdio]
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/serialization/CArchive.h>
#include <fstream> // io std streams

#include <iostream> // cout
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator

void WriteAndReadExampleStdIO()
{
	// Declare data to be serialized:
	std::vector<uint32_t>  m1{ 1,2,3 };

	// === Write ===
	{
		// CStream output:
		std::ofstream ofs("file.bin");
		auto arch_out = mrpt::serialization::CArchiveStream<std::ostream>(ofs);
		// Use << to serialize in binary form:
		arch_out << m1;
	}

	// === Read ===
	std::vector<uint32_t>  m2;
	{
		// CStream output:
		std::ifstream ifs("file.bin");
		auto arch_in = mrpt::serialization::CArchiveStream<std::istream>(ifs);
		// Use >> to deserialize:
		arch_in >> m2;
	}

	std::cout << "Wrote: ";
	std::copy(m1.begin(), m1.end(), std::ostream_iterator<uint32_t>(std::cout, " "));
	std::cout << std::endl << "Read: ";
	std::copy(m2.begin(), m2.end(), std::ostream_iterator<uint32_t>(std::cout, " "));
	std::cout << std::endl;
}
//! [example_stdio]

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		WriteAndReadExample();
		WriteAndReadExampleStdIO();
		return 0;
	}
	catch (std::exception& e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!");
		return -1;
	}
}
