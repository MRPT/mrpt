/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
/** \example serialization_json_example/test.cpp */

#include <iostream>  // cout
#include <sstream>  // stringstream

//! [example]
#include <mrpt/serialization/CSchemeArchive.h>
#include <mrpt/poses/CPosePDFGaussian.h>

#include <iostream>  // cout

void WriteAndReadExample()
{
	// Define the MRPT objects to be serialized:
	mrpt::poses::CPosePDFGaussian pdf1{
		mrpt::poses::CPose2D{1.0, 2.0, mrpt::DEG2RAD(90.0)},
		mrpt::math::CMatrixDouble33()};
	mrpt::poses::CPose2D p1{5.0, 6.0, mrpt::DEG2RAD(.0)};

	// --------------------
	// JSON Serialization
	// --------------------
	// Create a JSON archive:
	auto arch = mrpt::serialization::archiveJSON();

	// Writes the objects to the JSON archive:
	arch["pose_pdf"] = pdf1;
	arch["pose"] = p1;

	// Writes the JSON representation to an std::ostream
	std::stringstream ss;
	ss << arch;

	// also, print to cout for illustration purposes:
	std::cout << arch << std::endl;

	// --------------------
	// JSON Deserialization
	// --------------------
	// rewind stream for reading from the start
	ss.seekg(0);

	// Create a new JSON archive for reading
	auto arch2 = mrpt::serialization::archiveJSON();

	// Load the plain text representation into the archive:
	ss >> arch2;

	// Parse the JSON data into an MRPT object:
	mrpt::poses::CPosePDFGaussian pdf2;
	arch2["pose_pdf"].readTo(pdf2);
	mrpt::poses::CPose2D p2;
	arch2["pose"].readTo(p2);

	std::cout << "read pose:" << p2.asString() << std::endl;
}
//! [example]

int main(int argc, char** argv)
{
	try
	{
		WriteAndReadExample();
		return 0;
	}
	catch (const std::exception& e)
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

#if 0  // code disabled, only present as an example for the docs:

//! [example_raw]
#include <json/json.h>

void test()
{
	Json::Value val;
	auto arch = mrpt::serialization::CSchemeArchiveBase(
		std::make_unique<CSchemeArchive<Json::Value>>(val));

	mrpt::poses::CPose2D pt1{1.0, 2.0, 3.0};
	// Store any CSerializable object into the JSON value:
	arch = pt1;
	// Alternative:
	// arch["pose"] = pt1;

	std::stringstream ss;
	ss << val;
	std::cout << val << std::endl;
}

//! [example_raw]

#endif
