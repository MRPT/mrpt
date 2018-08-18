/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/serialization/CSchemeArchive.h>
#include <mrpt/poses/CPose2D.h>
#include <sstream>
#include <gtest/gtest.h>

// Check if we have jsoncpp to enable those tests:
#include <mrpt/config.h>
#if MRPT_HAS_JSONCPP
#include <json/json.h>
#endif

using namespace mrpt::serialization;

#if MRPT_HAS_JSONCPP
TEST(SchemaSerialization, JSON_archive)
{
	auto arch = archiveJSON();
	mrpt::poses::CPose2D pt1{1.0, 2.0, 3.0}, pt2;
	arch = pt1;
	std::stringstream ss;
	ss << arch;
	auto pos = ss.str().find("\"datatype\" : \"CPose2D\"");
	EXPECT_TRUE(pos != std::string::npos);

	// test deserializing:
	ss.seekg(0);  // rewind for reading
	auto arch2 = archiveJSON();
	// Load the plain text representation into the archive:
	ss >> arch2;
	// Parse the JSON data into an MRPT object:
	arch2.readTo(pt2);

	EXPECT_NEAR(pt1.x(), pt2.x(), 1e-6);
	EXPECT_NEAR(pt1.y(), pt2.y(), 1e-6);
	EXPECT_NEAR(pt1.phi(), pt2.phi(), 1e-6);
}

TEST(SchemaSerialization, JSON_raw)
{
	Json::Value val;
	auto arch = mrpt::serialization::CSchemeArchiveBase(
		std::make_unique<CSchemeArchive<Json::Value>>(val));
	mrpt::poses::CPose2D pt1{1.0, 2.0, 3.0}, pt2;
	arch = pt1;
	std::stringstream ss;
	ss << val;
	auto pos = ss.str().find("\"datatype\" : \"CPose2D\"");
	EXPECT_TRUE(pos != std::string::npos);

	// test deserializing:
	Json::Value val2;
	ss.seekg(0);  // rewind for reading
	ss >> val2;
	auto arch2 = mrpt::serialization::CSchemeArchiveBase(
		std::make_unique<CSchemeArchive<Json::Value>>(val2));
	arch2.readTo(pt2);
	EXPECT_NEAR(pt1.x(), pt2.x(), 1e-6);
	EXPECT_NEAR(pt1.y(), pt2.y(), 1e-6);
	EXPECT_NEAR(pt1.phi(), pt2.phi(), 1e-6);
}

#endif
