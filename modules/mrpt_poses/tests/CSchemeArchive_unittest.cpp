/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

#include <sstream>

// Check if we have jsoncpp to enable those tests:
#include <mrpt/serialization/config.h>

using namespace mrpt::serialization;

#if MRPT_HAS_JSONCPP
TEST(SchemaSerialization, JSON_archive)
{
  auto arch = archiveJSON();
  mrpt::poses::CPose2D pt1{1.0, 2.0, 3.0}, pt2;
  arch = pt1;
  std::stringstream ss;
  ss << arch;
  auto pos = ss.str().find("\"datatype\" : \"mrpt::poses::CPose2D\"");
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

#endif
