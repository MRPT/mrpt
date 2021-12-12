/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/containers/bimap.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/bimap_serialization.h>

using namespace mrpt::serialization;

TEST(Serialization, bimap)
{
	mrpt::containers::bimap<uint32_t, std::string> bm1, bm2;
	bm1.insert(1, "one");
	bm1.insert(2, "two");
	bm1.insert(0, "zero");

	mrpt::io::CMemoryStream f;
	auto arch = mrpt::serialization::archiveFrom(f);
	arch << bm1;

	f.Seek(0);
	arch >> bm2;
	EXPECT_EQ(bm1, bm2);

	// Test the != operator too:
	bm2.insert(3, "three");
	EXPECT_NE(bm1, bm2);
}
