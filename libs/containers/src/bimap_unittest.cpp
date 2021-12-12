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

TEST(bimap, operations)
{
	mrpt::containers::bimap<std::string, double> bm;

	EXPECT_TRUE(bm.empty());

	bm.insert("numero", 2.3);
	EXPECT_FALSE(bm.empty());
	bm.clear();
	EXPECT_TRUE(bm.empty());

	bm.insert("uno", 1.0);
	bm.insert("dos", 2.0);
	bm.insert("tres", 3.0);

	EXPECT_ANY_THROW(bm.insert("three", 3.0));	// dupl value

	EXPECT_ANY_THROW(bm.insert("dos", 22.0));  // dupl key
	bm.insert("dos", 2.0);	// does not throw since it's the same pair (k,v)

	EXPECT_EQ(1.0, bm.direct("uno"));
	EXPECT_EQ(2.0, bm.direct("dos"));
	EXPECT_EQ(3.0, bm.direct("tres"));
	EXPECT_ANY_THROW(bm.direct("xxx"));

	EXPECT_EQ("uno", bm.inverse(1.0));
	EXPECT_EQ("dos", bm.inverse(2.0));
	EXPECT_EQ("tres", bm.inverse(3.0));
	EXPECT_ANY_THROW(bm.inverse(0.0));

	bm.erase_by_key("uno");
	EXPECT_ANY_THROW(bm.direct("uno"));
	EXPECT_ANY_THROW(bm.inverse(1.0));

	bm.erase_by_value(2.0);
	EXPECT_ANY_THROW(bm.direct("dos"));
	EXPECT_ANY_THROW(bm.inverse(2.0));

	EXPECT_EQ(3.0, bm.direct("tres"));
	EXPECT_EQ("tres", bm.inverse(3.0));
}
