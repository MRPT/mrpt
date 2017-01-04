/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/utils/types_math.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;



TEST(CVectorDouble,resize)
{
	{
		mrpt::math::CVectorDouble v;
		EXPECT_TRUE(v.size()==0);
	}

	for (int i=0;i<10;i++)
	{
		mrpt::math::CVectorDouble v(i);
		EXPECT_TRUE(v.size()==i);
	}

	for (int i=0;i<10;i++)
	{
		mrpt::math::CVectorDouble v;
		v.resize(i);
		EXPECT_TRUE(v.size()==i);
	}

	for (int i=10;i>=0;i--)
	{
		mrpt::math::CVectorDouble v;
		v.resize(i);
		EXPECT_TRUE(v.size()==i);
	}

	{
		mrpt::math::CVectorDouble v;
		for (int i=0;i<10;i++)
		{
			v.push_back(double(i));
			EXPECT_TRUE(v.size()==(i+1));
		}
		for (int i=0;i<10;i++)
		{
			EXPECT_TRUE(v[i]==i);
		}
	}
}


