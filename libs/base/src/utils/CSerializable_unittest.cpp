/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */


#include <mrpt/base.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

// Defined in run_unittests.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}


// Load data from constant file and check for exact match.
TEST(SerializeTestBase, LoadDemoFile)
{
	struct TRegs
	{
		uint8_t		v1;
		int8_t		v2;
		uint16_t	v3;
		int16_t		v4;
		uint32_t	v5;
		int32_t		v6;
		uint64_t	v7;
		int64_t		v8;
		std::string 	v9;
	};

	// Reference data:
	TRegs	R;
	R.v1 = 8;
	R.v2 = -3;
	R.v3 = 781;
	R.v4 = -888;
	R.v5 =  100000;
	R.v6 = -100000;
	R.v7 =  555666777;
	R.v8 = -555666777;
	R.v9 = "an example test";

	// Loaded data to compare with reference:
	TRegs	L;
	const string fil = MRPT_GLOBAL_UNITTEST_SRC_DIR + string("/tests/serialize_test_data.bin");

	if (!mrpt::system::fileExists(fil))
	{
		cerr << "WARNING: Skipping test due to missing file: " << fil << "\n";
	}
	else
	{
		CFileInputStream	gg( fil );
		gg >> L.v1 >> L.v2 >> L.v3 >> L.v4 >> L.v5 >> L.v6 >> L.v7 >> L.v8 >> L.v9;

		EXPECT_EQ(R.v1,L.v1 );
		EXPECT_EQ(R.v2,L.v2 );
		EXPECT_EQ(R.v3,L.v3 );
		EXPECT_EQ(R.v4,L.v4 );
		EXPECT_EQ(R.v5,L.v5 );
		EXPECT_EQ(R.v6,L.v6 );
		EXPECT_EQ(R.v7,L.v7 );
		EXPECT_EQ(R.v8,L.v8 );
		EXPECT_EQ(R.v9,L.v9 );
	}
}

// Create a set of classes, then serialize and deserialize to test possible bugs:
TEST(SerializeTestBase, WriteReadToMem)
{
	const mrpt::utils::TRuntimeClassId* lstClasses[] = {
		// Misc:
		CLASS_ID(CPose2D),
		CLASS_ID(CPose3D),
		CLASS_ID(CPose3DQuat),
		CLASS_ID(CPoint2D),
		CLASS_ID(CPoint3D),
		// Poses:
		CLASS_ID(CPose3DPDFGaussian),
		CLASS_ID(CPose3DQuatPDFGaussian)
		};

	for (size_t i=0;i<sizeof(lstClasses)/sizeof(lstClasses[0]);i++)
	{
		try
		{
			CMemoryStream  buf;
			{
				CSerializable* o = static_cast<CSerializable*>(lstClasses[i]->createObject());
				buf << *o;
				delete o;
			}

			CSerializablePtr recons;
			buf.Seek(0);
			buf >> recons;
		}
		catch(std::exception &e)
		{
			GTEST_FAIL() <<
				"Exception during serialization test for class '"<< lstClasses[i]->className <<"':\n" << e.what() << endl;
		}
	}
}

// Create a set of classes, then serialize and deserialize to test possible bugs:
TEST(SerializeTestBase, CArray)
{
	try
	{
		CMemoryStream  buf;
		CArrayDouble<5>  a, b;
		for (CArrayDouble<5>::Index i=0;i<a.size();i++) a[i] = i+10;

		buf << a;
		buf.Seek(0);
		buf >> b;

		EXPECT_TRUE(a==b);
	}
	catch(std::exception &e)
	{
		GTEST_FAIL() <<
			"Exception:\n" << e.what() << endl;
	}

}

