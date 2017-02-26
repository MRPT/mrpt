/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/utils/stl_serialization.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/random.h>
#include <mrpt/math/ops_vectors.h>  // to serialize vectors
#include <mrpt/system/filesystem.h>
#include <mrpt/poses.h> // to test their serialization
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;

// Defined in tests/test_main.cpp
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

const mrpt::utils::TRuntimeClassId* lstClasses[] = {
	// Misc:
	CLASS_ID(CPose2D),
	CLASS_ID(CPose3D),
	CLASS_ID(CPose3DQuat),
	CLASS_ID(CPoint2D),
	CLASS_ID(CPoint3D),
	// Poses:
	CLASS_ID(CPose3DPDFGaussian),
	CLASS_ID(CPose3DQuatPDFGaussian),
	// Others:
	CLASS_ID(TStereoCamera)
	};

// Create a set of classes, then serialize and deserialize to test possible bugs:
TEST(SerializeTestBase, WriteReadToMem)
{
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

// Create a set of classes, then test that copy operators "work" (doesn't crash)
TEST(SerializeTestBase, CopyOperator)
{
	for (size_t i=0;i<sizeof(lstClasses)/sizeof(lstClasses[0]);i++)
	{
		try
		{
			CSerializable* o1 = static_cast<CSerializable*>(lstClasses[i]->createObject());
			CSerializable* o2 = static_cast<CSerializable*>(lstClasses[i]->createObject());
			*o2 = *o1;
			delete o1;
			delete o2;
		}
		catch(std::exception &e)
		{
			GTEST_FAIL() <<
				"Exception during copy operator test for class '"<< lstClasses[i]->className <<"':\n" << e.what() << endl;
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

// Serialize and deserialize complex STL types
TEST(SerializeTestBase, STL_serialization)
{
	try
	{
		// std::vector<>
		{
			CMemoryStream  buf;

			std::vector<double> a,b;
			a.resize(30);
			for (size_t i=0;i<a.size();i++) a[i]=50-i;

			buf << a; buf.Seek(0); buf >> b;
			EXPECT_TRUE(a==b);
		}

		// std::list<...>
		{
			CMemoryStream  buf;

			std::list<std::map<double,std::set<std::string> > >  a,b;

			// Fill with random:
			mrpt::random::CRandomGenerator rng;
			const size_t N = rng.drawUniform(10,30);
			for (size_t i=0;i<N;i++)
			{
				std::map<double,std::set<std::string> > d;
				const size_t M = rng.drawUniform(4,9);
				for (size_t j=0;j<M;j++)
				{
					std::set<std::string> & dd = d[ rng.drawGaussian1D_normalized() ];
					const size_t L = rng.drawUniform(2,15);
					for (size_t k=0;k<L;k++)
						dd.insert(mrpt::format("%f", rng.drawGaussian1D_normalized() ));
				}
				a.push_back(d);
			}


			buf << a; buf.Seek(0); buf >> b;
			EXPECT_TRUE(a==b);
		}
	}
	catch(std::exception &e)
	{
		GTEST_FAIL() <<
			"Exception:\n" << e.what() << endl;
	}

}

// Test casting of smart pointers:
TEST(SerializeTestBase, CastSmartPointers)
{
	using namespace mrpt::poses;

	// Create:
	CPose2DPtr p1 = CPose2D::Create();
	// Upcast:
	mrpt::utils::CSerializablePtr p2 = p1;
	// Downcast:
	mrpt::utils::CSerializablePtr p3 = p2;
	// Copy:
	mrpt::utils::CSerializablePtr p4 = p1;

	EXPECT_TRUE(IS_CLASS(p1,CPose2D));
	EXPECT_TRUE(IS_CLASS(p2,CPose2D));
	EXPECT_TRUE(IS_CLASS(p3,CPose2D));
	EXPECT_TRUE(IS_CLASS(p4,CPose2D));
}
