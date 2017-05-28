/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/system/filesystem.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace std;

// Defined in tests/test_main.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}

const mrpt::utils::TRuntimeClassId* lstClasses[] = {
	CLASS_ID(CLogFileRecord),
};

// Create a set of classes, then serialize and deserialize to test possible bugs:
TEST(NavTests, Serialization_WriteReadToMem)
{
	for (size_t i = 0; i<sizeof(lstClasses) / sizeof(lstClasses[0]); i++)
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
		catch (std::exception &e)
		{
			GTEST_FAIL() <<
				"Exception during serialization test for class '" << lstClasses[i]->className << "':\n" << e.what() << endl;
		}
	}
}

// Also try to convert them to octect vectors:
TEST(SerializeTestObs, WriteReadToOctectVectors)
{
	for (size_t i = 0; i<sizeof(lstClasses) / sizeof(lstClasses[0]); i++)
	{
		try
		{
			mrpt::vector_byte buf;
			{
				CSerializable* o = static_cast<CSerializable*>(lstClasses[i]->createObject());
				mrpt::utils::ObjectToOctetVector(o, buf);
				delete o;
			}

			CSerializablePtr recons;
			mrpt::utils::OctetVectorToObject(buf, recons);
		}
		catch (std::exception &e)
		{
			GTEST_FAIL() <<
				"Exception during serialization test for class '" << lstClasses[i]->className << "':\n" << e.what() << endl;
		}
	}
}




// Load test datalog
TEST(NavTests, NavLogLoadFromTestFile)
{
	const string navlog_file = MRPT_GLOBAL_UNITTEST_SRC_DIR + string("/tests/serialize_test_data.reactivenavlog");
	if (!mrpt::system::fileExists(navlog_file))
	{
		cerr << "WARNING: Skipping test due to missing file: " << navlog_file << "\n";
		return;
	}

	CFileGZInputStream f(navlog_file);

	try 
	{
		for (int i=0;i<2;i++)
		{
			mrpt::nav::CLogFileRecord lfr;
			f.ReadObject(&lfr);
		}
	}
	catch (std::exception &e)
	{
		FAIL() << "Failed to parse stored navlog. Exception was:\n" << e.what() << endl;
	}
}

