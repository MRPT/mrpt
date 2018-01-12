/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/serialization/CArchive.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace std;

// Defined in tests/test_main.cpp
namespace mrpt
{
extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
}

const mrpt::rtti::TRuntimeClassId* lstClasses[] = {
	CLASS_ID(CLogFileRecord),
};

// Create a set of classes, then serialize and deserialize to test possible
// bugs:
TEST(NavTests, Serialization_WriteReadToMem)
{
	for (size_t i = 0; i < sizeof(lstClasses) / sizeof(lstClasses[0]); i++)
	{
		try
		{
			CMemoryStream buf;
			auto arch = archiveFrom(buf);
			{
				CSerializable* o =
					static_cast<CSerializable*>(lstClasses[i]->createObject());
				arch << *o;
				delete o;
			}

			CSerializable::Ptr recons;
			buf.Seek(0);
			arch >> recons;
		}
		catch (std::exception& e)
		{
			GTEST_FAIL() << "Exception during serialization test for class '"
						 << lstClasses[i]->className << "':\n"
						 << e.what() << endl;
		}
	}
}

// Also try to convert them to octect vectors:
TEST(SerializeTestObs, WriteReadToOctectVectors)
{
	for (size_t i = 0; i < sizeof(lstClasses) / sizeof(lstClasses[0]); i++)
	{
		try
		{
			std::vector<uint8_t> buf;
			{
				CSerializable* o =
					static_cast<CSerializable*>(lstClasses[i]->createObject());
				mrpt::serialization::ObjectToOctetVector(o, buf);
				delete o;
			}

			CSerializable::Ptr recons;
			mrpt::serialization::OctetVectorToObject(buf, recons);
		}
		catch (std::exception& e)
		{
			GTEST_FAIL() << "Exception during serialization test for class '"
						 << lstClasses[i]->className << "':\n"
						 << e.what() << endl;
		}
	}
}

// Load test datalog
TEST(NavTests, NavLogLoadFromTestFile)
{
	const string navlog_file =
		MRPT_GLOBAL_UNITTEST_SRC_DIR +
		string("/tests/serialize_test_data.reactivenavlog");
	if (!mrpt::system::fileExists(navlog_file))
	{
		cerr << "WARNING: Skipping test due to missing file: " << navlog_file
			 << "\n";
		return;
	}

	CFileGZInputStream f(navlog_file);
	auto arch = archiveFrom(f);

	try
	{
		for (int i = 0; i < 2; i++)
		{
			mrpt::nav::CLogFileRecord lfr;
			arch.ReadObject(&lfr);
		}
	}
	catch (std::exception& e)
	{
		FAIL() << "Failed to parse stored navlog. Exception was:\n"
			   << e.what() << endl;
	}
}
