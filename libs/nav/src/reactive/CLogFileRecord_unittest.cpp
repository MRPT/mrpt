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
using namespace std;

// Defined in tests/test_main.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}

// Load test datalog
TEST(NavTests, LoadNavLogFile)
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

