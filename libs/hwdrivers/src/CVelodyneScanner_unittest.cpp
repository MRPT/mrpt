/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CVelodyneScanner.h>
#include <mrpt/system/filesystem.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace std;

// Defined in tests/test_main.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}

#include <mrpt/config.h>
#if MRPT_HAS_LIBPCAP

TEST(CVelodyneScanner, sample_vlp16_dataset)
{
	const string fil = MRPT_GLOBAL_UNITTEST_SRC_DIR + string("/tests/sample_velodyne_vlp16_gps.pcap");

	if (!mrpt::system::fileExists(fil))
	{
		std::cerr << "WARNING: Skipping test due to missing file: " << fil << "\n";
		return;
	}

	CVelodyneScanner velodyne;

	velodyne.setModelName( mrpt::hwdrivers::CVelodyneScanner::VLP16);
	velodyne.setPCAPInputFile(fil);
	velodyne.setPCAPInputFileReadOnce(true);
	velodyne.enableVerbose(false);
	velodyne.setPCAPVerbosity(false);

	velodyne.initialize();

	size_t nScans = 0, nGPS=0;
	bool rx_ok = true;
	for (size_t i=0;i<1000 && rx_ok;i++)
	{
		mrpt::obs::CObservationVelodyneScanPtr scan;
		mrpt::obs::CObservationGPSPtr          gps;
		rx_ok = velodyne.getNextObservation(scan,gps);
		if (scan) { nScans++;
		scan->generatePointCloud();
		}
		if (gps)  nGPS++;
	};
	EXPECT_EQ(nScans,4U);
	EXPECT_GT(nGPS,0U);
}

TEST(CVelodyneScanner, sample_hdl32_dataset)
{
	const string fil = MRPT_GLOBAL_UNITTEST_SRC_DIR + string("/tests/sample_velodyne_hdl32.pcap");

	if (!mrpt::system::fileExists(fil))
	{
		std::cerr << "WARNING: Skipping test due to missing file: " << fil << "\n";
		return;
	}

	CVelodyneScanner velodyne;

	velodyne.setModelName( mrpt::hwdrivers::CVelodyneScanner::HDL32);
	velodyne.setPCAPInputFile(fil);
	velodyne.setPCAPInputFileReadOnce(true);
	velodyne.enableVerbose(false);
	velodyne.setPCAPVerbosity(false);

	velodyne.initialize();

	size_t nScans = 0;
//	size_t nGPS=0;
	bool rx_ok = true;
	for (size_t i=0;i<1000 && rx_ok;i++)
	{
		mrpt::obs::CObservationVelodyneScanPtr scan;
		mrpt::obs::CObservationGPSPtr          gps;
		rx_ok = velodyne.getNextObservation(scan,gps);
		if (scan) nScans++;
//		if (gps)  nGPS++;
	};
	EXPECT_EQ(nScans,3U);
}

#endif // MRPT_HAS_LIBPCAP

