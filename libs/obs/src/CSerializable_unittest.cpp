/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/obs.h>
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


// Create a set of classes, then serialize and deserialize to test possible bugs:
TEST(SerializeTestObs, WriteReadToMem)
{
	const mrpt::utils::TRuntimeClassId* lstClasses[] = {
		// Observations:
		CLASS_ID(CObservation2DRangeScan),
		CLASS_ID(CObservation3DRangeScan),
		CLASS_ID(CObservationBearingRange),
		CLASS_ID(CObservationBatteryState),
		CLASS_ID(CObservationWirelessPower),
		CLASS_ID(CObservationRFID),
		CLASS_ID(CObservationBeaconRanges),
		CLASS_ID(CObservationComment),
		CLASS_ID(CObservationGasSensors),
		CLASS_ID(CObservationGPS),
		CLASS_ID(CObservationImage),
		CLASS_ID(CObservationReflectivity),
		CLASS_ID(CObservationIMU),
		CLASS_ID(CObservationOdometry),
		CLASS_ID(CObservationRange),
		CLASS_ID(CObservationStereoImages),
		CLASS_ID(CObservationCANBusJ1939),
		CLASS_ID(CObservationRawDAQ),
		// Actions:
		CLASS_ID(CActionRobotMovement2D),
		CLASS_ID(CActionRobotMovement3D)
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

