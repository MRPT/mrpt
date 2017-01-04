/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#define MRPT_NO_WARN_BIG_HDR  // Yes, we really want to include all classes.
#include <mrpt/obs.h>

#include <mrpt/utils/CMemoryStream.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

// Defined in tests/test_main.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}

const mrpt::utils::TRuntimeClassId* lstClasses[] = {
	// Observations:
	CLASS_ID(CObservation2DRangeScan),
	CLASS_ID(CObservation3DRangeScan),
	CLASS_ID(CObservationRGBD360),
	CLASS_ID(CObservationBearingRange),
	CLASS_ID(CObservationBatteryState),
	CLASS_ID(CObservationWirelessPower),
	CLASS_ID(CObservationRFID),
	CLASS_ID(CObservationBeaconRanges),
	CLASS_ID(CObservationComment),
	CLASS_ID(CObservationGasSensors),
	CLASS_ID(CObservationGPS),
	CLASS_ID(CObservationReflectivity),
	CLASS_ID(CObservationIMU),
	CLASS_ID(CObservationOdometry),
	CLASS_ID(CObservationRange),
#if MRPT_HAS_OPENCV   // These classes need CImage serialization
	CLASS_ID(CObservationImage),
	CLASS_ID(CObservationStereoImages),
#endif
	CLASS_ID(CObservationCANBusJ1939),
	CLASS_ID(CObservationRawDAQ),
	CLASS_ID(CObservation6DFeatures),
	CLASS_ID(CObservationVelodyneScan),
	// Actions:
	CLASS_ID(CActionRobotMovement2D),
	CLASS_ID(CActionRobotMovement3D)
	};


// Create a set of classes, then serialize and deserialize to test possible bugs:
TEST(SerializeTestObs, WriteReadToMem)
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

// Also try to convert them to octect vectors:
TEST(SerializeTestObs, WriteReadToOctectVectors)
{
	for (size_t i=0;i<sizeof(lstClasses)/sizeof(lstClasses[0]);i++)
	{
		try
		{
			mrpt::vector_byte buf;
			{
				CSerializable* o = static_cast<CSerializable*>(lstClasses[i]->createObject());
				mrpt::utils::ObjectToOctetVector(o,buf);
				delete o;
			}

			CSerializablePtr recons;
			mrpt::utils::OctetVectorToObject(buf,recons);
		}
		catch(std::exception &e)
		{
			GTEST_FAIL() <<
				"Exception during serialization test for class '"<< lstClasses[i]->className <<"':\n" << e.what() << endl;
		}
	}
}

