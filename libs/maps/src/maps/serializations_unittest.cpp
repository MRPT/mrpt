/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#define MRPT_NO_WARN_BIG_HDR  // Yes, we really want to include all classes.
#include <mrpt/maps.h>

#include <mrpt/utils/CMemoryStream.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace std;


// Create a set of classes, then serialize and deserialize to test possible bugs:
TEST(SerializeTestMaps, WriteReadToMem)
{
	const mrpt::utils::TRuntimeClassId* lstClasses[] = {
		CLASS_ID( CBeacon ),
		CLASS_ID( CBeaconMap ),
		CLASS_ID( CColouredPointsMap),
		CLASS_ID( CGasConcentrationGridMap2D),
		CLASS_ID( CWirelessPowerGridMap2D),
		CLASS_ID( CHeightGridMap2D),
		CLASS_ID( CReflectivityGridMap2D),
		CLASS_ID( COccupancyGridMap2D),
		CLASS_ID( CSimplePointsMap),
		CLASS_ID( CRandomFieldGridMap3D ),
		CLASS_ID( CWeightedPointsMap),
		CLASS_ID( COctoMap)
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

