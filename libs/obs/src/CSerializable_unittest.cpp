/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
			    //cout << lstClasses[i]->className << endl;
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

