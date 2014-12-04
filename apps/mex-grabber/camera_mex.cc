/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


/*-----------------------------------------------------------------------------
	APPLICATION: mex-grabber
	FILE: mexgrabber_main.cpp
	AUTHORS: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>
				Jesus Briales Garcia <jesusbriales@gmail.com>

	For instructions and details, see:
	 http://
  -----------------------------------------------------------------------------*/

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/round.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CObservationGPS.h>
#include <mrpt/slam/CObservationIMU.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>

// Matlab MEX interface headers
#include <mexplus.h>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;
using namespace mexplus;

template class mexplus::Session<CGenericSensor>;
//template class mexplus::Session<CImage>;

namespace {
// Defines MEX API for new.
MEX_DEFINE(new) (int nlhs, mxArray* plhs[],
					  int nrhs, const mxArray* prhs[]) {
	 InputArguments input(nrhs, prhs, 1);
	 OutputArguments output(nlhs, plhs, 1);

	 // Create mexplus handler
	 //CGenericSensorPtr sensor = CGenericSensor::createSensorPtr("CHokuyoURG");
	 //intptr_t id_ = Session<CGenericSensor>::create( sensor.pointer() );
	 //intptr_t id_ = Session<CHokuyoURG>::create( new CHokuyoURG() );
	 //intptr_t id_ = Session<CImage>::create( new CImage() );
	 //output.set(0, id_);

	 mexPrintf("Before sensor\n");
	 CGenericSensorPtr sensor = CGenericSensor::createSensorPtr("CHokuyoURG");
	 mexPrintf("After sensor\n");
	 //output.set(0, Session<CImage>::create(new CImage()));

	 intptr_t id_ = Session<CGenericSensor>::create( sensor.pointer() );
	 //intptr_t id_ = Session<CHokuyoURG>::create( new CHokuyoURG() );
	 //intptr_t id_ = Session<CImage>::create( new CImage() );
	 output.set(0, id_);

	 mexPrintf("Done new, assigning to output\n");
}

// Defines MEX API for delete.
MEX_DEFINE(delete) (int nlhs, mxArray* plhs[],
						  int nrhs, const mxArray* prhs[]) {
	 InputArguments input(nrhs, prhs, 1);
	 OutputArguments output(nlhs, plhs, 0);
	 Session<CGenericSensor>::destroy(input.get(0));
	 //Session<CHokuyoURG>::destroy(input.get(0));
	 //Session<CImage>::destroy(input.get(0));
}

}

MEX_DISPATCH // Don't forget to add this if MEX_DEFINE() is used.
