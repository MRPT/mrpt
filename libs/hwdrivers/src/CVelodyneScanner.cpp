/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CVelodyneScanner.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CVelodyneScanner,mrpt::hwdrivers)

CVelodyneScanner::CVelodyneScanner( )
{
	m_sensorLabel = "Velodyne_";
}

CVelodyneScanner::~CVelodyneScanner( )
{
}

void  CVelodyneScanner::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
	MRPT_START

//	m_usbSerialNumber = configSource.read_string(iniSection, "USB_serialname","",false);

	MRPT_END
}

bool CVelodyneScanner::getObservation( mrpt::obs::CObservationVelodyneScan &obs )
{
	try
	{

		return false;
	}
	catch(exception &e)
	{
		cerr << "[CEnoseModular::getObservation] Returning false due to exception: " << endl;
		cerr << e.what() << endl;
		return false;
	}
	catch(...)
	{
		return false;
	}
}


void CVelodyneScanner::doProcess()
{
	CObservationVelodyneScanPtr obs= CObservationVelodyneScan::Create();

	if (getObservation(*obs))
	{
		m_state = ssWorking;
		appendObservation( obs );
	}
	else
	{
		m_state = ssError;
		cerr << "No observation received from the Velodyne!" << endl;
	}
}

/** Tries to initialize the sensor, after setting all the parameters with a call to loadConfig.
*  \exception This method must throw an exception with a descriptive message if some critical error is found. */
void CVelodyneScanner::initialize()
{

}

