/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservationRange.h>
#include <mrpt/hwdrivers/CBoardIR.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace std;

int main()
{
	try
	{
		CBoardIR			IRBoard;

		IRBoard.setSerialPortName("ttyUSB1");
		cout << "Measuring distances. Press any key to exit..." << endl;


		// Load configuration:
		while ( !mrpt::system::os::kbhit() )
		{
			mrpt::obs::CObservationRange obs;
			bool thereIsObservation,hardwareError;

			IRBoard.getObservation(thereIsObservation,obs,hardwareError);

			if (thereIsObservation)
			{
				for (mrpt::obs::CObservationRange::const_iterator i=obs.begin();i!=obs.end();++i)
					printf("ID %u -> %.02f cm\n", (unsigned int)i->sensorID, i->sensedDistance * 100.0f);
				printf("\n");
			}

			//mrpt::system::sleep(150);

		} // end-while

	} // end-try
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return -1;
	}

	return 0;
}

