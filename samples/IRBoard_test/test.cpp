/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/slam.h>
#include <mrpt/hwdrivers/CBoardIR.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
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
			mrpt::slam::CObservationRange obs;
			bool thereIsObservation,hardwareError;
			
			IRBoard.getObservation(thereIsObservation,obs,hardwareError);

			if (thereIsObservation)
			{
				for (mrpt::slam::CObservationRange::const_iterator i=obs.begin();i!=obs.end();++i)
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

