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

#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/hwdrivers/CInterfaceFTDI.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;


// ------------------------------------------------------
//				Test_EnumerateDevices
// ------------------------------------------------------
void Test_EnumerateDevices()
{
	CInterfaceFTDI	usbDevice;

	unsigned long		nConectedDevices;

	TFTDIDeviceList lstDevs;

	while (!mrpt::system::os::kbhit())
	{
		// Create list of devices:
		usbDevice.ListAllDevices( lstDevs );

		nConectedDevices = (unsigned long)lstDevs.size();

		cout << "There are " << nConectedDevices << " USB devices - " << mrpt::system::dateTimeToString( mrpt::system::getCurrentTime() ) << endl;

		for (size_t i=0;i<nConectedDevices;i++)
			cout << lstDevs[i] << endl;

		printf("\nPRESS ANY KEY TO END THE PROGRAM...\n\n");
		cout.flush();
		mrpt::system::sleep(500);
	};
}

int main()
{
	try
	{
		Test_EnumerateDevices();
		return 0;
	} catch (std::exception &e)
	{
		std::cout << "EXCEPCION: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}

}

