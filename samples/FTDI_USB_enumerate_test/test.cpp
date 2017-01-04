/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CInterfaceFTDI.h>
#include <mrpt/system/os.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/threads.h>
#include <cstdio>
#include <iostream>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
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

