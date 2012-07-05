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

#include <mrpt/base.h>
#include <mrpt/hwdrivers/CInterfaceFTDI.h>

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

