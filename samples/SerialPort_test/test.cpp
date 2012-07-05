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

#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/utils.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace std;

int main()
{
	try
	{
		CSerialPort		*serPort;

		string 			serName;

		cout << "Serial port test application: Use it with a loopback serial port (pins 2-3 connected)"<< endl;

		cout << "Enter the serial port name (e.g. COM1, ttyS0, ttyUSB0): ";
		getline(cin,serName);

		cout << endl;
		cout << "Opening serial port...";
		serPort = new CSerialPort(serName);
		cout << "OK" << endl;

		cout << "Setting timeouts...";
		serPort->setTimeouts(100,1,100, 1, 100);
		cout << "OK" << endl;

		cout << "Setting baud rate...";
		serPort->setConfig(500000);
		cout << "OK" << endl;


		for (int i=0;i<10;i++)
		{
			// Test write:
			cout << "Writing test data...";
			const char buf1[] = "Hello world!";
			size_t written = serPort->Write(buf1,sizeof(buf1));
			cout << written << " bytes written." << endl;

			// Read:
			cout << "Reading data...";
			char buf2[100];
			size_t nRead = serPort->Read(buf2,sizeof(buf2));
			cout << nRead << " bytes read: '";

			buf2[nRead]=0;
			cout << buf2 << "'" << endl;

		}

		delete serPort;

	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return -1;
	}

	return 0;
}

