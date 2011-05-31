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

