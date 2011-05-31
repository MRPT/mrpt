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

#include <mrpt/hwdrivers/CNTRIPClient.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::utils;


// ------------------------------------------------------
//				TestNTRIP
// ------------------------------------------------------
void TestNTRIP()
{
	const string server = "www.euref-ip.net";
	const int    server_port = 2101;

	CNTRIPClient::TListMountPoints	lst;
	string errMsg;

	bool ret = CNTRIPClient::retrieveListOfMountpoints(lst,errMsg,server, server_port);

	if (!ret)
	{
		cout << "Error: " << errMsg << endl;
		return;
	}

	if (lst.empty())
	{
		cout << "Zero streams listed in caster...?" << endl;
		return;
	}

	// List:
	// -----------------------------------
	for (CNTRIPClient::TListMountPoints::const_iterator it=lst.begin();it!=lst.end();it++)
	{
		const CNTRIPClient::TMountPoint &m = *it;
		cout <<
			"MOUNT: " << m.mountpoint_name <<
			"  | COUNTRY: "<< m.country_code <<
			"  | NMEA?: " << m.needs_nmea <<
			"  | FORMAT: " << m.format << " (" << m.id << ") " <<
			m.extra_info << 
			endl;
	}

	// Now connect to a random server:
	// -----------------------------------
	CNTRIPClient	ntrip;
	CNTRIPClient::NTRIPArgs params;

	CNTRIPClient::TListMountPoints::iterator it=lst.begin();
	//std::advance(it,8);

	cout << "Connecting to: " << it->mountpoint_name << " - " << it->id << endl;
	params.mountpoint = it->mountpoint_name;
	params.server = server;
	params.port   = server_port;

	params.user   = "";
	params.password = "";

	string msgerr;

	if (!ntrip.open(params,msgerr))
	{
		cout << "ERROR: " << msgerr << endl;
	}
	else
	{
		cout << "Reading stream... press any key to finish." << endl;

		vector_byte dat;
		while (!mrpt::system::os::kbhit() )
		{
			ntrip.stream_data.readAndClear(dat);
			cout << "Read " << dat.size() << " bytes." << endl;
			mrpt::system::sleep(1000);
		}
	}
}


void TestNTRIP2()
{
	//const string server = "www.euref-ip.net";
	const string server = "193.144.251.13";
	const int    server_port = 2101;

	// Now connect to a random server:
	// -----------------------------------
	CNTRIPClient	ntrip;
	CNTRIPClient::NTRIPArgs params;

	params.mountpoint = "ACOR0";
	params.server = server;
	params.port   = server_port;
	params.user   = "";
	params.password = "";

	string msgerr;

	if (!ntrip.open(params,msgerr))
	{
		cout << "ERROR: " << msgerr << endl;
	}
	else
	{
		cout << "Reading stream... press any key to finish." << endl;

		vector_byte dat;
		while (!mrpt::system::os::kbhit() )
		{
			ntrip.stream_data.readAndClear(dat);
			cout << "Read " << dat.size() << " bytes." << endl;
			mrpt::system::sleep(1000);
		}
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestNTRIP();
		//TestNTRIP2();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}

