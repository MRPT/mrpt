/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CNTRIPClient.h>
#include <mrpt/system/os.h>
#include <cstdio>

using namespace std;
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

