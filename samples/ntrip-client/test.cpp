/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

