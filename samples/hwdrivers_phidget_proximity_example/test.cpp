/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/obs/CObservationRange.h>
#include <iostream>
#include <thread>

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace mrpt::config;
using namespace std;

/** Usage : ./test <conf file name>.ini
 */
int main(int argc, char** argv)
{
	try
	{
		string confFileName;
		if (argc < 2)
			confFileName = string("./conf.ini");
		else
			confFileName = string(argv[1]);

		CPhidgetInterfaceKitProximitySensors ik;
		string section("IK_1");
		CConfigFile conf(confFileName);
		ik.loadConfig(conf, section);
		ik.initialize();
		CObservationRange obs;
		do
		{
			ik.doProcess();
			ik.getObservation(obs);
			for (size_t i = 0; i < obs.sensedData.size(); i++)
			{
				cout << obs.sensedData[i].sensedDistance << "\t";
			}
			cout << endl;
			std::this_thread::sleep_for(10ms);
		} while (!mrpt::system::os::kbhit());

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what();
		return 1;
	}
}
