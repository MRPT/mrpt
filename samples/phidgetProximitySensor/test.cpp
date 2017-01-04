/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/system.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace std;

/** Usage : ./test <conf file name>.ini
 */
int main(int argc, char **argv)
{
	try
	{

	string confFileName;
	if(argc < 2) confFileName = string("./conf.ini");
	else confFileName = string(argv[1]);


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
		for(size_t i = 0 ; i < obs.sensedData.size() ; i ++)
		{
			cout << obs.sensedData[i].sensedDistance << "\t";
		}
		cout << endl;
		mrpt::system::sleep(10);
	}while(	!mrpt::system::os::kbhit() );

	return 0;

	}
	catch(std::exception &e)
	{
		std::cerr << e.what();
		return 1;
	}
}

