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

#include <mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils.h>
#include <mrpt/system.h>
#include <mrpt/slam.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::slam;
#include <string>
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

