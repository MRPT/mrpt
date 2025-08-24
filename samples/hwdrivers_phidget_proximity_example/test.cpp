/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/config/CConfigFile.h>
#include <mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/system/CTicTac.h>

#include <chrono>
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
