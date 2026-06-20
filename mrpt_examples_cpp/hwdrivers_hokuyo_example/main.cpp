/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/comms/CSerialPort.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::gui;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

string SERIAL_NAME;  // Name of the serial port to open

// ------------------------------------------------------
//				Test_HOKUYO
// ------------------------------------------------------
void Test_HOKUYO()
{
  CHokuyoURG laser;

  string serName, type;

  string ip;

  unsigned int port;

  std::cout << "Specify the type of the Hokuyo connection, usb or ethernet: ";
  getline(cin, type);

  while ((mrpt::system::lowerCase(type) != "usb") && (mrpt::system::lowerCase(type) != "ethernet"))
  {
    std::cout << "Incorrect type"
              << "\n";
    std::cout << "Specify the type of the Hokuyo connection, usb or ethernet: ";
    getline(cin, type);
  }

  std::cout << endl << endl << "HOKUYO laser range finder test application." << endl << "\n";

  if (mrpt::system::lowerCase(type) == "usb")
  {
    if (SERIAL_NAME.empty())
    {
      std::cout << "Enter the serial port name (e.g. COM1, ttyS0, ttyUSB0, "
                   "ttyACM0): ";
      getline(cin, serName);
    }
    else
    {
      std::cout << "Using serial port: " << SERIAL_NAME << "\n";
      serName = SERIAL_NAME;
    }

    // Set the laser serial port:
    laser.setSerialPort(serName);
  }
  else
  {
    std::cout << "Enter the ip direction: ";
    getline(cin, ip);

    std::cout << "Enter the port number: ";
    cin >> port;

    // Set the laser serial port:
    laser.setIPandPort(ip, port);
  }
  string intensity;
  std::cout << endl << endl << "Enable intensity [y/n]:";
  getline(cin, intensity);
  laser.setIntensityMode(mrpt::system::lowerCase(intensity) == "y");

  // Config: Use defaults + selected port ( serial or ethernet )

  printf("[TEST] Turning laser ON...\n");
  if (laser.turnOn())
    printf("[TEST] Initialization OK!\n");
  else
  {
    printf("[TEST] Initialization failed!\n");
    return;
  }

#if MRPT_HAS_WXWIDGETS
  CDisplayWindowPlots win("Laser scans");
#endif

  std::cout << "Press any key to stop capturing..."
            << "\n";

  CTicTac tictac;
  tictac.Tic();

  while (!mrpt::system::os::kbhit())
  {
    bool thereIsObservation, hardError;
    CObservation2DRangeScan obs;

    laser.doProcessSimple(thereIsObservation, obs, hardError);

    if (hardError) printf("[TEST] Hardware error=true!!\n");

    if (thereIsObservation)
    {
      // double FPS = 1.0 / tictac.Tac();

      obs.getDescriptionAsText(std::cout);

      obs.sensorPose = CPose3D(0, 0, 0);

      mrpt::maps::CSimplePointsMap theMap;
      theMap.insertionOptions.minDistBetweenLaserPoints = 0;
      theMap.insertObservation(obs);
      // map.save2D_to_text_file("_out_scan.txt");

      /*
      Scene			scene3D;
      viz::CPointCloud::Ptr	points =
      viz::CPointCloud::Create();
      points->loadFromPointsMap(&map);
      scene3D.insert(points);
      CFileStream("_out_point_cloud.3Dscene",fomWrite) << scene3D;
      */

#if MRPT_HAS_WXWIDGETS
      std::vector<float> xs, ys, zs;
      theMap.getAllPoints(xs, ys, zs);
      win.plot(xs, ys, ".b3");
      win.axis_equal();
#endif

      tictac.Tic();
    }

    std::this_thread::sleep_for(15ms);
  };

  laser.turnOff();
}

int main(int argc, char** argv)
{
  try
  {
    if (argc > 1)
    {
      SERIAL_NAME = string(argv[1]);
    }

    Test_HOKUYO();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << "\n";
    return -1;
  }
  catch (...)
  {
    printf("Another exception!!");
    return -1;
  }
}
