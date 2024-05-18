/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/hwdrivers/CTaoboticsIMU.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>

#include <chrono>
#include <iostream>
#include <thread>

std::string serialPort;  // Name of the serial port to open

// ------------------------------------------------------
//				Test_IMU
// ------------------------------------------------------
void TestIMU()
{
  mrpt::hwdrivers::CTaoboticsIMU imu;

  if (!serialPort.empty())
  {
    std::cout << "Using serial port: " << serialPort << std::endl;
    imu.setSerialPort(serialPort);
  }
  // otherwise, use default port.

  // Load config from INI file:
  // imu.loadConfig( CConfigFile( "./config.ini") ,"IMU");

  std::cout << "Trying to initialize the sensor..." << std::endl;
  imu.initialize();  // This will raise an exception on error

  mrpt::gui::CDisplayWindow3D win("IMU", 1024, 800);

  auto glIMU = mrpt::opengl::stock_objects::CornerXYZ();
  {
    auto& scene = win.get3DSceneAndLock();
    scene->insert(mrpt::opengl::CGridPlaneXY::Create());
    scene->insert(glIMU);

    win.unlockAccess3DScene();
  }

  const double t0 = mrpt::Clock::nowDouble();
  size_t totalObsCount = 0;

  while (1)
  {
    imu.doProcess();

    const auto lstObs = imu.getObservations();
    if (lstObs.empty()) continue;

    auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(lstObs.begin()->second);
    ASSERT_(o);

    std::cout << o->asString() << "\n";

    const auto imuPose = mrpt::poses::CPose3D::FromQuaternion(mrpt::math::CQuaternionDouble(
        o->get(mrpt::obs::IMU_ORI_QUAT_W), o->get(mrpt::obs::IMU_ORI_QUAT_X),
        o->get(mrpt::obs::IMU_ORI_QUAT_Y), o->get(mrpt::obs::IMU_ORI_QUAT_Z)));

    // Sensor rate:
    const double t1 = mrpt::Clock::nowDouble();
    totalObsCount += lstObs.size();

    double sensorRate = totalObsCount / (t1 - t0);

    // Update 3D view:
    {
      // auto& scene =
      win.get3DSceneAndLock();

      glIMU->setPose(imuPose);

      win.addTextMessage(5, 5, mrpt::format("Sensor Rate: %.02f Hz", sensorRate));

      win.unlockAccess3DScene();
    }
    win.forceRepaint();
  };
}

int main(int argc, char** argv)
{
  try
  {
    if (argc > 1)
    {
      serialPort = std::string(argv[1]);
    }

    TestIMU();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << mrpt::exception_to_str(e) << std::endl;
    return -1;
  }
}
