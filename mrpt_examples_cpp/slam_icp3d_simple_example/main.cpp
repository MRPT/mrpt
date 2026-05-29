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

/**
 * icp3d
 * Execute an Iterative Closest Point algorithm using two 3D point clouds.
 */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/random.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/stock_objects.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace std;
using namespace mrpt::literals;  // _deg
using namespace mrpt::gui;
using namespace mrpt::viz;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;

// Increase these values to get more precision. It will also increase run time.
const size_t HOW_MANY_YAWS = 120;
const size_t HOW_MANY_PITCHS = 120;

CPose3D SCAN2_POSE_ERROR(0.15, -0.07, 0.10, -0.03, 0.1, 0.1);

// Synthesize a 3D point cloud by sampling two spheres and two disks analytically.
void generatePointClouds(CSimplePointsMap& M1, CSimplePointsMap& M2)
{
  auto& rng = mrpt::random::getRandomGenerator();
  const size_t N = HOW_MANY_YAWS * HOW_MANY_PITCHS;

  // Sphere at origin, radius 0.5
  for (size_t i = 0; i < N / 3; i++)
  {
    const double theta = rng.drawUniform(0, 2 * M_PI);
    const double phi = std::acos(rng.drawUniform(-1.0, 1.0));
    const double r = 0.5;
    M1.insertPoint(
        r * std::sin(phi) * std::cos(theta), r * std::sin(phi) * std::sin(theta),
        r * std::cos(phi));
  }

  // Disk 1: radius 2, pose (0,0,0, 0, 5deg, 5deg)
  const CPose3D disk1_pose(0, 0, 0, 0, 5.0_deg, 5.0_deg);
  for (size_t i = 0; i < N / 3; i++)
  {
    const double ang = rng.drawUniform(0, 2 * M_PI);
    const double r = rng.drawUniform(0.0, 2.0);
    mrpt::math::TPoint3D local(r * std::cos(ang), r * std::sin(ang), 0);
    mrpt::math::TPoint3D world;
    disk1_pose.composePoint(local, world);
    M1.insertPoint(world.x, world.y, world.z);
  }

  // Disk 2: radius 2, pose (0, 0, 0, 30deg, -20deg, -2deg)
  const CPose3D disk2_pose(0, 0, 0, 30.0_deg, -20.0_deg, -2.0_deg);
  for (size_t i = 0; i < N / 3; i++)
  {
    const double ang = rng.drawUniform(0, 2 * M_PI);
    const double r = rng.drawUniform(0.0, 2.0);
    mrpt::math::TPoint3D local(r * std::cos(ang), r * std::sin(ang), 0);
    mrpt::math::TPoint3D world;
    disk2_pose.composePoint(local, world);
    M1.insertPoint(world.x, world.y, world.z);
  }

  // M2 is M1 with a known pose error applied
  M2 = M1;
  M2.changeCoordinatesReference(SCAN2_POSE_ERROR);
}

void test_icp3D()
{
  Scene::Ptr scene1 = Scene::Create();
  Scene::Ptr scene2 = Scene::Create();
  Scene::Ptr scene3 = Scene::Create();

  auto plane1 = CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
  plane1->setColor(0.3f, 0.3f, 0.3f);
  scene1->insert(plane1);
  scene2->insert(plane1);
  scene3->insert(plane1);

  // Build reference scene visualization
  {
    CSphere::Ptr sph = CSphere::Create(0.5);
    sph->setLocation(0, 0, 0);
    sph->setColor(1, 0, 0);
    scene1->insert(sph);

    CDisk::Ptr pln = CDisk::Create();
    pln->setDiskRadius(2);
    pln->setPose(CPose3D(0, 0, 0, 0, 5.0_deg, 5.0_deg));
    pln->setColor(0.8f, 0, 0);
    scene1->insert(pln);

    CDisk::Ptr pln2 = CDisk::Create();
    pln2->setDiskRadius(2);
    pln2->setPose(CPose3D(0, 0, 0, 30.0_deg, -20.0_deg, -2.0_deg));
    pln2->setColor(0.9f, 0, 0);
    scene1->insert(pln2);
  }

  // Generate the two 3D point clouds
  CSimplePointsMap M1, M2_noisy;
  std::cout << "Generating synthetic point clouds...\n";
  generatePointClouds(M1, M2_noisy);
  std::cout << "Done. M1: " << M1.size() << " pts, M2: " << M2_noisy.size() << " pts\n";

  M1.renderOptions.color = mrpt::img::TColorf(1, 0, 0);
  M2_noisy.renderOptions.color = mrpt::img::TColorf(0, 0, 1);

  scene2->insert(M1.getVisualization());
  scene2->insert(M2_noisy.getVisualization());

  // ICP-3D
  CICP icp;
  CICP::TReturnInfo icp_info;

  icp.options.thresholdDist = 0.40;
  icp.options.thresholdAng = 0;

  CPose3DPDF::Ptr pdf = icp.Align3D(
      &M2_noisy,  // Map to align
      &M1,        // Reference map
      CPose3D(),  // Initial gross estimate
      icp_info);

  CPose3D mean = pdf->getMeanVal();

  std::cout << "ICP run took " << icp_info.executionTime << " secs.\n";
  std::cout << "Goodness: " << 100 * icp_info.goodness
            << "% , # of iterations= " << icp_info.nIterations << " Quality: " << icp_info.quality
            << "\n";
  std::cout << "ICP output: mean= " << mean << "\n";
  std::cout << "Real displacement: " << SCAN2_POSE_ERROR << "\n";

  // Aligned maps
  M2_noisy.changeCoordinatesReference(CPose3D() - mean);

  scene3->insert(M1.getVisualization());
  scene3->insert(M2_noisy.getVisualization());

  CDisplayWindow3D window("ICP-3D demo: scene", 500, 500);
  CDisplayWindow3D window2("ICP-3D demo: UNALIGNED scans", 500, 500);
  CDisplayWindow3D window3("ICP-3D demo: ICP-ALIGNED scans", 500, 500);

  window.setPos(10, 10);
  window2.setPos(530, 10);
  window3.setPos(10, 520);

  window.get3DSceneAndLock() = scene1;
  window.unlockAccess3DScene();

  window2.get3DSceneAndLock() = scene2;
  window2.unlockAccess3DScene();

  window3.get3DSceneAndLock() = scene3;
  window3.unlockAccess3DScene();

  std::this_thread::sleep_for(20ms);
  window.forceRepaint();
  window2.forceRepaint();

  window.setCameraElevationDeg(15);
  window.setCameraAzimuthDeg(90);
  window.setCameraZoom(15);

  window2.setCameraElevationDeg(15);
  window2.setCameraAzimuthDeg(90);
  window2.setCameraZoom(15);

  window3.setCameraElevationDeg(15);
  window3.setCameraAzimuthDeg(90);
  window3.setCameraZoom(15);

  std::cout << "Press any key to exit...\n";
  window.waitForKey();
}

int main()
{
  mrpt::random::getRandomGenerator().randomize();
  try
  {
    test_icp3D();
    return 0;
  }
  catch (exception& e)
  {
    std::cout << "Error: " << e.what() << '.' << "\n";
    return -1;
  }
}
