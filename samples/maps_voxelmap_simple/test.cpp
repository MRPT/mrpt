/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/os.h>

#include <chrono>
#include <iostream>
#include <thread>

// ------------------------------------------------------
//				TestVoxelMap
// ------------------------------------------------------
void TestVoxelMap()
{
  mrpt::maps::CVoxelMap map(0.1);

  // map.insertionOptions.max_range = 5.0;  // [m]

  // Manually update voxels:
  if (false)
  {
    map.updateVoxel(1, 1, 1, true);  // integrate 'occupied' measurement

    map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement
    map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement
    map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement

    map.updateVoxel(-1, -1, 1, false);  // integrate 'free' measurement

    double occup;
    bool is_mapped;
    mrpt::math::TPoint3D pt;

    pt = mrpt::math::TPoint3D(1, 1, 1);
    is_mapped = map.getPointOccupancy(pt.x, pt.y, pt.z, occup);
    std::cout << "pt: " << pt << " is mapped?: " << (is_mapped ? "YES" : "NO")
              << " occupancy: " << occup << std::endl;

    pt = mrpt::math::TPoint3D(-1, -1, 1);
    is_mapped = map.getPointOccupancy(pt.x, pt.y, pt.z, occup);
    std::cout << "pt: " << pt << " is mapped?: " << (is_mapped ? "YES" : "NO")
              << " occupancy: " << occup << std::endl;
  }

  // Insert 2D scan:
  if (true)
  {
    mrpt::obs::CObservation2DRangeScan scan1;
    mrpt::obs::stock_observations::example2DRangeScan(scan1);
    map.insertObservation(scan1);
  }

  mrpt::gui::CDisplayWindow3D win("VoxelMap demo", 640, 480);

  auto gl_map = mrpt::opengl::COctoMapVoxels::Create();

  {
    mrpt::opengl::Scene::Ptr& scene = win.get3DSceneAndLock();

    {
      auto gl_grid = mrpt::opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
      gl_grid->setColor_u8(mrpt::img::TColor(0x80, 0x80, 0x80));
      scene->insert(gl_grid);
    }
    scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple());

    map.getAsOctoMapVoxels(*gl_map);

    // View occupied points:
    {
      auto mapPts = map.getOccupiedVoxels();
      mapPts->renderOptions.point_size = 5.0;
      scene->insert(mapPts->getVisualization());
    }

    gl_map->showGridLines(false);
    gl_map->showVoxels(mrpt::opengl::VOXEL_SET_OCCUPIED, true);
    gl_map->showVoxels(mrpt::opengl::VOXEL_SET_FREESPACE, true);
    scene->insert(gl_map);

    win.unlockAccess3DScene();
  }

  std::cout << "Close the window to exit" << std::endl;

  bool update_msg = true;

  while (win.isOpen())
  {
    if (win.keyHit())
    {
      const unsigned int k = win.getPushedKey();

      switch (k)
      {
        case 'g':
        case 'G':
          gl_map->showGridLines(!gl_map->areGridLinesVisible());
          break;
        case 'f':
        case 'F':
          gl_map->showVoxels(
              mrpt::opengl::VOXEL_SET_FREESPACE,
              !gl_map->areVoxelsVisible(mrpt::opengl::VOXEL_SET_FREESPACE));
          break;
        case 'o':
        case 'O':
          gl_map->showVoxels(
              mrpt::opengl::VOXEL_SET_OCCUPIED,
              !gl_map->areVoxelsVisible(mrpt::opengl::VOXEL_SET_OCCUPIED));
          break;
        case 'l':
        case 'L':
          gl_map->enableLights(!gl_map->areLightsEnabled());
          break;
      };
      update_msg = true;
    }

    if (update_msg)
    {
      update_msg = false;

      win.addTextMessage(
          5, 5,
          mrpt::format(
              "Commands: 'f' (freespace=%s) | 'o' (occupied=%s) | 'l' "
              "(lights=%s)",
              gl_map->areVoxelsVisible(mrpt::opengl::VOXEL_SET_FREESPACE) ? "YES" : "NO",
              gl_map->areVoxelsVisible(mrpt::opengl::VOXEL_SET_OCCUPIED) ? "YES" : "NO",
              gl_map->areLightsEnabled() ? "YES" : "NO"));

      win.repaint();
    }

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(10ms);
  };
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    TestVoxelMap();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cout << "Exception: " << e.what() << std::endl;
    return -1;
  }
}
