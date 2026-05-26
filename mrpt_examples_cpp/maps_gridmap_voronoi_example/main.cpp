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

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::random;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace std;

// MRPT_EXAMPLES_BASE_DIRECTORY provided via compile definition
const string sample_simplemap_file =
    MRPT_DATA_DIR + string("/datasets/localization_demo.simplemap.gz");

// ------------------------------------------------------
//				TestVoronoi
// ------------------------------------------------------
void TestVoronoi(const std::string& image_file)
{
  COccupancyGridMap2D gridmap;

  if (!image_file.empty())
  {
    // Load from image file provided by the user:
    std::cout << "Loading gridmap from image: " << image_file << "\n";
    if (!mrpt::system::fileExists(image_file))
    {
      cerr << "Error: file doesn't exist: " << image_file << "\n";
      return;
    }
    const float resolution = 0.05f;  // meters/pixel — adjust as needed
    if (!gridmap.loadFromBitmapFile(image_file, resolution))
    {
      cerr << "Error loading gridmap from image file.\n";
      return;
    }
  }
  else
  {
    // Load from default simplemap:
    if (!mrpt::system::fileExists(sample_simplemap_file))
    {
      cerr << "Error: file doesn't exist: " << sample_simplemap_file << "\n";
      return;
    }
    std::cout << "Loading simplemap: " << sample_simplemap_file << "\n";

    CSimpleMap simplemap;
    if (!simplemap.loadFromFile(sample_simplemap_file))
      THROW_EXCEPTION_FMT("Failed to load simplemap: %s", sample_simplemap_file.c_str());

    std::cout << "Building gridmap...\n";
    gridmap = COccupancyGridMap2D(-5, 5, -5, 5, 0.10f);
    gridmap.loadFromSimpleMap(simplemap);
  }

  // Build Voronoi diagram:
  std::cout << "Building Voronoi diagram...\n";
  gridmap.buildVoronoiDiagram(0.5f, 0.3f);

  // --- Build 3D scene ---
  // 1) Gridmap as a flat 3D object:
  auto gl_grid = mrpt::viz::CSetOfObjects::Create();
  gridmap.getVisualizationInto(*gl_grid);

  // 2) Voronoi nodes as a thick point cloud at z = 0.01 (slightly above map):
  auto gl_voronoi = mrpt::viz::CPointCloud::Create();
  gl_voronoi->setColor(0.0f, 0.8f, 1.0f);  // cyan
  gl_voronoi->setPointSize(5.0f);

  const auto& vd = gridmap.getVoronoiDiagram();
  const int cx0 = 0, cy0 = 0;
  const int cxN = static_cast<int>(vd.getSizeX());
  const int cyN = static_cast<int>(vd.getSizeY());

  for (int cx = cx0; cx < cxN; cx++)
  {
    for (int cy = cy0; cy < cyN; cy++)
    {
      const uint16_t* cell = vd.cellByIndex(cx, cy);
      if (cell && *cell > 0)
      {
        const float x = gridmap.idx2x(cx);
        const float y = gridmap.idx2y(cy);
        gl_voronoi->insertPoint(x, y, 0.01f);
      }
    }
  }

  // Show in a single 3D window:
  mrpt::gui::CDisplayWindow3D win("Gridmap + Voronoi diagram", 1024, 768);

  {
    mrpt::viz::Scene::Ptr& scene = win.get3DSceneAndLock();
    scene->insert(gl_grid);
    scene->insert(gl_voronoi);
    win.unlockAccess3DScene();
  }
  win.repaint();

  std::cout << "Close the window or press any key to exit.\n";
  win.waitForKey();
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    const std::string image_file = (argc >= 2) ? std::string(argv[1]) : std::string{};
    TestVoronoi(image_file);
    return 0;
  }
  catch (exception& e)
  {
    std::cout << "MRPT exception caught: " << e.what() << "\n";
    return -1;
  }
  catch (...)
  {
    printf("Another exception!!");
    return -1;
  }
}
