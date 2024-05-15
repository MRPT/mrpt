/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CVoxelMapRGB.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <chrono>
#include <iostream>
#include <thread>

// ------------------------------------------------------
//				TestVoxelMapFromTUM
// ------------------------------------------------------
void TestVoxelMapFromTUM(
    const std::string& datasetRawlogFile,
    const std::string& groundTruthFile,
    double VOXELMAP_RESOLUTION,
    double VOXELMAP_MAX_RANGE)
{
  // To find out external image files:
  mrpt::io::setLazyLoadPathBase(mrpt::obs::CRawlog::detectImagesDirectory(datasetRawlogFile));

  std::cout << "Loading dataset: " << datasetRawlogFile << "..." << std::endl;

  mrpt::obs::CRawlog dataset;
  dataset.loadFromRawLogFile(datasetRawlogFile);

  std::cout << "Done! " << dataset.size() << " entries." << std::endl;

  std::cout << "Loading GT path from: " << groundTruthFile << "..." << std::endl;

  mrpt::math::CMatrixDouble gtData;
  gtData.loadFromTextFile(groundTruthFile);

  std::cout << "Done! " << gtData.rows() << " rows." << std::endl;

  // # timestamp tx ty tz qx qy qz qw
  mrpt::poses::CPose3DInterpolator gt;
  for (int i = 0; i < gtData.rows(); i++)
  {
    gt.insert(
        mrpt::Clock::fromDouble(gtData(i, 0)),
        mrpt::poses::CPose3D::FromQuaternionAndTranslation(
            mrpt::math::CQuaternionDouble(gtData(i, 7), gtData(i, 4), gtData(i, 5), gtData(i, 6)),
            mrpt::math::TPoint3D(gtData(i, 1), gtData(i, 2), gtData(i, 3))));
  }

  // ----------------------
  // Voxel map
  // ----------------------
  mrpt::maps::CVoxelMapRGB map(VOXELMAP_RESOLUTION);

  map.insertionOptions.max_range = VOXELMAP_MAX_RANGE;  // [m]
  map.insertionOptions.ray_trace_free_space = false;    // only occupied

  // gui and demo app:
  mrpt::gui::CDisplayWindow3D win("VoxelMap demo", 640, 480);

  auto glVoxels = mrpt::opengl::COctoMapVoxels::Create();

  // *IMPORTANT*: Required to see RGB color in the opengl visualization:
  glVoxels->setVisualizationMode(mrpt::opengl::COctoMapVoxels::COLOR_FROM_RGB_DATA);

  // create GL visual objects:
  auto glCamGroup = mrpt::opengl::CSetOfObjects::Create();
  glCamGroup->insert(mrpt::opengl::stock_objects::CornerXYZSimple(0.3));
  auto glObsPts = mrpt::opengl::CPointCloudColoured::Create();
  glCamGroup->insert(glObsPts);
  bool glCamFrustrumDone = false;

  mrpt::opengl::Viewport::Ptr glViewRGB;

  {
    mrpt::opengl::Scene::Ptr& scene = win.get3DSceneAndLock();

    // Set a large near plane so we can "see thru walls" easily when
    // approaching a point:
    scene->getViewport()->setViewportClipDistances(2.0, 200.0);

    {
      auto gl_grid = mrpt::opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
      gl_grid->setColor_u8(mrpt::img::TColor(0x80, 0x80, 0x80));
      scene->insert(gl_grid);
    }
    scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple());

    scene->insert(glCamGroup);

    // View occupied points:
    {
      auto mapPts = map.getOccupiedVoxels();
      mapPts->renderOptions.point_size = 5.0;
      scene->insert(mapPts->getVisualization());
    }

    scene->insert(glVoxels);

    glViewRGB = scene->createViewport("rgb_view");
    glViewRGB->setViewportPosition(0, 0.7, 0.3, 0.25);
    glViewRGB->setTransparent(true);

    win.unlockAccess3DScene();
  }

  std::cout << "Close the window to exit" << std::endl;

  size_t rawlogIndex = 0;
  bool paused = false;

  mrpt::Clock::time_point lastObsTim;

  while (win.isOpen())
  {
    win.get3DSceneAndLock();

    // Get and process one observation:
    if (rawlogIndex < dataset.size() && !paused)
    {
      mrpt::obs::CObservation3DRangeScan::Ptr obs;

      if (dataset.getType(rawlogIndex) == mrpt::obs::CRawlog::etObservation &&
          (obs = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(
               dataset.getAsObservation(rawlogIndex))))
      {
        bool poseOk = false;
        mrpt::poses::CPose3D camPose;
        lastObsTim = obs->getTimeStamp();
        gt.interpolate(lastObsTim, camPose, poseOk);

        if (poseOk)
        {
          // set viz camera pose:
          glCamGroup->setPose(camPose);

          using namespace mrpt::literals;
          obs->sensorPose = mrpt::poses::CPose3D::FromYawPitchRoll(0.0_deg, -90.0_deg, 90.0_deg);

          // draw observation raw data:
          mrpt::maps::CColouredPointsMap colPts;

          mrpt::obs::T3DPointsProjectionParams pp;
          pp.takeIntoAccountSensorPoseOnRobot = true;
          obs->unprojectInto(colPts, pp);

          glObsPts->loadFromPointsMap(&colPts);

          if (!glCamFrustrumDone)
          {
            glCamFrustrumDone = true;
            auto glFrustrum =
                mrpt::opengl::CFrustum::Create(obs->cameraParamsIntensity, 1e-3 /*focalDistScale*/);
            glFrustrum->setPose(obs->sensorPose);
            glCamGroup->insert(glFrustrum);
          }

          // update the voxel map:
          map.insertObservation(*obs, camPose);

          // Update the voxel map visualization:
          static int decimUpdateViz = 0;
          if (decimUpdateViz++ > 20)
          {
            decimUpdateViz = 0;
            map.renderingOptions.generateFreeVoxels = false;
            map.getAsOctoMapVoxels(*glVoxels);
          }

          // RGB view:
          if (obs->hasIntensityImage)
          {
            glViewRGB->setImageView(obs->intensityImage);
          }
        }
      }
      rawlogIndex++;
    }

    win.unlockAccess3DScene();

    if (win.keyHit())
    {
      const unsigned int k = win.getPushedKey();

      switch (k)
      {
        case ' ':
          paused = !paused;
          break;
      };
    }

    win.addTextMessage(
        5, 5,
        mrpt::format(
            "Timestamp: %s RawlogIndex: %zu ActiveVoxelCells: %zu",
            mrpt::system::dateTimeLocalToString(lastObsTim).c_str(), rawlogIndex,
            map.grid().activeCellsCount()),
        1 /*id*/);

    win.repaint();

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(10ms);
  };
}

int main(int argc, char** argv)
{
  try
  {
    if (argc != 3 && argc != 4)
      throw std::invalid_argument(
          "Usage: PROGRAM <PATH_TO_TUM_DATASET.rawlog> "
          "<VOXELMAP_RESOLUTION> [<VOXELMAP_MAX_RANGE>]");

    const std::string gtFile =
        mrpt::system::pathJoin({mrpt::system::extractFileDirectory(argv[1]), "groundtruth.txt"});

    double VOXELMAP_MAX_RANGE = 5.0;
    if (argc == 4)
    {
      VOXELMAP_MAX_RANGE = std::stod(argv[3]);
    }

    TestVoxelMapFromTUM(argv[1], gtFile, std::stod(argv[2]), VOXELMAP_MAX_RANGE);

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cout << "Exception: " << e.what() << std::endl;
    return -1;
  }
}
