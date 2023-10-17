/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/os.h>

#include <chrono>
#include <iostream>
#include <thread>

// ------------------------------------------------------
//				TestVoxelMapFromTUM
// ------------------------------------------------------
void TestVoxelMapFromTUM(
	const std::string& datasetRawlogFile, const std::string& groundTruthFile)
{
	// To find out external image files:
	mrpt::io::setLazyLoadPathBase(
		mrpt::obs::CRawlog::detectImagesDirectory(datasetRawlogFile));

	std::cout << "Loading dataset: " << datasetRawlogFile << "..." << std::endl;

	mrpt::obs::CRawlog dataset;
	dataset.loadFromRawLogFile(datasetRawlogFile);

	std::cout << "Done! " << dataset.size() << " entries." << std::endl;

	std::cout << "Loading GT path from: " << groundTruthFile << "..."
			  << std::endl;

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
				mrpt::math::CQuaternionDouble(
					gtData(i, 7), gtData(i, 4), gtData(i, 5), gtData(i, 6)),
				mrpt::math::TPoint3D(
					gtData(i, 1), gtData(i, 2), gtData(i, 3))));
	}

	mrpt::maps::CVoxelMap map(0.02);

	// map.insertionOptions.max_range = 5.0;  // [m]

	mrpt::gui::CDisplayWindow3D win("VoxelMap demo", 640, 480);

	auto gl_map = mrpt::opengl::COctoMapVoxels::Create();

	auto glCamGroup = mrpt::opengl::CSetOfObjects::Create();
	glCamGroup->insert(mrpt::opengl::stock_objects::CornerXYZSimple(0.3));
	auto glObsPts = mrpt::opengl::CPointCloudColoured::Create();
	glCamGroup->insert(glObsPts);
	bool glCamFrustrumDone = false;

	{
		mrpt::opengl::Scene::Ptr& scene = win.get3DSceneAndLock();

		{
			auto gl_grid =
				mrpt::opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
			gl_grid->setColor_u8(mrpt::img::TColor(0x80, 0x80, 0x80));
			scene->insert(gl_grid);
		}
		scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple());

		scene->insert(glCamGroup);

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

	size_t rawlogIndex = 0;

	mrpt::Clock::time_point lastObsTim;

	while (win.isOpen())
	{
		win.get3DSceneAndLock();

		// Get and process one observation:
		if (rawlogIndex < dataset.size())
		{
			mrpt::obs::CObservation3DRangeScan::Ptr obs;

			if (dataset.getType(rawlogIndex) ==
					mrpt::obs::CRawlog::etObservation &&
				(obs = std::dynamic_pointer_cast<
					 mrpt::obs::CObservation3DRangeScan>(
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
					obs->sensorPose = mrpt::poses::CPose3D::FromYawPitchRoll(
						0.0_deg, -90.0_deg, 90.0_deg);

					// draw observation raw data:
					mrpt::obs::T3DPointsProjectionParams pp;
					pp.takeIntoAccountSensorPoseOnRobot = true;
					obs->unprojectInto(*glObsPts, pp);

					if (!glCamFrustrumDone)
					{
						glCamFrustrumDone = true;
						auto glFrustrum = mrpt::opengl::CFrustum::Create(
							obs->cameraParamsIntensity,
							1e-3 /*focalDistScale*/);
						glFrustrum->setPose(obs->sensorPose);
						glCamGroup->insert(glFrustrum);
					}

					// update the voxel map:

					// Update the voxel map visualization:
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
				case 'g':
				case 'G':
					gl_map->showGridLines(!gl_map->areGridLinesVisible());
					break;
				case 'f':
				case 'F':
					gl_map->showVoxels(
						mrpt::opengl::VOXEL_SET_FREESPACE,
						!gl_map->areVoxelsVisible(
							mrpt::opengl::VOXEL_SET_FREESPACE));
					break;
				case 'o':
				case 'O':
					gl_map->showVoxels(
						mrpt::opengl::VOXEL_SET_OCCUPIED,
						!gl_map->areVoxelsVisible(
							mrpt::opengl::VOXEL_SET_OCCUPIED));
					break;
				case 'l':
				case 'L':
					gl_map->enableLights(!gl_map->areLightsEnabled());
					break;
			};
		}

		win.addTextMessage(
			5, 5,
			mrpt::format(
				"Commands: 'f' (freespace=%s) | 'o' (occupied=%s) | 'l' "
				"(lights=%s)",
				gl_map->areVoxelsVisible(mrpt::opengl::VOXEL_SET_FREESPACE)
					? "YES"
					: "NO",
				gl_map->areVoxelsVisible(mrpt::opengl::VOXEL_SET_OCCUPIED)
					? "YES"
					: "NO",
				gl_map->areLightsEnabled() ? "YES" : "NO"),
			0 /*id*/);

		win.addTextMessage(
			5, 20,
			mrpt::format(
				"Timestamp: %s RawlogIndex: %zu",
				mrpt::system::dateTimeLocalToString(lastObsTim).c_str(),
				rawlogIndex),
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
		if (argc != 3)
			throw std::invalid_argument(
				"Usage: PROGRAM <PATH_TO_TUM_DATASET.rawlog> "
				"<GROUND_TRUTH.txt>");

		TestVoxelMapFromTUM(argv[1], argv[2]);
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		return -1;
	}
}
