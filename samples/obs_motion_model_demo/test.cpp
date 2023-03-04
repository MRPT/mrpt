/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/pose_pdfs.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>

#include <iostream>

// demo rawlog path:
#include <mrpt/examples_config.h>

constexpr size_t NUM_PARTICLES = 100;

void DemoMotionModel(int argc, const char** argv)
{
	using namespace std::string_literals;

	mrpt::random::getRandomGenerator().randomize();

	// Default dataset, or override with user cli one:
	std::string datasetFile = MRPT_EXAMPLES_BASE_DIRECTORY +
		"../share/mrpt/datasets/intel_2003_partial.rawlog.gz"s;
	if (argc > 1) datasetFile = argv[1];

	std::string legendText;
	if (argc > 2) legendText = argv[2];

	ASSERT_FILE_EXISTS_(datasetFile);

	mrpt::obs::CRawlog dataset;
	dataset.loadFromRawLogFile(datasetFile);

	std::cout << "Read dataset: " << dataset.size() << " entries.\n";

	size_t position = 0;

	mrpt::obs::CActionCollection::Ptr actions;
	mrpt::obs::CSensoryFrame::Ptr sf;

	mrpt::poses::CPose2D deadReckoning;
	mrpt::poses::CPosePDFParticles parts;

	parts.resetDeterministic({0, 0, 0}, NUM_PARTICLES);

	// gui:
	mrpt::gui::CDisplayWindow3D win("Motion model uncertainty demo", 500, 1000);

	auto glPartsGroup = mrpt::opengl::CSetOfObjects::Create();
	auto glLidar = mrpt::opengl::CPlanarLaserScan::Create();
	auto glOdoTrack = mrpt::opengl::CSetOfLines::Create();
	mrpt::opengl::Viewport::Ptr glBottomView;
	glOdoTrack->appendLine(0, 0, 0, 0, 0, 0);
	glOdoTrack->setColor_u8(0x00, 0x00, 0x00);
	{
		auto& scene = win.get3DSceneAndLock();
		scene->insert(mrpt::opengl::CGridPlaneXY::Create());
		scene->insert(glPartsGroup);
		scene->insert(glLidar);
		scene->insert(glOdoTrack);

		// Create the two views:
		auto glMainView = scene->getViewport();
		glBottomView = scene->createViewport("bottom");

		glMainView->setViewportPosition(0, 0.5, 1.0, 0.5);
		glBottomView->setViewportPosition(0.01, 0.01, 0.98, 0.48);
		glBottomView->setCloneView("main");

		auto& glCloseCam = glBottomView->getCamera();
		glCloseCam.setAzimuthDegrees(90.0f);
		glCloseCam.setElevationDegrees(90.0f);
		glCloseCam.setZoomDistance(1.5f);

		win.setCameraAzimuthDeg(90.0f);
		win.setCameraElevationDeg(90.0f);
		win.setCameraZoom(15.0f);
		win.setCameraPointingToPoint(1.0f, -2.0f, .0f);

		win.addTextMessage(5, 5, legendText);

		win.unlockAccess3DScene();
	}

	std::optional<double> lastObsTime;
	bool paused = false;

	// iterate over the dataset:
	while (position + 1 < dataset.size() &&
		   dataset.getActionObservationPair(actions, sf, position))
	{
		const auto actMotion =
			actions->getActionByClass<mrpt::obs::CActionRobotMovement2D>();
		ASSERT_(actMotion);

		const auto obsLidar =
			sf->getObservationByClass<mrpt::obs::CObservation2DRangeScan>();
		ASSERT_(obsLidar);

		const double thisObsTime = mrpt::Clock::toDouble(obsLidar->timestamp);
		const double obsPeriod =
			std::max(1e-3, lastObsTime ? (thisObsTime - *lastObsTime) : .01);
		lastObsTime = thisObsTime;

		if (!paused)
		{
			// Accumulate the mean pose increments, as a gross pose estimation
			// (no filtering, no localization, no SLAM, just dead reckoning):
			deadReckoning += actMotion->poseChange->getMeanVal();

			// Propagate random samples:
			actMotion->prepareFastDrawSingleSamples();

			for (auto& part : parts.m_particles)
			{
				mrpt::poses::CPose2D odoSample;
				actMotion->fastDrawSingleSample(odoSample);

				part.d = part.d + odoSample.asTPose();
			}
		}

		const auto glParticles =
			parts.getAs3DObject<mrpt::opengl::CSetOfObjects::Ptr>();

		{
			win.get3DSceneAndLock();

			glPartsGroup->clear();
			glPartsGroup->insert(glParticles);

			glLidar->setScan(*obsLidar);
			glLidar->setPose(deadReckoning);

			glOdoTrack->appendLineStrip(
				deadReckoning.x(), deadReckoning.y(), 0.01);

			auto& glCloseCam = glBottomView->getCamera();
			glCloseCam.setPointingAt(deadReckoning.x(), deadReckoning.y(), .0);

			win.unlockAccess3DScene();
			win.forceRepaint();

			std::this_thread::sleep_for(
				std::chrono::microseconds(static_cast<int>(1e6 * obsPeriod)));
		}

		std::cout << "timestep: " << position
				  << " deadRecoking: " << deadReckoning << "\n";

		if (win.keyHit())
		{
			switch (win.getPushedKey())
			{
				case ' ': paused = !paused; break;
				case 'R':
				case 'r': win.grabImagesStart(); break;
			}
		}

		if (paused) position -= 2;
	}
}

int main(int argc, const char** argv)
{
	try
	{
		DemoMotionModel(argc, argv);
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
}
