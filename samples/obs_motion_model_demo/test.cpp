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
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CSetOfLines.h>
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
	mrpt::gui::CDisplayWindow3D win("Motion model uncertainty demo", 800, 600);

	auto glPartsGroup = mrpt::opengl::CSetOfObjects::Create();
	auto glLidar = mrpt::opengl::CPlanarLaserScan::Create();
	auto glOdoTrack = mrpt::opengl::CSetOfLines::Create();
	glOdoTrack->appendLine(0, 0, 0, 0, 0, 0);
	glOdoTrack->setColor_u8(0x00, 0x00, 0x00);
	{
		auto& scene = win.get3DSceneAndLock();
		scene->insert(mrpt::opengl::CGridPlaneXY::Create());
		scene->insert(glPartsGroup);
		scene->insert(glLidar);
		scene->insert(glOdoTrack);
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
