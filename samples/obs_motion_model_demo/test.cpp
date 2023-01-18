/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>

#include <iostream>

// demo rawlog path:
#include <mrpt/examples_config.h>

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

	while (position + 1 < dataset.size() &&
		   dataset.getActionObservationPair(actions, sf, position))
	{
		const auto actMotion =
			actions->getActionByClass<mrpt::obs::CActionRobotMovement2D>();
		ASSERT_(actMotion);

		const auto obsLidar =
			sf->getObservationByClass<mrpt::obs::CObservation2DRangeScan>();
		ASSERT_(obsLidar);

		// Accumulate the mean pose increments, as a gross pose estimation (no
		// filtering, no localization, no SLAM, just dead reckoning):
		deadReckoning += actMotion->poseChange->getMeanVal();

		std::cout << "timestep: " << position
				  << " deadRecoking: " << deadReckoning << "\n";
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
