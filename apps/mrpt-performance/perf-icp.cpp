/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/system/filesystem.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace std;

// ------------------------------------------------------
//				Benchmark: A whole ICP-SLAM run
// ------------------------------------------------------
double icp_test_1(int a1, int a2)
{
	using namespace std::string_literals;
	const std::string rawlog_file = mrpt::system::getShareMRPTDir() +
		"datasets/2006-01ENE-21-SENA_Telecom Faculty_one_loop_only.rawlog"s;

	if (!mrpt::system::fileExists(rawlog_file)) return 1;

	CTicTac tictac;

	int step = 0;
	size_t rawlogEntry = 0;
	mrpt::io::CFileGZInputStream rawlogFile(rawlog_file);

	TSetOfMetricMapInitializers metricMapsOpts;

	const bool use_grid = a1 != 0;

	if (use_grid)
	{
		mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
		def.resolution = 0.05f;
		metricMapsOpts.push_back(def);
	}
	else
	{
		mrpt::maps::CSimplePointsMap::TMapDefinition def;
		def.insertionOpts.minDistBetweenLaserPoints = 0.03f;
		metricMapsOpts.push_back(def);
	}

	double insertionLinDistance = 0.75;
	double insertionAngDistance = 30.0_deg;

	CICP::TConfigParams icpOptions;

	icpOptions.maxIterations = 40;

	// ---------------------------------
	//		Constructor
	// ---------------------------------
	CMetricMapBuilderICP mapBuilder;

	mapBuilder.ICP_options.mapInitializers = metricMapsOpts;

	mapBuilder.ICP_options.insertionLinDistance = insertionLinDistance;
	mapBuilder.ICP_options.insertionAngDistance = insertionAngDistance;

	mapBuilder.ICP_options.matchAgainstTheGrid = use_grid;
	mapBuilder.ICP_params = icpOptions;

	// Start with an empty map:
	mapBuilder.initialize(CSimpleMap());

	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
	mapBuilder.setVerbosityLevel(mrpt::system::LVL_ERROR);
	mapBuilder.options.enableMapUpdating = true;

	// ----------------------------------------------------------
	//						Map Building
	// ----------------------------------------------------------
	CActionCollection::Ptr action;
	CSensoryFrame::Ptr observations;

	auto arch = mrpt::serialization::archiveFrom(rawlogFile);
	for (;;)
	{
		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (!CRawlog::readActionObservationPair(
				arch, action, observations, rawlogEntry))
			break;	// file EOF

		// Execute:
		mapBuilder.processActionObservation(*action, *observations);

		step++;
		//		printf("\n---------------- STEP %u | RAWLOG ENTRY %u
		//----------------\n",step, (unsigned)rawlogEntry);

		// Free memory:
		action.reset();
		observations.reset();
	}

#if 0
	mapBuilder.getCurrentlyBuiltMetricMap().saveMetricMapRepresentationToFile( format("icp_%i_result",a1) );
#endif

	if (!step) step++;

	return tictac.Tac() / step;
}

// ------------------------------------------------------
// register_tests_icpslam
// ------------------------------------------------------
void register_tests_icpslam()
{
	lstTests.emplace_back(
		"icp-slam (match points): Run with sample dataset", icp_test_1, 0);
	lstTests.emplace_back(
		"icp-slam (match grid): Run with sample dataset", icp_test_1, 1);
}
