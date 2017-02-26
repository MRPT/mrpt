/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CRawlog.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
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
#ifdef MRPT_DATASET_DIR
	const string rawlog_file = MRPT_DATASET_DIR  "/2006-01ENE-21-SENA_Telecom Faculty_one_loop_only.rawlog";
	if (!mrpt::system::fileExists(rawlog_file))
		return 1;

	CTicTac	 tictac;

	int			step = 0;
	size_t		rawlogEntry = 0;
	CFileGZInputStream	rawlogFile( rawlog_file );

	TSetOfMetricMapInitializers  metricMapsOpts;

	const bool use_grid = a1!=0;

	if (use_grid)
	{
		COccupancyGridMap2D::TMapDefinition def;
		def.resolution = 0.05f;
		metricMapsOpts.push_back( def );
	}
	else
	{
		CSimplePointsMap::TMapDefinition def;
		def.insertionOpts.minDistBetweenLaserPoints = 0.03f;
		metricMapsOpts.push_back( def );
	}

	double insertionLinDistance = 0.75;
	double insertionAngDistance = DEG2RAD(30);

	CICP::TConfigParams  icpOptions;

	icpOptions.maxIterations = 40;


	// ---------------------------------
	//		Constructor
	// ---------------------------------
	CMetricMapBuilderICP mapBuilder;

	mapBuilder.ICP_options.mapInitializers = metricMapsOpts;

	mapBuilder.ICP_options.insertionLinDistance = insertionLinDistance;
	mapBuilder.ICP_options.insertionAngDistance = insertionAngDistance;

	mapBuilder.ICP_options.matchAgainstTheGrid = use_grid ;
	mapBuilder.ICP_params = icpOptions;

	// Start with an empty map:
	mapBuilder.initialize( CSimpleMap() );

	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
	mapBuilder.setVerbosityLevel( mrpt::utils::LVL_ERROR);
	mapBuilder.options.enableMapUpdating		= true;

	// ----------------------------------------------------------
	//						Map Building
	// ----------------------------------------------------------
	CActionCollectionPtr	action;
	CSensoryFramePtr		observations;

	for (;;)
	{
		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (! CRawlog::readActionObservationPair( rawlogFile, action, observations, rawlogEntry) )
			break; // file EOF

		// Execute:
		mapBuilder.processActionObservation( *action, *observations );

		step++;
//		printf("\n---------------- STEP %u | RAWLOG ENTRY %u ----------------\n",step, (unsigned)rawlogEntry);

		// Free memory:
		action.clear_unique();
		observations.clear_unique();
	}

#if 0
	mapBuilder.getCurrentlyBuiltMetricMap()->saveMetricMapRepresentationToFile( format("icp_%i_result",a1) );
#endif

	if (!step) step++;

	return tictac.Tac()/step;
#else
	return 1;
#endif
}

// ------------------------------------------------------
// register_tests_icpslam
// ------------------------------------------------------
void register_tests_icpslam()
{
	lstTests.push_back( TestData("icp-slam (match points): Run with sample dataset",icp_test_1,  0) );
	lstTests.push_back( TestData("icp-slam (match grid): Run with sample dataset",icp_test_1,  1) );
}


