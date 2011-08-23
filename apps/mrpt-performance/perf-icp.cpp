/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>
#include <mrpt/slam.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
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
		TMetricMapInitializer ini;
		ini.metricMapClassType = CLASS_ID( COccupancyGridMap2D );
		ini.occupancyGridMap2D_options.resolution = 0.05;
		metricMapsOpts.push_back( ini );
	}
	else
	{
		TMetricMapInitializer ini;
		ini.metricMapClassType = CLASS_ID( CSimplePointsMap );
		ini.pointsMapOptions_options.insertionOpts.minDistBetweenLaserPoints = 0.03;
		metricMapsOpts.push_back( ini );
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
	mapBuilder.options.verbose					= false;
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


