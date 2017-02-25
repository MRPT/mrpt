/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "detectors-precomp.h"   // Precompiled headers

#include <mrpt/detectors/CDetectorDoorCrossing.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/poses/CPosePDF.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::detectors;
using namespace mrpt::utils;
using namespace mrpt::poses;



/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CDetectorDoorCrossing::CDetectorDoorCrossing()  :
	COutputLogger("CDetectorDoorCrossing"),
	options(),
	lastObs(),
	entropy(),
	lastEntropy(),
	lastEntropyValid(false)
{
	clear();
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CDetectorDoorCrossing::clear()
{
	lastObs.clear();
	lastEntropyValid = false;
}

/*---------------------------------------------------------------
						process
  ---------------------------------------------------------------*/
void  CDetectorDoorCrossing::process(
		CActionRobotMovement2D	&in_poseChange,
		CSensoryFrame			&in_sf,
		TDoorCrossingOutParams	&out_estimation
		)
{
	// Variables for generic use:
	size_t				i;

	out_estimation.cumulativeTurning = 0;

	MRPT_START

	// 1) Add new pair to the list:
	// -----------------------------------------
	lastObs.addAction( in_poseChange );
	lastObs.addObservations( in_sf );

	// 2) Remove oldest pair:
	// -----------------------------------------
	ASSERT_( options.windowSize > 1 );
	ASSERT_( (lastObs.size() % 2) == 0 );	// Assure even size

	while (lastObs.size()>options.windowSize*2)
	{
		lastObs.remove(0);
		lastObs.remove(0);
	}

	if ( lastObs.size() < options.windowSize * 2 )
	{
		// Not enought old data yet:
		out_estimation.enoughtInformation = false;
		return;
	}

	// 3) Build an occupancy grid map with observations
	// -------------------------------------------------
	CPose2D					p, pos;

	TSetOfMetricMapInitializers			mapInitializer;

	{
		CSimplePointsMap::TMapDefinition def;
		mapInitializer.push_back( def );
	}
	{
		COccupancyGridMap2D::TMapDefinition def;
		def.resolution = options.gridResolution;
		mapInitializer.push_back( def );
	}

	CMultiMetricMap			auxMap( &mapInitializer );

	for (i=0;i<options.windowSize;i++)
	{
		CActionCollectionPtr	acts = lastObs.getAsAction( i*2+0 );
		CActionPtr				act = acts->get(0);

		ASSERT_( act->GetRuntimeClass()->derivedFrom( CLASS_ID( CActionRobotMovement2D ) ) )
		CActionRobotMovement2DPtr action = CActionRobotMovement2DPtr( act );

		action->poseChange->getMean(pos);

		out_estimation.cumulativeTurning+= fabs(pos.phi());

		// Get the cumulative pose for the current observation:
		p = p + pos;

		// Add SF to the grid map:
		CSensoryFramePtr	sf = lastObs.getAsObservations( i*2+1 );
		CPose3D				pose3D(p);
		sf->insertObservationsInto( &auxMap, &pose3D );
	}

	// 4) Compute the information differece between this
	//      "map patch" and the previous one:
	// -------------------------------------------------------
	auxMap.m_gridMaps[0]->computeEntropy( entropy );

	if (!lastEntropyValid)
	{
		out_estimation.enoughtInformation = false;
	}
	else
	{
		// 5) Fill output data
		// ---------------------------------
		out_estimation.enoughtInformation = true;


		out_estimation.informationGain = entropy.I - lastEntropy.I;
		out_estimation.pointsMap.copyFrom( *auxMap.m_pointsMaps[0] );
	}


	// For next iterations:
	lastEntropy = entropy;
	lastEntropyValid = true;

	MRPT_END

}

