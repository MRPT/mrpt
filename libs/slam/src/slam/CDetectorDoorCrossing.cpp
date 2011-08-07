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

#include <mrpt/slam.h>  // Precompiled header



#include <mrpt/slam/CDetectorDoorCrossing.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/poses/CPosePDF.h>

using namespace mrpt;
using namespace mrpt::slam; using namespace mrpt::utils; using namespace mrpt::poses;


/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CDetectorDoorCrossing::CDetectorDoorCrossing()  :
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
	TMetricMapInitializer				mapElement;

	mapElement.metricMapClassType = CLASS_ID( CSimplePointsMap );
	mapInitializer.push_back( mapElement );

	mapElement.metricMapClassType = CLASS_ID( COccupancyGridMap2D );
	mapElement.occupancyGridMap2D_options.resolution = options.gridResolution;
	mapInitializer.push_back( mapElement );

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

