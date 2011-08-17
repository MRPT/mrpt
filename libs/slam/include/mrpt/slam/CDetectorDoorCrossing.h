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
#ifndef CDetectorDoorCrossing_H
#define CDetectorDoorCrossing_H

#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CActionRobotMovement2D.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
namespace slam
{

	/**
     *
	 * \sa CPointsMap   \ingroup mrpt_slam_grp
	 */
	class SLAM_IMPEXP CDetectorDoorCrossing : public utils::CDebugOutputCapable
	{
	public:
		/** The constructor. See options for customizing the default parameters.
		  *
		  */
		CDetectorDoorCrossing() ;

		/** In this structure parameters can be changed to customize the
		  *   behaviour of this algorithm.
		  */
		struct SLAM_IMPEXP TOptions
		{
			TOptions() : windowSize(5), gridResolution(0.2f) //, gridUpdateFactor(0.5)
			{
			}

			/** The window size, in (action,observations) pairs;min. value is 2.
			  */
			unsigned int	windowSize;

			float			gridResolution; //,gridUpdateFactor;

		} options;


		/** A structure used as output in this method.
		  * \sa process
		  */
		struct SLAM_IMPEXP TDoorCrossingOutParams
		{
			TDoorCrossingOutParams() :
				enoughtInformation(false),
				doorCrossingLikelihood(0),
				informationGain(0),
				cumulativeTurning(0),
				pointsMap()
			{
			};

			/** If this is false, all other output fields must not be
			  *   taken into account since there is not yet enought information
			  *   to fill the required observations window size.
			  */
			bool	enoughtInformation;

			/** The likelihood of having just entering a new room, in
			  *   the range [0,1]
			  */
			float	doorCrossingLikelihood;

			/** The gain in information produced by the last observation, in "bits".
			  */
			float	informationGain;

			/** The cumulative turning of the robot in radians for the movements in the "window"
			  */
			float	cumulativeTurning;

			CSimplePointsMap	pointsMap;
		};

		/** The main method, where a new action/observation pair is added to the list.
		  *   Here the list of old observations is updated, and a value with the probability
		  *   of having pass a door is returned.
		  * \param in_poseChange The odometry (or any other meanway) based change in the robot pose since last observation to this one.
		  * \param in_sf The observations.
		  * \param out_estimation The estimation results.
		  *
		  * \sa TDoorCrossingOutParams
		  */
		void  process(
				CActionRobotMovement2D	&in_poseChange,
				CSensoryFrame			&in_sf,
				TDoorCrossingOutParams	&out_estimation
				);

		/** Reset the detector, i.e. it erases all previous observations.
		  */
		void  clear();

	private:
		/** The last observations and consecutive actions are stored here:
		  *   Indexes (0,1) is the earlier (act,obs) pair, and the lastest pair
		  *    is in indexes ((M-1)*2,(M-1)*2-1).
		  *    Always contains (Action, Observation) pairs, in that order.
		  */
		CRawlog		lastObs;

		/** Entropy of current, and last "map patchs".
		  */
		COccupancyGridMap2D::TEntropyInfo	entropy, lastEntropy;
		bool				lastEntropyValid;



	};

	} // End of namespace
} // End of namespace

#endif
