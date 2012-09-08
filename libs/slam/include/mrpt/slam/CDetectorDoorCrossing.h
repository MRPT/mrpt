/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
