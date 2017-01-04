/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CDetectorDoorCrossing_H
#define CDetectorDoorCrossing_H

#include <mrpt/utils/COutputLogger.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CActionRobotMovement2D.h>

#include <mrpt/detectors/link_pragmas.h>

namespace mrpt
{
namespace detectors
{

	/**
     *
	 * \sa CPointsMap   \ingroup mrpt_slam_grp
	 */
	class DETECTORS_IMPEXP CDetectorDoorCrossing : public mrpt::utils::COutputLogger
	{
	public:
		/** The constructor. See options for customizing the default parameters.
		  *
		  */
		CDetectorDoorCrossing() ;

		/** In this structure parameters can be changed to customize the
		  *   behaviour of this algorithm.
		  */
		struct DETECTORS_IMPEXP TOptions
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
		struct DETECTORS_IMPEXP TDoorCrossingOutParams
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

			mrpt::maps::CSimplePointsMap pointsMap;
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
			mrpt::obs::CActionRobotMovement2D & in_poseChange,
			mrpt::obs::CSensoryFrame          & in_sf,
			TDoorCrossingOutParams             & out_estimation
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
		mrpt::obs::CRawlog  lastObs;

		/** Entropy of current, and last "map patchs". */
		mrpt::maps::COccupancyGridMap2D::TEntropyInfo entropy, lastEntropy;
		bool  lastEntropyValid;



	};

	} // End of namespace
} // End of namespace

#endif
