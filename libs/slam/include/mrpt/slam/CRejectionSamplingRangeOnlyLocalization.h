/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
#ifndef CRejectionSamplingRangeOnlyLocalization_H
#define CRejectionSamplingRangeOnlyLocalization_H

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/bayes/CRejectionSamplingCapable.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		using namespace mrpt::poses;
		using namespace mrpt::math;

		class	CLandmarksMap;
		class   CObservationBeaconRanges;

		/** An implementation of rejection sampling for generating 2D robot pose from range-only measurements within a landmarks (beacons) map.
		 *    Before calling the method "rejectionSampling" to generate the samples, you must call "setParams".
		 *    It is assumed a planar scenario, where the robot is at a fixed height (default=0).
		 * \sa bayes::CRejectionSamplingCapable  \ingroup mrpt_slam_grp 
		 */
		class SLAM_IMPEXP CRejectionSamplingRangeOnlyLocalization : public bayes::CRejectionSamplingCapable<poses::CPose2D>
		{

		public:
			/** Constructor
			  */
			CRejectionSamplingRangeOnlyLocalization();

			/** Destructor
			  */
			virtual ~CRejectionSamplingRangeOnlyLocalization() { }

			/** The parameters used in the generation of random samples:
			  * \param beaconsMap The map containing the N beacons (indexed by their "beacon ID"s). Only the mean 3D position of the beacons is used, the covariance is ignored.
			  * \param observation An observation with, at least ONE range measurement.
			  * \param sigmaRanges The standard deviation of the "range measurement noise".
			  * \param robot_z The height of the robot on the floor (default=0). Note that the beacon sensor on the robot may be at a different height, according to data within the observation object.
			  * \param autoCheckAngleRanges Whether to make a simple check for potential good angles from the beacons to generate samples (disable to speed-up the preparation vs. making slower the drawn).
			  *  This method fills out the member "m_dataPerBeacon".
			  * \return true if at least ONE beacon has been successfully loaded, false otherwise. In this case do not call "rejectionSampling" or an exception will be launch, since there is no information to generate samples.
			  */
			bool setParams(
				const CLandmarksMap				&beaconsMap,
				const CObservationBeaconRanges	&observation,
				float							sigmaRanges,
				const CPose2D					&oldPose,
				float							robot_z = 0,
				bool							autoCheckAngleRanges = true);

		protected:
			/** Generates one sample, drawing from some proposal distribution.
			  */
			void RS_drawFromProposal( CPose2D &outSample );

			/** Returns the NORMALIZED observation likelihood (linear, not exponential!!!) at a given point of the state space (values in the range [0,1]).
			  */
			double RS_observationLikelihood( const CPose2D &x);

			/** Z coordinate of the robot.
			  */
			float		m_z_robot;

			float		m_sigmaRanges;
			CPose2D		m_oldPose;

			/** The index in "m_dataPerBeacon" used to draw samples (the rest will be used to evaluate the likelihood)
			  */
			size_t		m_drawIndex;

			/** Data for each beacon observation with a correspondence with the map.
			  */
			struct SLAM_IMPEXP TDataPerBeacon
			{
				TDataPerBeacon()  : sensorOnRobot(), beaconPosition(), radiusAtRobotPlane(0),minAngle(0),maxAngle(0)
				{}

				TPoint3D	sensorOnRobot;
				TPoint2D	beaconPosition;
				float		radiusAtRobotPlane;
				float		minAngle,maxAngle;
			};

			/** Data for each beacon observation with a correspondence with the map.
			  */
			std::deque<TDataPerBeacon>	m_dataPerBeacon;

		};

	} // End of namespace
} // End of namespace

#endif
