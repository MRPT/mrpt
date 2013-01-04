/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#ifndef CObservationBeaconRanges_H
#define CObservationBeaconRanges_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationBeaconRanges, CObservation, OBS_IMPEXP  )

	/** Declares a class derived from "CObservation" that represents one (or more) range measurements to labeled beacons.
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationBeaconRanges : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationBeaconRanges )

	 public:
		/** Default constructor.
		 */
		CObservationBeaconRanges( );

		 /** Information about the sensor:
		  */
		float	minSensorDistance, maxSensorDistance;

		/** The "sigma" of the sensor, assuming a zero-mean Gaussian noise model.
		  */
		float	stdError;

		/** Each one of the measurements:
			*/
		struct OBS_IMPEXP TMeasurement
		{
			TMeasurement() : sensorLocationOnRobot(), sensedDistance(0),beaconID(INVALID_BEACON_ID)
			{}

			/** The position of the sensor on the robot.
			  */
			CPoint3D		sensorLocationOnRobot;

			/** The sensed range itself (in meters).
			  */
			float			sensedDistance;

			/** The ID of the sensed beacon (or INVALID_BEACON_ID)
			  */
			int32_t			beaconID;
		};

		/** The list of observed ranges:
		  */
		std::deque<TMeasurement> sensedData;

		/** The (X,Y,PHI) pose estimated by the UWB software, for comparison purposes (Added in streamming version 1)
		  */
		CPose2D					auxEstimatePose;


		 /** Prints out the contents of the object.
		   */
		 void  debugPrintOut();

		/** A general method to retrieve the sensor pose on the robot.
		  *  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const;


		/** A general method to change the sensor pose on the robot.
		  *  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose );

		/** Easy look-up into the vector sensedData, returns the range for a given beacon, or 0 if the beacon is not observed.
		  */
		float getSensedRangeByBeaconID(int32_t beaconID);

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
