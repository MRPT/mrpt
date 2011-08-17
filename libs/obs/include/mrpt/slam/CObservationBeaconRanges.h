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
